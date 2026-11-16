"""Rizon 4 interface for Flexiv RDK 1.9.4.1 (non-real-time Python control)."""

from __future__ import annotations

import logging
import time
import warnings
from collections.abc import Mapping
from contextlib import contextmanager
from importlib.metadata import version
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .config import RobotConfig, integer, pose_vector, positive, vector


class Bestman_Real_Flexiv:
    """Single Rizon 4, seven joints, no external axes.

    Connect by serial number, e.g. ``Bestman_Real_Flexiv('Rizon4-123456')``.
    Construction reads information but does not enable or move the robot.
    All poses are [x, y, z, qw, qx, qy, qz] vectors, matching RDK directly.
    ``robot`` is dependency injection for offline interface tests, not a simulator.
    """

    def __init__(
        self, robot_sn, frequency=None, *, network_interface_whitelist=None, config=None, robot=None
    ):
        self.config = config if isinstance(config, RobotConfig) else RobotConfig.load(config)
        self.frequency = positive(self.config.frequency if frequency is None else frequency, "frequency")
        if not 1 <= self.frequency <= 100:
            raise ValueError("Python NRT command frequency must be in [1, 100] Hz")
        if not isinstance(robot_sn, str) or not robot_sn.replace(" ", "").startswith("Rizon4-"):
            raise ValueError("Use a Rizon 4 serial number such as Rizon4-123456, not robot/local IPs")
        installed = version("flexivrdk")
        if installed != "1.9.4.1":
            raise RuntimeError(
                f"Rizon 4 requires the tested flexivrdk==1.9.4.1; found {installed}. "
                "RDK 2.x targets Enlight, not Rizon 4."
            )
        import flexivrdk

        if not all(
            hasattr(flexivrdk.Robot, name) for name in ("SwitchMode", "SetForceControlAxis", "states")
        ):
            raise RuntimeError(
                "An old flexivrdk binary shadows the installed wheel; remove legacy SDK PYTHONPATH"
            )

        self.rdk = flexivrdk
        self.mode = flexivrdk.Mode
        self.log = logging.getLogger(__name__)
        self.robot = (
            robot if robot is not None else flexivrdk.Robot(robot_sn, network_interface_whitelist or [])
        )
        info = self.robot.info()
        if info.model_name.replace(" ", "").lower() != "rizon4" or info.DoF != 7 or info.DoF_e:
            raise ValueError(
                "This configuration supports Rizon 4 with exactly seven joints and no external axes"
            )
        self.gripper = None
        self.robot_states = self.robot.states()
        self._robot_chain = None
        self._force_axes = None
        self._force_frame = None
        self._contact_wrench = None
        self._passive_force = None

    @property
    def robot_chain(self):
        if self._robot_chain is None:
            from ikpy.chain import Chain

            path = Path(__file__).resolve().parents[1] / "Asset/flexiv_rizon4_kinematics.urdf"
            self._robot_chain = Chain.from_urdf_file(
                str(path), active_links_mask=[False] + [True] * 7 + [False]
            )
        return self._robot_chain

    @property
    def active_joints(self):
        return [link for link in self.robot_chain.links if link.joint_type in ("revolute", "prismatic")]

    def _check_ready(self):
        if not self.robot.connected():
            raise RuntimeError("Robot connection is lost")
        if self.robot.fault():
            raise RuntimeError("Robot fault; stop and inspect before clearing it")
        if not self.robot.operational():
            raise RuntimeError("Robot is not operational")

    def _switch_mode(self, mode):
        self._check_ready()
        if self.robot.mode() != mode:
            self.robot.SwitchMode(mode)
            self._force_axes = None
            self._force_frame = None
            self._contact_wrench = None
            self._passive_force = None

    @contextmanager
    def _control(self):
        """Stop on command failure or interruption; preserve the original exception."""
        try:
            yield
        except BaseException:
            try:
                self.stop()
            except Exception:
                self.log.exception("Robot Stop() also failed")
            raise

    def initialize_robot(self, timeout=30.0):
        """Clear faults, enable, then wait at most timeout seconds. Raise on failure."""
        timeout = positive(timeout, "timeout")
        with self._control():
            if self.robot.fault() and not self.robot.ClearFault():
                raise RuntimeError("Robot fault could not be cleared")
            self.robot.Enable()
            deadline = time.monotonic() + timeout
            while not self.robot.operational():
                if self.robot.fault():
                    raise RuntimeError("Robot fault while enabling")
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        "Robot did not become operational; check E-stop and Auto (Remote) mode"
                    )
                time.sleep(0.05)
        return True

    def stop(self):
        """Blocking stop and transition to IDLE. Does not disable the servos."""
        try:
            self.robot.Stop()
        finally:
            self._force_axes = self._force_frame = self._contact_wrench = None
            self._passive_force = None

    def update_robot_states(self):
        self.robot_states = self.robot.states()
        return self.robot_states

    def execute_primitive(self, name, parameters=None, *, wait=True, timeout=30.0, block_until_started=True):
        """Execute SDK primitive parameters; wait=False allows state-driven transitions.

        Parameters use the primitive's native schema/units. For robot pose streaming,
        use the servo methods, whose pose inputs are always [x,y,z,qw,qx,qy,qz].
        """
        timeout = positive(timeout, "timeout")
        self._boolean(wait, "wait")
        self._boolean(block_until_started, "block_until_started")
        if wait and not block_until_started:
            raise ValueError("Waiting for completion requires block_until_started=True")
        with self._control():
            self._switch_mode(self.mode.NRT_PRIMITIVE_EXECUTION)
            self.robot.ExecutePrimitive(name, {} if parameters is None else parameters, block_until_started)
            if wait:
                self.wait_for_primitive(timeout=timeout)

    def wait_for_primitive(self, state="terminated", value=True, timeout=30.0):
        """Wait for a primitive state, e.g. reachedTarget=1, without assuming termination."""
        timeout = positive(timeout, "timeout")
        with self._control():
            self._require_mode(self.mode.NRT_PRIMITIVE_EXECUTION)
            deadline = time.monotonic() + timeout
            while True:
                self._check_ready()
                states = self.robot.primitive_states()
                if state in states and states[state] == value:
                    return
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Primitive did not reach {state}={value!r}")
                time.sleep(0.02)

    def execute_plan(
        self, name, *, wait=True, timeout=60.0, continue_execution=False, block_until_started=True
    ):
        """Run a plan by name or zero-based index; optionally return after starting.

        continue_execution=True explicitly allows the plan to outlive a lost RDK
        connection. Robot faults still stop it. Default is False.
        """
        timeout = positive(timeout, "timeout")
        for key, flag in (
            ("wait", wait),
            ("continue_execution", continue_execution),
            ("block_until_started", block_until_started),
        ):
            self._boolean(flag, key)
        if not isinstance(name, str):
            name = integer(name, "plan index", lower=0)
        if wait and not block_until_started:
            raise ValueError("Waiting for completion requires block_until_started=True")
        with self._control():
            self._switch_mode(self.mode.NRT_PLAN_EXECUTION)
            self.robot.ExecutePlan(name, continue_execution, block_until_started)
            if wait:
                deadline = time.monotonic() + timeout
                while self.robot.busy():
                    self._check_ready()
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"Plan {name} timed out")
                    time.sleep(0.02)
                self._check_ready()

    def go_home(self, timeout=60.0):
        self.execute_plan("PLAN-Home", timeout=timeout)

    def zero_force_torque_sensors(self, timeout=30.0):
        """Run only at rest, with correct tool mass/CoM and no environmental contact."""
        self.execute_primitive("ZeroFTSensor", timeout=timeout)

    def get_tool(self):
        """Return the active controller tool name and its RDK ToolParams."""
        tool = self.rdk.Tool(self.robot)
        return tool.name(), tool.params()

    def configure_tool(self, name, *, mass, center_of_mass, inertia, tcp_pose):
        """Add/update and activate measured tool parameters while IDLE.

        CoM is in flange coordinates (m); inertia at CoM is
        [Ixx,Iyy,Izz,Ixy,Ixz,Iyz] (kg m²); tcp_pose is flange-relative [x, y, z, qw, qx, qy, qz].
        This changes controller configuration, so supply actual measured values.
        """
        if not isinstance(name, str) or not name.strip() or name == "Flange":
            raise ValueError("Use a nonempty custom tool name, not Flange")
        mass = vector([mass], 1, "mass", lower=0)[0]
        com = vector(center_of_mass, 3, "center_of_mass")
        inertia = vector(inertia, 6, "inertia")
        xx, yy, zz, xy, xz, yz = inertia
        moments = np.linalg.eigvalsh([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        if moments[0] < -1e-12 or moments[2] > moments[0] + moments[1] + 1e-12:
            raise ValueError("Tool inertia must be positive semidefinite and physically consistent")
        tcp_pose = pose_vector(tcp_pose, "tcp_pose")
        params = self.rdk.ToolParams()
        params.mass, params.CoM, params.inertia = mass, com, inertia
        params.tcp_location = tcp_pose
        with self._control():
            self.stop()
            self._check_ready()
            tool = self.rdk.Tool(self.robot)
            if tool.exist(name):
                tool.Update(name, params)
            else:
                tool.Add(name, params)
            tool.Switch(name)
        # Keep offline FK/IK consistent with the newly selected controller TCP.
        from dataclasses import replace

        self.config = replace(self.config, flange_to_tcp=tuple(params.tcp_location))

    def get_joint_bounds(self):
        info = self.robot.info()
        return list(zip(info.q_min, info.q_max))

    def get_arm_id(self):
        return self.robot.info().serial_num

    def get_dof(self):
        return self.robot.info().DoF

    def get_joint_idx(self):
        return list(range(self.get_dof()))

    def get_links_info(self):
        """Offline URDF terminal link is the flange, not the controller's active TCP."""
        return self.robot_chain.links[-1].name

    def print_arm_jointInfo(self):
        for i, joint in enumerate(self.active_joints):
            print(i, joint.name, joint.bounds)

    def print_robot_info(self):
        print(self.robot.info())

    def get_current_joint_values(self):
        return list(self.update_robot_states().q)

    def get_current_eef_pose(self):
        """Return a copied WORLD [x, y, z, qw, qx, qy, qz] list (metres)."""
        return list(self.update_robot_states().tcp_pose)

    def get_tcp_wrench(self, frame="WORLD", *, raw=False):
        """Measured external [Fx,Fy,Fz,Mx,My,Mz], in N and Nm, at the TCP."""
        frame = self._frame_name(frame).lower()
        return list(getattr(self.update_robot_states(), f"ext_wrench_in_{frame}" + ("_raw" if raw else "")))

    def get_joint_torques(self, *, external=False):
        states = self.update_robot_states()
        return list(states.tau_ext if external else states.tau)

    def _joints(self, values):
        values = vector(values, 7, "joint values")
        if any(not lo <= value <= hi for value, (lo, hi) in zip(values, self.get_joint_bounds())):
            raise ValueError("Joint target exceeds controller joint limits")
        return values

    def _validate_joint_limits(self, values):
        try:
            self._joints(values)
            return True
        except ValueError:
            return False

    def move_arm_to_joint_values(
        self, joint_values, target_vel=None, target_acc=None, MAX_VEL=None, MAX_ACC=None, *, impedance=False
    ):
        """Send an NRT setpoint; call wait_for_joints for blocking motion.

        RDK removed target acceleration from SendJointPosition. Legacy zero input
        is accepted with a warning; nonzero input is rejected, never reinterpreted.
        """
        q = self._joints(joint_values)
        dq = vector([0.0] * 7 if target_vel is None else target_vel, 7, "target_vel")
        if target_acc is not None:
            if any(vector(target_acc, 7, "target_acc")):
                raise ValueError("RDK 1.9 SendJointPosition has no target_acc argument; use MAX_ACC")
            warnings.warn("target_acc is obsolete in RDK 1.9; omit it", DeprecationWarning, stacklevel=2)
        limits = self.robot.info().dq_max
        vmax = vector(
            [self.config.joint_velocity_scale * v for v in limits] if MAX_VEL is None else MAX_VEL,
            7,
            "MAX_VEL",
            lower=0,
            strict=True,
        )
        amax = vector(
            [self.config.joint_max_acc] * 7 if MAX_ACC is None else MAX_ACC,
            7,
            "MAX_ACC",
            lower=0,
            strict=True,
        )
        if any(v > limit or abs(d) > v for v, limit, d in zip(vmax, limits, dq)):
            raise ValueError("Joint velocity exceeds the command or controller limit")
        with self._control():
            self._switch_mode(self.mode.NRT_JOINT_IMPEDANCE if impedance else self.mode.NRT_JOINT_POSITION)
            self.robot.SendJointPosition(q, dq, vmax, amax)

    @staticmethod
    def _frame_name(frame):
        if not isinstance(frame, str) or frame.upper() not in ("WORLD", "TCP"):
            raise ValueError("frame must be WORLD or TCP")
        return frame.upper()

    def configure_force_control(
        self,
        axes,
        *,
        frame="WORLD",
        frame_transform=None,
        passive=False,
        max_linear_velocity=None,
        contact_wrench=None,
        super_primitive=False,
    ):
        """Configure once before sending hybrid/force commands.

        Axes: [X,Y,Z,Rx,Ry,Rz]. True selects explicit force/moment control.
        super_primitive=True selects NRT_SUPER_PRIMITIVE for onboard waypoint batches.
        frame_transform: [x,y,z,qw,qx,qy,qz] relative to the selected root frame.
        Active control is closed-loop; passive is open-loop wrench feedforward.
        Mode changes discard this wrapper's configuration: configure again afterwards.
        """
        self._boolean(super_primitive, "super_primitive")
        axes = list(axes)
        if len(axes) != 6 or any(type(a) is not bool for a in axes):
            raise ValueError("axes must contain exactly six booleans")
        frame = self._frame_name(frame)
        if type(passive) is not bool:
            raise ValueError("passive must be a boolean")
        transform = pose_vector(
            [0, 0, 0, 1, 0, 0, 0] if frame_transform is None else frame_transform,
            "frame_transform",
        )
        vmax = vector(
            self.config.force_axis_max_linear_velocity
            if max_linear_velocity is None
            else max_linear_velocity,
            3,
            "force-axis velocity",
            lower=0.005,
            upper=2.0,
        )
        limits = vector(
            self.config.max_contact_wrench if contact_wrench is None else contact_wrench,
            6,
            "contact_wrench",
            lower=0,
            allow_inf=True,
        )
        if any(axes[3:]) and any(np.isfinite(limits)):
            raise ValueError("Moment control cannot use contact-wrench regulation; explicitly pass [inf]*6")
        with self._control():
            # RDK requires IDLE for changing passive force control, even when false.
            self.stop()
            self._check_ready()
            self.robot.SetPassiveForceControl(passive)
            self._switch_mode(
                self.mode.NRT_SUPER_PRIMITIVE if super_primitive else self.mode.NRT_CARTESIAN_MOTION_FORCE
            )
            self.robot.SetForceControlAxis([False] * 6, vmax)
            self.robot.SetMaxContactWrench(limits)
            self.robot.SetForceControlFrame(getattr(self.rdk.CoordType, frame), transform)
            self.robot.SetForceControlAxis(axes, vmax)
            self._force_axes, self._force_frame, self._contact_wrench = axes, frame, limits
            self._passive_force = passive

    def _require_cartesian(self, *, mode=None):
        self._check_ready()
        allowed = (
            (self.mode.NRT_CARTESIAN_MOTION_FORCE, self.mode.NRT_SUPER_PRIMITIVE) if mode is None else (mode,)
        )
        if self.robot.mode() not in allowed or self._force_axes is None:
            raise RuntimeError("Call configure_force_control() for the required Cartesian mode first")

    def set_cartesian_impedance(self, stiffness=None, damping_ratio=None):
        """Stiffness [N/m,Nm/rad], damping ratio [0.3,0.8]; None restores nominal."""
        nominal = self.robot.info().K_x_nom
        stiffness = vector(nominal if stiffness is None else stiffness, 6, "stiffness", lower=0)
        if any(k > n for k, n in zip(stiffness, nominal)):
            raise ValueError("Cartesian stiffness exceeds K_x_nom")
        damping = vector(
            [0.7] * 6 if damping_ratio is None else damping_ratio, 6, "damping", lower=0.3, upper=0.8
        )
        with self._control():
            self._require_cartesian()
            self.robot.SetCartesianImpedance(stiffness, damping)

    def set_joint_impedance(self, stiffness=None, damping_ratio=None):
        """Enter NRT joint impedance mode; stiffness Nm/rad, damping ratio [0.3,0.8]."""
        nominal = self.robot.info().K_q_nom
        stiffness = vector(nominal if stiffness is None else stiffness, 7, "stiffness", lower=0)
        if any(k > n for k, n in zip(stiffness, nominal)):
            raise ValueError("Joint stiffness exceeds K_q_nom")
        damping = vector(
            [0.7] * 7 if damping_ratio is None else damping_ratio, 7, "damping", lower=0.3, upper=0.8
        )
        with self._control():
            self._switch_mode(self.mode.NRT_JOINT_IMPEDANCE)
            self.robot.SetJointImpedance(stiffness, damping)

    def set_max_contact_wrench(self, wrench):
        """Limit motion-controlled axes only; this is not a global collision detector."""
        limits = vector(wrench, 6, "contact_wrench", lower=0, allow_inf=True)
        self._require_cartesian()
        if any(self._force_axes[3:]) and any(np.isfinite(limits)):
            raise ValueError("Moment control cannot use contact-wrench regulation")
        with self._control():
            self.robot.SetMaxContactWrench(limits)
            self._contact_wrench = limits

    def set_max_contact_torque(self, torques):
        limits = vector(torques, 7, "contact torques", lower=0)
        if any(value > maximum for value, maximum in zip(limits, self.robot.info().tau_max)):
            raise ValueError("Contact torque limit exceeds RobotInfo.tau_max")
        with self._control():
            self._check_ready()
            if self.robot.mode() != self.mode.NRT_JOINT_IMPEDANCE:
                raise RuntimeError("Contact torque regulation requires NRT_JOINT_IMPEDANCE")
            self.robot.SetMaxContactTorque(limits)

    def set_null_space_posture(self, joint_values):
        q = self._joints(joint_values)
        with self._control():
            self._require_cartesian()
            self.robot.SetNullSpacePosture(q)

    def set_null_space_objectives(self, linear=0.0, angular=0.0, tracking=0.5):
        values = vector([linear, angular, tracking], 3, "null-space weights", lower=0, upper=1)
        if values[2] < 0.1:
            raise ValueError("tracking must be in [0.1, 1]")
        with self._control():
            self._require_cartesian()
            self.robot.SetNullSpaceObjectives(*values)

    def _cartesian_arguments(
        self, pose, wrench, velocity, max_linear_vel, max_angular_vel, max_linear_acc, max_angular_acc
    ):
        return (
            pose_vector(pose, "goal_pose"),
            vector(wrench, 6, "wrench"),
            vector([0.0] * 6 if velocity is None else velocity, 6, "velocity"),
            positive(
                self.config.max_linear_vel if max_linear_vel is None else max_linear_vel, "max_linear_vel"
            ),
            positive(
                self.config.max_angular_vel if max_angular_vel is None else max_angular_vel, "max_angular_vel"
            ),
            positive(
                self.config.max_linear_acc if max_linear_acc is None else max_linear_acc, "max_linear_acc"
            ),
            positive(
                self.config.max_angular_acc if max_angular_acc is None else max_angular_acc, "max_angular_acc"
            ),
        )

    def move_eef_to_goal_pose(
        self,
        goal_pose,
        max_linear_vel=None,
        max_angular_vel=None,
        *,
        velocity=None,
        max_linear_acc=None,
        max_angular_acc=None,
    ):
        """Send WORLD [x,y,z,qw,qx,qy,qz]; disable force axes and passive mode."""
        args = self._cartesian_arguments(
            goal_pose, [0.0] * 6, velocity, max_linear_vel, max_angular_vel, max_linear_acc, max_angular_acc
        )
        with self._control():
            if (
                self._force_axes != [False] * 6
                or self._passive_force is not False
                or self.robot.mode() != self.mode.NRT_CARTESIAN_MOTION_FORCE
            ):
                self.configure_force_control([False] * 6)
            self._require_cartesian(mode=self.mode.NRT_CARTESIAN_MOTION_FORCE)
            self.robot.SendCartesianMotionForce(*args)

    def move_eef_to_goal_pose_wrench(
        self,
        goal_pose,
        wrench,
        max_linear_vel=None,
        max_angular_vel=None,
        contact_wrench=None,
        *,
        velocity=None,
        max_linear_acc=None,
        max_angular_acc=None,
    ):
        """Send hybrid control after configure_force_control.

        goal_pose: WORLD [x,y,z,qw,qx,qy,qz], with position in metres.
        Wrench is the desired force sensed at TCP, not force exerted by TCP:
        WORLD Fz=+5 N presses toward world -Z. Non-force axes must have zero wrench.
        """
        args = self._cartesian_arguments(
            goal_pose, wrench, velocity, max_linear_vel, max_angular_vel, max_linear_acc, max_angular_acc
        )
        self._require_cartesian(mode=self.mode.NRT_CARTESIAN_MOTION_FORCE)
        if any(w != 0 and not enabled for w, enabled in zip(args[1], self._force_axes)):
            raise ValueError("Nonzero wrench requires explicitly enabled force-control axes")
        with self._control():
            if contact_wrench is not None:
                self.set_max_contact_wrench(contact_wrench)
            self.robot.SendCartesianMotionForce(*args)

    def rotate_eef_tcp(self, axis, angle):
        if axis not in ("x", "y", "z") or not np.isfinite(angle):
            raise ValueError("axis must be x/y/z and angle finite radians")
        pose = self.get_current_eef_pose()
        rotation = Rotation.from_quat(pose[3:], scalar_first=True) * Rotation.from_euler(axis, angle)
        self.move_eef_to_goal_pose(pose[:3] + rotation.as_quat(scalar_first=True).tolist())

    def rotate_eef_joint(self, angle):
        q = self.get_current_joint_values()
        q[-1] += float(angle)
        self.move_arm_to_joint_values(q)

    def _wait(self, predicate, timeout):
        deadline = time.monotonic() + positive(timeout, "timeout")
        with self._control():
            while True:
                self._check_ready()
                if predicate(self.update_robot_states()):
                    return True
                if time.monotonic() >= deadline:
                    self.stop()
                    return False
                time.sleep(1.0 / self.frequency)

    def wait_for_joints(self, target_joint_values, error_threshold=0.01, speed_threshold=0.01, timeout=30.0):
        target = np.array(self._joints(target_joint_values))
        error_threshold = positive(error_threshold, "error_threshold")
        speed_threshold = positive(speed_threshold, "speed_threshold")
        return self._wait(
            lambda s: (
                np.max(np.abs(np.array(s.q) - target)) <= error_threshold
                and np.max(np.abs(s.dq)) <= speed_threshold
            ),
            timeout,
        )

    def wait_for_eef(
        self,
        target_tcp_pose,
        position_error_threshold=0.003,
        timeout=30.0,
        *,
        orientation_error_threshold=0.02,
        linear_velocity_threshold=0.01,
        angular_velocity_threshold=0.02,
    ):
        """Wait for WORLD [x,y,z,qw,qx,qy,qz], checking position, angle and speed."""
        target_tcp_pose = pose_vector(target_tcp_pose, "target_tcp_pose")
        if self._force_axes and any(self._force_axes):
            raise RuntimeError("Pose convergence is not defined for force-controlled axes")
        limits = vector(
            [
                position_error_threshold,
                orientation_error_threshold,
                linear_velocity_threshold,
                angular_velocity_threshold,
            ],
            4,
            "thresholds",
            lower=0,
            strict=True,
        )

        def reached(states):
            actual = pose_vector(states.tcp_pose, "measured tcp_pose")
            return (
                np.linalg.norm(np.array(actual[:3]) - target_tcp_pose[:3]) <= limits[0]
                and self.compute_orientation_error(actual[3:], target_tcp_pose[3:]) <= limits[1]
                and np.linalg.norm(states.tcp_vel[:3]) <= limits[2]
                and np.linalg.norm(states.tcp_vel[3:]) <= limits[3]
            )

        return self._wait(reached, timeout)

    @staticmethod
    def compute_orientation_error(current_orientation, target_orientation):
        """Angular distance between two [qw, qx, qy, qz] quaternions, in radians."""
        current_q = vector(current_orientation, 4, "current_orientation")
        target_q = vector(target_orientation, 4, "target_orientation")
        current = Rotation.from_quat(current_q, scalar_first=True)
        target = Rotation.from_quat(target_q, scalar_first=True)
        return float((target * current.inv()).magnitude())

    @staticmethod
    def _pose_matrix(pose):
        """Numerical transform for internal kinematics only; public poses remain vectors."""
        pose = pose_vector(pose)
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(pose[3:], scalar_first=True).as_matrix()
        matrix[:3, 3] = pose[:3]
        return matrix

    def _tool_matrix(self):
        return self._pose_matrix(self.config.flange_to_tcp)

    def joints_to_cartesian(self, joint_values):
        """Offline FK in robot base frame using the configured flange-to-TCP transform.

        Returns a plain [x,y,z,qw,qx,qy,qz] list.
        It equals RDK WORLD only if the controller world/base transform is identity.
        This kinematic URDF has blurred dynamics; do not use it for torque control.
        """
        q = self._joints(joint_values)
        matrix = self.robot_chain.forward_kinematics([0.0] + q + [0.0]) @ self._tool_matrix()
        quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat(scalar_first=True)
        return matrix[:3, 3].tolist() + quaternion.tolist()

    def cartesian_to_joints(self, pose, initial_joint_values=None):
        """Solve IK for base-frame [x, y, z, qw, qx, qy, qz]."""
        pose = pose_vector(pose)
        seed = self.get_current_joint_values() if initial_joint_values is None else initial_joint_values
        if len(seed) == 9:  # Legacy full-chain input, strip fixed links before checking.
            seed = seed[1:8]
        seed = self._joints(seed)
        flange = self._pose_matrix(pose) @ np.linalg.inv(self._tool_matrix())
        full = self.robot_chain.inverse_kinematics_frame(
            flange, initial_position=[0.0] + seed + [0.0], orientation_mode="all"
        )
        q = self._joints(full[1:8])
        result = self.joints_to_cartesian(q)
        if (
            np.linalg.norm(np.array(result[:3]) - pose[:3]) > 0.003
            or self.compute_orientation_error(result[3:], pose[3:]) > 0.02
        ):
            raise ValueError("IK did not converge to the requested pose")
        return q

    def calculate_IK_error(self, goal_pose):
        """IK position-plus-angle residual for a base-frame seven-value pose vector."""
        target = pose_vector(goal_pose, "goal_pose")
        actual = self.joints_to_cartesian(self.cartesian_to_joints(target))
        return float(
            np.linalg.norm(np.array(actual[:3]) - target[:3])
            + self.compute_orientation_error(actual[3:], target[3:])
        )

    # Python RDK provides NRT setpoints, not RT Stream* bindings.
    def servo_joint_position(
        self, positions, velocities=None, *, max_vel=None, max_acc=None, impedance=False
    ):
        """Send one NRT joint servo target (rad/rad/s), without waiting for arrival.

        Successive calls replan the onboard trajectory. Reuse the current mode;
        use run_servo() or your own 1–100 Hz loop. This is not a 1 kHz RT stream.
        """
        with self._control():
            self._boolean(impedance, "impedance")
            self.move_arm_to_joint_values(
                positions, target_vel=velocities, MAX_VEL=max_vel, MAX_ACC=max_acc, impedance=impedance
            )

    def servo_cartesian_pose(
        self,
        pose,
        velocity=None,
        *,
        max_linear_vel=None,
        max_angular_vel=None,
        max_linear_acc=None,
        max_angular_acc=None,
    ):
        """One pure-motion NRT target: WORLD [x,y,z,qw,qx,qy,qz], velocity [vx,vy,vz,wx,wy,wz]."""
        with self._control():
            self.move_eef_to_goal_pose(
                pose,
                max_linear_vel,
                max_angular_vel,
                velocity=velocity,
                max_linear_acc=max_linear_acc,
                max_angular_acc=max_angular_acc,
            )

    def servo_cartesian_motion_force(
        self,
        pose,
        wrench,
        velocity=None,
        *,
        max_linear_vel=None,
        max_angular_vel=None,
        max_linear_acc=None,
        max_angular_acc=None,
    ):
        """One NRT motion-force target; configure_force_control() once before the loop.

        Pose: WORLD [x,y,z,qw,qx,qy,qz]. Wrench: sensed N/Nm in the configured force frame.
        Velocity: WORLD [vx,vy,vz,wx,wy,wz]. No mode reset between successive targets.
        """
        with self._control():
            self.move_eef_to_goal_pose_wrench(
                pose,
                wrench,
                max_linear_vel,
                max_angular_vel,
                velocity=velocity,
                max_linear_acc=max_linear_acc,
                max_angular_acc=max_angular_acc,
            )

    def run_servo(self, callback, duration, *, frequency=None, max_lag=0.1):
        """Call callback(elapsed_seconds) at 1–100 Hz; stop on completion or any failure.

        The callback sends its target through a servo_* method. Missed periods are
        skipped rather than emitted in a burst. max_lag (seconds) is checked before
        and after callbacks. This is best-effort Python timing, not a hardware
        watchdog: a blocked callback cannot be interrupted by this same thread.
        """
        if not callable(callback):
            raise TypeError("callback must be callable")
        duration = positive(duration, "duration")
        frequency = positive(self.frequency if frequency is None else frequency, "frequency")
        if not 1 <= frequency <= 100:
            raise ValueError("Python NRT servo frequency must be in [1, 100] Hz")
        max_lag = positive(max_lag, "max_lag")
        period = 1.0 / frequency
        with self._control():
            started = next_tick = time.monotonic()
            deadline = started + duration
            while (now := time.monotonic()) < deadline:
                if now - next_tick > max_lag:
                    raise TimeoutError("Servo loop missed its dispatch deadline")
                self._check_ready()
                callback(now - started)
                after = time.monotonic()
                if after - next_tick > max_lag:
                    raise TimeoutError("Servo callback exceeded max_lag")
                next_tick += period
                if next_tick <= after:
                    next_tick = after + period
                delay = min(next_tick, deadline) - after
                if delay > 0:
                    time.sleep(delay)
            self.stop()

    def send_cartesian_waypoints(
        self,
        poses,
        joint_positions,
        *,
        wrenches=None,
        velocities=None,
        durations=None,
        max_linear_vel=None,
        max_angular_vel=None,
        max_linear_acc=None,
        max_angular_acc=None,
    ):
        """Send an onboard NRT_SUPER_PRIMITIVE batch; all poses are WORLD wxyz lists.

        Configure force axes first with super_primitive=True. Supply one 7-joint
        target per pose as required by RDK; joint_targets must match the Cartesian
        path/tool/world calibration. Durations are seconds (0 = SDK chooses timing).
        Returns after dispatch, without waiting for execution to finish.
        """
        poses, joint_positions = list(poses), list(joint_positions)
        count = len(poses)
        if not count or len(joint_positions) != count:
            raise ValueError("Provide matching nonempty poses and joint_positions")
        wrenches = [[0.0] * 6 for _ in poses] if wrenches is None else list(wrenches)
        velocities = [[0.0] * 6 for _ in poses] if velocities is None else list(velocities)
        durations = [0.0] * count if durations is None else vector(durations, count, "durations", lower=0)
        if len(wrenches) != count or len(velocities) != count:
            raise ValueError("wrenches and velocities must match the waypoint count")
        joints = [self._joints(q) for q in joint_positions]
        arguments = [
            self._cartesian_arguments(
                p, w, v, max_linear_vel, max_angular_vel, max_linear_acc, max_angular_acc
            )
            for p, w, v in zip(poses, wrenches, velocities)
        ]
        self._require_cartesian(mode=self.mode.NRT_SUPER_PRIMITIVE)
        if any(w != 0 and not enabled for args in arguments for w, enabled in zip(args[1], self._force_axes)):
            raise ValueError("Nonzero waypoint wrench requires enabled force-control axes")
        commands = [self.rdk.NrtCartesianCmd(*args, duration) for args, duration in zip(arguments, durations)]
        with self._control():
            self.robot.SendMultiCartesianMotionForce(commands, joints)

    @staticmethod
    def _boolean(value, name):
        if type(value) is not bool:
            raise ValueError(f"{name} must be a boolean")
        return value

    def _require_mode(self, *modes):
        self._check_ready()
        if self.robot.mode() not in modes:
            raise RuntimeError(f"Operation requires one of {modes}")

    def enable(self):
        """Request motor enable; initialize_robot() additionally clears faults and waits."""
        with self._control():
            self.robot.Enable()

    def clear_fault(self, timeout=30):
        """Return SDK ClearFault success; the controller remains in IDLE afterwards."""
        timeout = integer(timeout, "timeout", lower=1)
        with self._control():
            result = self.robot.ClearFault(timeout)
            self._force_axes = self._force_frame = self._contact_wrench = self._passive_force = None
            return result

    def switch_mode(self, mode):
        """Switch to an exported Python NRT mode. This may stop existing motion."""
        allowed = (
            self.mode.IDLE,
            self.mode.NRT_PLAN_EXECUTION,
            self.mode.NRT_PRIMITIVE_EXECUTION,
            self.mode.NRT_JOINT_POSITION,
            self.mode.NRT_JOINT_IMPEDANCE,
            self.mode.NRT_CARTESIAN_MOTION_FORCE,
            self.mode.NRT_SUPER_PRIMITIVE,
        )
        if mode not in allowed:
            raise ValueError("Only valid Python RDK NRT modes are supported")
        with self._control():
            if mode == self.mode.IDLE:
                self.stop()
            else:
                self._switch_mode(mode)

    def get_status(self):
        """Read SDK status flags without enabling/moving; values are not an atomic snapshot."""
        return dict(
            connected=self.robot.connected(),
            operational=self.robot.operational(),
            operational_status=self.robot.operational_status(),
            mode=self.robot.mode(),
            fault=self.robot.fault(),
            busy=self.robot.busy(),
            stopped=self.robot.stopped(),
            recovery=self.robot.recovery(),
            reduced=self.robot.reduced(),
            estop_released=self.robot.estop_released(),
            enabling_button_pressed=self.robot.enabling_button_pressed(),
            lite=self.robot.lite(),
            reached_timeliness_failure_limit=self.robot.reached_timeliness_failure_limit(),
        )

    def get_robot_info(self):
        return self.robot.info()

    def get_robot_actions(self):
        """RDK RobotActions snapshot; tcp_pose_d uses [x,y,z,qw,qx,qy,qz]."""
        return self.robot.actions()

    def get_event_log(self):
        return list(self.robot.event_log())

    def get_plan_list(self):
        return list(self.robot.plan_list())

    def get_plan_info(self):
        return self.robot.plan_info()

    def get_primitive_states(self):
        return dict(self.robot.primitive_states())

    def pause_plan(self, paused=True):
        """Pause/resume a user plan; SDK does not allow resuming internal plans."""
        paused = self._boolean(paused, "paused")
        with self._control():
            self._require_mode(self.mode.NRT_PLAN_EXECUTION)
            self.robot.PausePlan(paused)

    def stop_plan(self):
        """Stop plan execution; use stop() to also transition the controller to IDLE."""
        with self._control():
            self._require_mode(self.mode.NRT_PLAN_EXECUTION)
            self.robot.StopPlan()

    def set_breakpoint_mode(self, enabled):
        enabled = self._boolean(enabled, "enabled")
        with self._control():
            self._require_mode(self.mode.NRT_PLAN_EXECUTION)
            self.robot.SetBreakpointMode(enabled)

    def step_breakpoint(self):
        with self._control():
            self._require_mode(self.mode.NRT_PLAN_EXECUTION)
            self.robot.StepBreakpoint()

    def set_velocity_scale(self, percentage):
        """Plan/primitive velocity scale, integer percent 0–100; not a servo speed limit."""
        percentage = integer(percentage, "percentage", lower=0, upper=100)
        with self._control():
            self._require_mode(self.mode.NRT_PLAN_EXECUTION, self.mode.NRT_PRIMITIVE_EXECUTION)
            self.robot.SetVelocityScale(percentage)

    def set_joint_inertia_scale(self, scales):
        """Joint impedance inertia shaping [0.75,1.0]; 1 is nominal/no shaping."""
        scales = vector(scales, 7, "inertia scales", lower=0.75, upper=1.0)
        with self._control():
            self._require_mode(self.mode.NRT_JOINT_IMPEDANCE)
            self.robot.SetJointInertiaScale(scales)

    def get_digital_inputs(self):
        """18 input bits: control box 0–15 and wrist 16–17."""
        return list(self.robot.digital_inputs())

    def set_digital_outputs(self, outputs):
        """Set {port_index: bool}, ports 0–17. No automatic motion/mode change."""
        if not isinstance(outputs, Mapping) or not outputs:
            raise ValueError("outputs must be a nonempty mapping of port numbers to booleans")
        outputs = {
            integer(port, "port", lower=0, upper=17): self._boolean(value, "output")
            for port, value in outputs.items()
        }
        self.robot.SetDigitalOutputs(outputs)

    def get_global_variables(self):
        return dict(self.robot.global_variables())

    def set_global_variables(self, values):
        """Set existing Elements variables with SDK types; use integer 0/1 for booleans."""
        if (
            not isinstance(values, Mapping)
            or not values
            or any(not isinstance(k, str) or not k for k in values)
        ):
            raise ValueError("values must be a nonempty mapping of variable names to SDK values")
        self.robot.SetGlobalVariables(dict(values))

    def run_auto_recovery(self):
        """Blocking SDK recovery for out-of-range joints; a controller reboot is required afterwards."""
        if not self.robot.recovery():
            raise RuntimeError("Robot is not in the SDK recovery state")
        with self._control():
            self._force_axes = self._force_frame = self._contact_wrench = self._passive_force = None
            self.robot.RunAutoRecovery()

    def sdk_interface(self, name, *, password=None, gravity_vector=None):
        """Create a native SDK service without starting its operations.

        Tool/Gripper/Device/WorkCoord/FileIO/Maintenance/Model/Safety retain their
        official APIs and license requirements. Safety requires its configured
        password. Model defaults to WORLD gravity [0,0,-9.81]. Keep the returned
        object for reuse. Raw SDK calls do not synchronize wrapper mode/tool caches.
        """
        classes = {
            key: getattr(self.rdk, key)
            for key in ("Tool", "Gripper", "Device", "WorkCoord", "FileIO", "Maintenance", "Model", "Safety")
        }
        if name not in classes:
            raise ValueError(f"Unknown SDK interface; choose from {tuple(classes)}")
        if password is not None and name != "Safety":
            raise ValueError("password is only used by Safety")
        if gravity_vector is not None and name != "Model":
            raise ValueError("gravity_vector is only used by Model")
        if name == "Safety":
            if not isinstance(password, str) or not password:
                raise ValueError("Safety requires an explicit password")
            return classes[name](self.robot, password)
        if name == "Model":
            gravity = vector([0, 0, -9.81] if gravity_vector is None else gravity_vector, 3, "gravity")
            return classes[name](self.robot, gravity)
        return classes[name](self.robot)

    def list_tools(self):
        return list(self.rdk.Tool(self.robot).list())

    def switch_tool(self, name):
        """Activate an existing controller tool in IDLE and synchronize the offline TCP."""
        from dataclasses import replace

        tool = self.rdk.Tool(self.robot)
        pose = pose_vector(tool.params(name).tcp_location, "tool tcp_location")
        with self._control():
            self.stop()
            self._check_ready()
            tool.Switch(name)
            self.config = replace(self.config, flange_to_tcp=tuple(pose))

    def remove_tool(self, name):
        """Remove an inactive custom tool. Switch tools explicitly before deleting the active one."""
        tool = self.rdk.Tool(self.robot)
        if name == "Flange" or name == tool.name():
            raise ValueError("Cannot remove Flange or the active tool")
        with self._control():
            self.stop()
            self._check_ready()
            tool.Remove(name)

    def list_work_coordinates(self):
        return list(self.rdk.WorkCoord(self.robot).list())

    def get_work_coordinate(self, name):
        """Return WORLD [x,y,z,qw,qx,qy,qz] for an existing work coordinate."""
        return list(self.rdk.WorkCoord(self.robot).pose(name))

    def configure_work_coordinate(self, name, pose):
        """Add/update a WORLD [x,y,z,qw,qx,qy,qz] work coordinate in IDLE."""
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Work coordinate name must be nonempty")
        pose = pose_vector(pose)
        with self._control():
            self.stop()
            self._check_ready()
            work = self.rdk.WorkCoord(self.robot)
            if work.exist(name):
                work.Update(name, pose)
            else:
                work.Add(name, pose)

    def remove_work_coordinate(self, name):
        with self._control():
            self.stop()
            self._check_ready()
            self.rdk.WorkCoord(self.robot).Remove(name)

    def find_gripper(self):
        """List a likely USB serial port without writing to unrelated serial devices."""
        from serial.tools import list_ports

        ports = [p.device for p in list_ports.comports() if "USB" in p.device or "usb" in p.device]
        if len(ports) != 1:
            raise RuntimeError("Specify connect_gripper(port=...) when USB port is absent or ambiguous")
        return ports[0]

    def connect_gripper(self, port=None):
        import pyRobotiqGripper

        self.gripper = pyRobotiqGripper.RobotiqGripper(portname=port or self.find_gripper())
        if not self.gripper.isActivated():
            self.gripper.activate()
        return self.gripper

    def gripper_goto(self, value, speed=255, force=255):
        values = [value, speed, force]
        if any(type(v) is not int or not 0 <= v <= 255 for v in values):
            raise ValueError("Gripper position, speed and force must be integers in [0,255]")
        if self.gripper is None:
            raise RuntimeError("Call connect_gripper first")
        self.gripper.goTo(position=value, speed=speed, force=force)

    def open_gripper(self, speed=255, force=255):
        self.gripper_goto(0, speed, force)

    def close_gripper(self, speed=255, force=255):
        self.gripper_goto(255, speed, force)
