"""Rizon 4 interface for Flexiv RDK 1.9.4.1 (non-real-time Python control)."""

from __future__ import annotations

import logging
import time
import warnings
from contextlib import contextmanager
from importlib.metadata import version
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .config import RobotConfig, pose_vector, positive, vector


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

    def execute_primitive(self, name, parameters=None, *, timeout=30.0):
        """Execute the new name + parameter-dictionary API and wait for termination."""
        timeout = positive(timeout, "timeout")
        with self._control():
            self._switch_mode(self.mode.NRT_PRIMITIVE_EXECUTION)
            self.robot.ExecutePrimitive(name, {} if parameters is None else parameters)
            deadline = time.monotonic() + timeout
            while True:
                self._check_ready()
                if self.robot.primitive_states().get("terminated", False):
                    return
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Primitive {name} timed out")
                time.sleep(0.02)

    def execute_plan(self, name, *, wait=True, timeout=60.0):
        timeout = positive(timeout, "timeout")
        with self._control():
            self._switch_mode(self.mode.NRT_PLAN_EXECUTION)
            self.robot.ExecutePlan(name)
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
    ):
        """Configure once before sending hybrid/force commands.

        Axes: [X,Y,Z,Rx,Ry,Rz]. True selects explicit force/moment control.
        frame_transform: [x,y,z,qw,qx,qy,qz] relative to the selected root frame.
        Active control is closed-loop; passive is open-loop wrench feedforward.
        Mode changes discard this wrapper's configuration: configure again afterwards.
        """
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
            self._switch_mode(self.mode.NRT_CARTESIAN_MOTION_FORCE)
            self.robot.SetForceControlAxis([False] * 6, vmax)
            self.robot.SetMaxContactWrench(limits)
            self.robot.SetForceControlFrame(getattr(self.rdk.CoordType, frame), transform)
            self.robot.SetForceControlAxis(axes, vmax)
            self._force_axes, self._force_frame, self._contact_wrench = axes, frame, limits
            self._passive_force = passive

    def _require_cartesian(self):
        self._check_ready()
        if self.robot.mode() != self.mode.NRT_CARTESIAN_MOTION_FORCE or self._force_axes is None:
            raise RuntimeError("Call configure_force_control() before Cartesian force/impedance commands")

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
        limits = vector(torques, 7, "contact torques", lower=0, allow_inf=True)
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
            self._require_cartesian()
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
        self._require_cartesian()
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
