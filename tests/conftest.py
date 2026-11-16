"""Strict command recorder, not a physics simulator or controller emulator."""

from types import SimpleNamespace

import flexivrdk
import pytest

from Robotics_API import Bestman_Real_Flexiv


class RecordingRobot:
    def __init__(self):
        self.events = []
        self.current_mode = flexivrdk.Mode.IDLE
        self.has_fault = False
        self.is_ready = True
        self.fail_send = False
        self.terminated = True
        self.plan_busy = False
        self.state = SimpleNamespace(
            q=[0.0] * 7,
            dq=[0.0] * 7,
            tau=[1.0] * 7,
            tau_ext=[2.0] * 7,
            tcp_pose=[0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
            tcp_vel=[0.0] * 6,
            ext_wrench_in_world=[0.0, 0.0, 3.0, 0.0, 0.0, 0.0],
            ext_wrench_in_tcp=[0.0, 0.0, -3.0, 0.0, 0.0, 0.0],
            ext_wrench_in_world_raw=[0.0, 0.0, 4.0, 0.0, 0.0, 0.0],
            ext_wrench_in_tcp_raw=[0.0, 0.0, -4.0, 0.0, 0.0, 0.0],
        )
        self.information = SimpleNamespace(
            model_name="Rizon4",
            DoF=7,
            DoF_e=0,
            serial_num="Rizon4-123456",
            q_min=[-2.0] * 7,
            q_max=[2.0] * 7,
            dq_max=[2.0] * 7,
            K_x_nom=[1000.0] * 3 + [100.0] * 3,
            K_q_nom=[100.0] * 7,
        )

    def info(self):
        return self.information

    def states(self):
        return self.state

    def mode(self):
        return self.current_mode

    def fault(self):
        return self.has_fault

    def operational(self):
        return self.is_ready

    def busy(self):
        return self.plan_busy

    def primitive_states(self):
        return {"terminated": self.terminated}

    def ClearFault(self):
        self.events.append(("ClearFault",))
        self.has_fault = False
        return True

    def Enable(self):
        self.events.append(("Enable",))

    def Stop(self):
        self.events.append(("Stop",))
        self.current_mode = flexivrdk.Mode.IDLE

    def SwitchMode(self, mode):
        self.events.append(("SwitchMode", mode))
        self.current_mode = mode

    def SetPassiveForceControl(self, is_enabled):
        assert self.mode() == flexivrdk.Mode.IDLE
        self.events.append(("SetPassiveForceControl", is_enabled))

    def _cartesian(self):
        assert self.mode() == flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE

    def SetForceControlFrame(self, root_coord, T_in_root):
        self._cartesian()
        self.events.append(("SetForceControlFrame", root_coord, T_in_root))

    def SetForceControlAxis(self, enabled_axes, max_linear_vel):
        self._cartesian()
        self.events.append(("SetForceControlAxis", enabled_axes, max_linear_vel))

    def SetMaxContactWrench(self, max_wrench):
        self._cartesian()
        self.events.append(("SetMaxContactWrench", max_wrench))

    def SendCartesianMotionForce(
        self, pose, wrench, velocity, max_linear_vel, max_angular_vel, max_linear_acc, max_angular_acc
    ):
        self._cartesian()
        if self.fail_send:
            raise RuntimeError("injected transport failure")
        self.events.append(
            (
                "SendCartesianMotionForce",
                pose,
                wrench,
                velocity,
                max_linear_vel,
                max_angular_vel,
                max_linear_acc,
                max_angular_acc,
            )
        )

    def SendJointPosition(self, positions, velocities, max_vel, max_acc):
        assert self.mode() in (flexivrdk.Mode.NRT_JOINT_POSITION, flexivrdk.Mode.NRT_JOINT_IMPEDANCE)
        self.events.append(("SendJointPosition", positions, velocities, max_vel, max_acc))

    def SetCartesianImpedance(self, K_x, Z_x):
        self._cartesian()
        self.events.append(("SetCartesianImpedance", K_x, Z_x))

    def SetJointImpedance(self, K_q, Z_q):
        assert self.mode() == flexivrdk.Mode.NRT_JOINT_IMPEDANCE
        self.events.append(("SetJointImpedance", K_q, Z_q))

    def SetMaxContactTorque(self, max_torques):
        assert self.mode() == flexivrdk.Mode.NRT_JOINT_IMPEDANCE
        self.events.append(("SetMaxContactTorque", max_torques))

    def SetNullSpacePosture(self, ref_positions):
        self._cartesian()
        self.events.append(("SetNullSpacePosture", ref_positions))

    def SetNullSpaceObjectives(self, linear_manipulability, angular_manipulability, ref_positions_tracking):
        self._cartesian()
        self.events.append(
            ("SetNullSpaceObjectives", linear_manipulability, angular_manipulability, ref_positions_tracking)
        )

    def ExecutePrimitive(self, primitive_name, input_params):
        assert self.mode() == flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION
        self.events.append(("ExecutePrimitive", primitive_name, input_params))

    def ExecutePlan(self, name):
        assert self.mode() == flexivrdk.Mode.NRT_PLAN_EXECUTION
        self.events.append(("ExecutePlan", name))


@pytest.fixture
def backend():
    return RecordingRobot()


@pytest.fixture
def arm(backend):
    return Bestman_Real_Flexiv("Rizon4-123456", robot=backend)
