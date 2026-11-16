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
        self.needs_recovery = False
        self.digital_values = {}
        self.variables = {"test_value": 1}
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
            tau_max=[80.0] * 7,
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

    def ClearFault(self, timeout_sec=30):
        self.events.append(("ClearFault", timeout_sec))
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
        assert self.mode() in (flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE, flexivrdk.Mode.NRT_SUPER_PRIMITIVE)

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

    def ExecutePrimitive(self, primitive_name, input_params, block_until_started=True):
        assert self.mode() == flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION
        self.events.append(("ExecutePrimitive", primitive_name, input_params, block_until_started))

    def ExecutePlan(self, name, continue_exec=False, block_until_started=True):
        assert self.mode() == flexivrdk.Mode.NRT_PLAN_EXECUTION
        self.events.append(("ExecutePlan", name, continue_exec, block_until_started))

    def SendMultiCartesianMotionForce(self, cart_cmds, joint_pos):
        assert self.mode() == flexivrdk.Mode.NRT_SUPER_PRIMITIVE
        self.events.append(("SendMultiCartesianMotionForce", cart_cmds, joint_pos))

    def SetJointInertiaScale(self, inertia_scales):
        assert self.mode() == flexivrdk.Mode.NRT_JOINT_IMPEDANCE
        self.events.append(("SetJointInertiaScale", inertia_scales))

    def PausePlan(self, toggle):
        assert self.mode() == flexivrdk.Mode.NRT_PLAN_EXECUTION
        self.events.append(("PausePlan", toggle))

    def StopPlan(self):
        assert self.mode() == flexivrdk.Mode.NRT_PLAN_EXECUTION
        self.events.append(("StopPlan",))
        self.plan_busy = False

    def SetBreakpointMode(self, is_enabled):
        assert self.mode() == flexivrdk.Mode.NRT_PLAN_EXECUTION
        self.events.append(("SetBreakpointMode", is_enabled))

    def StepBreakpoint(self):
        assert self.mode() == flexivrdk.Mode.NRT_PLAN_EXECUTION
        self.events.append(("StepBreakpoint",))

    def SetVelocityScale(self, velocity_scale):
        assert self.mode() in (flexivrdk.Mode.NRT_PLAN_EXECUTION, flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
        self.events.append(("SetVelocityScale", velocity_scale))

    def SetDigitalOutputs(self, digital_outputs):
        self.digital_values.update(digital_outputs)
        self.events.append(("SetDigitalOutputs", digital_outputs))

    def digital_inputs(self):
        return [False] * 18

    def global_variables(self):
        return self.variables.copy()

    def SetGlobalVariables(self, global_vars):
        self.variables.update(global_vars)
        self.events.append(("SetGlobalVariables", global_vars))

    def connected(self):
        return True

    def stopped(self):
        return self.mode() == flexivrdk.Mode.IDLE

    def recovery(self):
        return self.needs_recovery

    def reduced(self):
        return False

    def estop_released(self):
        return True

    def enabling_button_pressed(self):
        return False

    def lite(self):
        return False

    def reached_timeliness_failure_limit(self):
        return False

    def operational_status(self):
        return flexivrdk.OperationalStatus.READY

    def actions(self):
        return SimpleNamespace(tcp_pose_d=self.state.tcp_pose.copy())

    def event_log(self):
        return []

    def plan_list(self):
        return ["PLAN-Home", "user-plan"]

    def plan_info(self):
        return SimpleNamespace(waiting_for_step=False)

    def RunAutoRecovery(self):
        assert self.needs_recovery
        self.events.append(("RunAutoRecovery",))


@pytest.fixture
def backend():
    return RecordingRobot()


@pytest.fixture
def arm(backend):
    return Bestman_Real_Flexiv("Rizon4-123456", robot=backend)
