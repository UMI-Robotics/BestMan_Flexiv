import importlib

import flexivrdk
import pytest


class Clock:
    def __init__(self):
        self.now = 0.0
        self.delays = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.delays.append(seconds)
        self.now += seconds


@pytest.fixture
def clock(monkeypatch):
    module = importlib.import_module("Robotics_API.Bestman_Real_Flexiv")
    clock = Clock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(module.time, "sleep", clock.sleep)
    return clock


def test_joint_servo_reuses_mode_and_stops_at_duration(arm, backend, clock):
    ticks = []

    def callback(t):
        ticks.append(t)
        arm.servo_joint_position([t] * 7, [0.1] * 7, max_vel=[0.2] * 7)

    arm.run_servo(callback, 0.11, frequency=20)
    assert ticks == pytest.approx([0.0, 0.05, 0.10])
    assert [e[0] for e in backend.events].count("SwitchMode") == 1
    assert [e[0] for e in backend.events].count("SendJointPosition") == 3
    assert backend.events[-1] == ("Stop",)
    assert clock.now == pytest.approx(0.11)


def test_servo_cartesian_preserves_wxyz_and_velocity_without_force_reconfigure(arm, backend, clock):
    pose = [0.4, 0.1, 0.5, 0.8, 0.2, 0.4, 0.4]
    velocity = [0.01, 0, 0, 0, 0, 0]
    arm.configure_force_control([False, False, True, False, False, False])
    original = len(backend.events)
    arm.run_servo(lambda t: arm.servo_cartesian_motion_force(pose, [0, 0, 3, 0, 0, 0], velocity), 0.11)
    calls = backend.events[original:]
    assert [e[0] for e in calls] == ["SendCartesianMotionForce"] * 3 + ["Stop"]
    assert calls[0][1:4] == (pose, [0.0, 0.0, 3.0, 0.0, 0.0, 0.0], velocity)


def test_pure_cartesian_servo_removes_previous_force_axes(arm, backend):
    arm.configure_force_control([False, False, True, False, False, False])
    arm.servo_cartesian_pose(arm.get_current_eef_pose())
    assert arm._force_axes == [False] * 6
    assert backend.events[-1][2] == [0.0] * 6


@pytest.mark.parametrize("error", [RuntimeError("callback failed"), KeyboardInterrupt()])
def test_servo_callback_failure_always_stops(arm, backend, clock, error):
    def callback(t):
        raise error

    with pytest.raises(type(error)):
        arm.run_servo(callback, 1.0)
    assert backend.events[-1] == ("Stop",)


def test_servo_does_not_send_after_late_wakeup(arm, backend, clock, monkeypatch):
    def delayed_sleep(seconds):
        clock.now += seconds + 0.2

    monkeypatch.setattr(
        importlib.import_module("Robotics_API.Bestman_Real_Flexiv").time, "sleep", delayed_sleep
    )
    with pytest.raises(TimeoutError, match="dispatch"):
        arm.run_servo(lambda t: arm.servo_joint_position([0.0] * 7), 1.0)
    assert [e[0] for e in backend.events].count("SendJointPosition") == 1
    assert backend.events[-1] == ("Stop",)


def test_servo_detects_slow_callback_on_return(arm, backend, clock):
    def callback(t):
        clock.now += 0.2

    with pytest.raises(TimeoutError, match="callback"):
        arm.run_servo(callback, 1.0)
    assert backend.events[-1] == ("Stop",)


def test_servo_skips_missed_periods_without_burst(arm, backend, clock):
    ticks = []

    def callback(t):
        ticks.append(t)
        clock.now += 0.07

    arm.run_servo(callback, 0.2, frequency=20)
    assert ticks == pytest.approx([0.0, 0.12])
    assert min(clock.delays) > 0


@pytest.mark.parametrize("frequency", [0, 101, float("nan")])
def test_servo_frequency_validation_does_not_move(arm, backend, frequency):
    with pytest.raises(ValueError):
        arm.run_servo(lambda t: None, 1.0, frequency=frequency)
    assert backend.events == []


def test_joint_impedance_servo_keeps_impedance_mode(arm, backend):
    arm.set_joint_impedance([10.0] * 7)
    arm.set_joint_inertia_scale([0.8] * 7)
    arm.servo_joint_position([0.1] * 7, impedance=True)
    assert backend.mode() == flexivrdk.Mode.NRT_JOINT_IMPEDANCE
    assert [e[0] for e in backend.events].count("SwitchMode") == 1


def test_inertia_and_contact_torque_ranges_match_sdk(arm, backend):
    for scales in ([0.7] * 7, [1.1] * 7, [float("nan")] * 7):
        with pytest.raises(ValueError):
            arm.set_joint_inertia_scale(scales)
    for limits in ([81.0] * 7, [float("inf")] * 7, [-1.0] * 7):
        with pytest.raises(ValueError):
            arm.set_max_contact_torque(limits)
    assert backend.events == []


def test_onboard_waypoints_use_real_sdk_commands_and_wxyz(arm, backend):
    poses = [[0.4, 0.1, 0.5, 0.8, 0.2, 0.4, 0.4], [0.41, 0.1, 0.5, 0.8, 0.2, 0.4, 0.4]]
    arm.configure_force_control([False] * 6, super_primitive=True)
    arm.set_cartesian_impedance([10.0] * 6)
    arm.send_cartesian_waypoints(poses, [[0.0] * 7, [0.1] * 7], durations=[1.0, 2.0])
    name, commands, joints = backend.events[-1]
    assert name == "SendMultiCartesianMotionForce"
    assert isinstance(commands[0], flexivrdk.NrtCartesianCmd)
    assert list(commands[0].pose_d) == poses[0]
    assert commands[1].duration == 2.0
    assert commands[0].max_linear_vel == arm.config.max_linear_vel
    assert joints == [[0.0] * 7, [0.1] * 7]
    with pytest.raises(RuntimeError):
        arm.servo_cartesian_motion_force(poses[0], [0.0] * 6)


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(poses=[], joint_positions=[]),
        dict(poses=[[0, 0, 0, 1, 0, 0, 0]], joint_positions=[]),
        dict(poses=[[0] * 7], joint_positions=[[0] * 7]),
        dict(poses=[[0, 0, 0, 1, 0, 0, 0]], joint_positions=[[0] * 7], durations=[-1]),
        dict(poses=[[0, 0, 0, 1, 0, 0, 0]], joint_positions=[[0] * 7], wrenches=[]),
    ],
)
def test_invalid_waypoint_batches_have_no_side_effects(arm, backend, kwargs):
    with pytest.raises(ValueError):
        arm.send_cartesian_waypoints(**kwargs)
    assert backend.events == []


def test_multiwaypoint_wrench_requires_explicit_axes(arm, backend):
    arm.configure_force_control([False] * 6, super_primitive=True)
    count = len(backend.events)
    with pytest.raises(ValueError):
        arm.send_cartesian_waypoints([arm.get_current_eef_pose()], [[0] * 7], wrenches=[[0, 0, 3, 0, 0, 0]])
    assert len(backend.events) == count


def test_async_primitive_allows_state_based_completion(arm, backend):
    backend.terminated = False
    arm.execute_primitive("MoveL", {}, wait=False)
    backend.primitive_states = lambda: {"terminated": False, "reachedTarget": 1}
    arm.wait_for_primitive("reachedTarget", 1)
    assert backend.events[-1] == ("ExecutePrimitive", "MoveL", {}, True)
    assert arm.get_primitive_states()["reachedTarget"] == 1


@pytest.mark.parametrize("method", ["execute_plan", "execute_primitive"])
def test_completion_wait_requires_started_command(arm, backend, method):
    with pytest.raises(ValueError):
        getattr(arm, method)("test", block_until_started=False)
    assert backend.events == []


def test_plan_controls_and_index_overload(arm, backend):
    arm.execute_plan(1, wait=False, continue_execution=True)
    assert backend.events[-1] == ("ExecutePlan", 1, True, True)
    arm.pause_plan()
    arm.pause_plan(False)
    arm.set_breakpoint_mode(True)
    arm.step_breakpoint()
    arm.set_velocity_scale(25)
    arm.stop_plan()
    assert [e[0] for e in backend.events][-6:] == [
        "PausePlan",
        "PausePlan",
        "SetBreakpointMode",
        "StepBreakpoint",
        "SetVelocityScale",
        "StopPlan",
    ]
    assert arm.get_plan_list() == ["PLAN-Home", "user-plan"]
    assert not arm.get_plan_info().waiting_for_step


@pytest.mark.parametrize("percentage", [True, -1, 101, 1.2])
def test_velocity_scale_rejects_noninteger_or_out_of_range(arm, backend, percentage):
    with pytest.raises(ValueError):
        arm.set_velocity_scale(percentage)
    assert backend.events == []


def test_plan_control_does_not_switch_modes(arm, backend):
    arm.servo_joint_position([0] * 7)
    with pytest.raises(RuntimeError):
        arm.pause_plan()
    assert [e[0] for e in backend.events].count("SwitchMode") == 1
    assert backend.events[-1] == ("Stop",)


def test_status_actions_io_globals_and_fault_clearing(arm, backend):
    assert arm.get_status()["connected"]
    assert arm.get_robot_actions().tcp_pose_d == backend.state.tcp_pose
    assert arm.get_robot_info().DoF == 7
    assert arm.get_event_log() == []
    assert arm.get_digital_inputs() == [False] * 18
    assert backend.events == []
    arm.set_digital_outputs({0: True, 17: False})
    assert backend.digital_values == {0: True, 17: False}
    arm.set_global_variables({"test_value": 2})
    assert arm.get_global_variables()["test_value"] == 2
    arm.configure_force_control([False] * 6)
    arm.clear_fault(5)
    assert backend.events[-1] == ("ClearFault", 5)
    assert arm._force_axes is None


@pytest.mark.parametrize("outputs", [{18: True}, {-1: True}, {True: True}, {0: 1}, {}])
def test_digital_output_validation(arm, backend, outputs):
    with pytest.raises(ValueError):
        arm.set_digital_outputs(outputs)
    assert backend.events == []


def test_recovery_is_explicit_and_requires_recovery_state(arm, backend):
    with pytest.raises(RuntimeError):
        arm.run_auto_recovery()
    assert backend.events == []
    backend.needs_recovery = True
    backend.is_ready = False  # Recovery is specifically not normal operational mode.
    arm.run_auto_recovery()
    assert backend.events[-1] == ("RunAutoRecovery",)


def test_sdk_service_constructor_parameters_and_password_not_stored(arm, backend, monkeypatch):
    calls = []
    for name in ("Model", "Safety", "Gripper", "Device", "WorkCoord", "FileIO", "Maintenance", "Tool"):

        def factory(*args, name=name):
            calls.append((name, args))
            return object()

        monkeypatch.setattr(arm.rdk, name, factory)
    for name in ("Gripper", "Device", "WorkCoord", "FileIO", "Maintenance", "Tool"):
        arm.sdk_interface(name)
        assert calls[-1] == (name, (backend,))
    arm.sdk_interface("Model", gravity_vector=[0, 0, -9.8])
    assert calls[-1] == ("Model", (backend, [0.0, 0.0, -9.8]))
    with pytest.raises(ValueError):
        arm.sdk_interface("Safety")
    arm.sdk_interface("Safety", password="test-only")
    assert calls[-1] == ("Safety", (backend, "test-only"))
    assert "password" not in arm.__dict__
    with pytest.raises(ValueError):
        arm.sdk_interface("NotAService")
    with pytest.raises(ValueError):
        arm.sdk_interface("Tool", gravity_vector=[0] * 3)


def test_work_coordinates_keep_wxyz_and_use_idle_mutations(arm, backend, monkeypatch):
    entries = {}

    class Work:
        def __init__(self, robot):
            assert robot is backend

        def exist(self, name):
            return name in entries

        def list(self):
            return list(entries)

        def pose(self, name):
            return entries[name].copy()

        def Add(self, name, pose):
            assert backend.mode() == flexivrdk.Mode.IDLE
            entries[name] = pose.copy()

        def Update(self, name, pose):
            self.Add(name, pose)

        def Remove(self, name):
            assert backend.mode() == flexivrdk.Mode.IDLE
            del entries[name]

    monkeypatch.setattr(arm.rdk, "WorkCoord", Work)
    pose = [0.2, 0.0, 0.1, 0.8, 0.2, 0.4, 0.4]
    arm.configure_work_coordinate("bench", pose)
    assert arm.get_work_coordinate("bench") == pose
    pose[0] = 0.3
    arm.configure_work_coordinate("bench", pose)
    assert arm.list_work_coordinates() == ["bench"]
    assert arm.get_work_coordinate("bench") == pose
    arm.remove_work_coordinate("bench")
    assert arm.list_work_coordinates() == []


def test_tool_switch_synchronizes_offline_tcp_and_active_tool_cannot_be_removed(arm, backend, monkeypatch):
    from types import SimpleNamespace

    pose = [0.1, 0.0, 0.2, 0.8, 0.2, 0.4, 0.4]
    active = ["Flange"]
    tools = {"Flange": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "calibrated": pose}

    class Tool:
        def __init__(self, robot):
            assert robot is backend

        def list(self):
            return list(tools)

        def name(self):
            return active[0]

        def params(self, name):
            return SimpleNamespace(tcp_location=tools[name])

        def Switch(self, name):
            assert backend.mode() == flexivrdk.Mode.IDLE
            active[0] = name

        def Remove(self, name):
            assert backend.mode() == flexivrdk.Mode.IDLE
            del tools[name]

    monkeypatch.setattr(arm.rdk, "Tool", Tool)
    assert arm.list_tools() == ["Flange", "calibrated"]
    arm.switch_tool("calibrated")
    assert arm.config.flange_to_tcp == tuple(pose)
    with pytest.raises(ValueError):
        arm.remove_tool("calibrated")
    with pytest.raises(ValueError):
        arm.remove_tool("Flange")
    arm.switch_tool("Flange")
    arm.remove_tool("calibrated")
    assert arm.list_tools() == ["Flange"]


def test_switch_mode_rejects_unknown_and_enable_does_not_move(arm, backend):
    with pytest.raises(ValueError):
        arm.switch_mode(flexivrdk.Mode.UNKNOWN)
    arm.enable()
    assert backend.events == [("Enable",)]
    arm.switch_mode(flexivrdk.Mode.NRT_JOINT_POSITION)
    arm.switch_mode(flexivrdk.Mode.IDLE)
    assert backend.events[-1] == ("Stop",)


def test_servo_examples_default_hold_and_keep_wxyz(arm, backend, monkeypatch, clock):
    import sys

    from Examples import _common, sdk_status, servo_cartesian_motion_force, servo_joint_position

    monkeypatch.setattr(_common, "Bestman_Real_Flexiv", lambda *a, **kw: arm)
    monkeypatch.setattr(sys, "argv", ["servo", "Rizon4-123456", "--duration", "0.11"])
    servo_joint_position.main()
    sent = [e for e in backend.events if e[0] == "SendJointPosition"]
    assert len(sent) == 3 and all(e[1] == [0.0] * 7 for e in sent)
    backend.events.clear()
    backend.state.tcp_pose = [0.4, 0.1, 0.5, 0.8, 0.2, 0.4, 0.4]
    servo_cartesian_motion_force.main()
    sent = [e for e in backend.events if e[0] == "SendCartesianMotionForce"]
    assert len(sent) == 3 and all(e[1] == backend.state.tcp_pose for e in sent)
    assert backend.events[-1] == ("Stop",)
    backend.events.clear()
    monkeypatch.setattr(sys, "argv", ["status", "Rizon4-123456"])
    sdk_status.main()
    assert backend.events == []


def test_servo_example_checks_measured_force_before_sending(arm, backend, monkeypatch, clock):
    import sys

    from Examples import _common, servo_cartesian_motion_force

    monkeypatch.setattr(_common, "Bestman_Real_Flexiv", lambda *a, **kw: arm)
    monkeypatch.setattr(sys, "argv", ["servo", "Rizon4-123456", "--duration", "0.11", "--force-z", "3"])
    backend.state.ext_wrench_in_world = [0.0, 0.0, 20.0, 0.0, 0.0, 0.0]
    with pytest.raises(RuntimeError, match="exceeded"):
        servo_cartesian_motion_force.main()
    assert not any(e[0] == "SendCartesianMotionForce" for e in backend.events)
    assert backend.events[-1] == ("Stop",)


@pytest.mark.parametrize(
    "method,args",
    [
        ("servo_joint_position", [[0] * 6]),
        ("servo_cartesian_pose", [[0] * 7]),
        ("servo_cartesian_motion_force", [[0] * 7, [0] * 6]),
    ],
)
def test_invalid_servo_target_stops_previous_motion_without_loop_helper(arm, backend, method, args):
    arm.servo_joint_position([0.1] * 7)
    with pytest.raises(ValueError):
        getattr(arm, method)(*args)
    assert backend.events[-1] == ("Stop",)


def test_servo_connection_loss_stops_before_next_command(arm, backend, clock):
    backend.connected = lambda: False
    with pytest.raises(RuntimeError, match="connection"):
        arm.run_servo(lambda t: arm.servo_joint_position([0] * 7), 0.1)
    assert backend.events == [("Stop",)]
