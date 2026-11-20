import importlib
from dataclasses import replace
from unittest.mock import Mock

import flexivrdk
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from Robotics_API import Bestman_Real_Flexiv, RobotConfig

AXES_Z = [False, False, True, False, False, False]


@pytest.mark.parametrize("container", [list, tuple, np.array])
def test_pose_vectors_pass_through_sdk_without_reordering(arm, backend, container):
    values = [0.3, 0.1, 0.5, 0.8, 0.2, 0.4, 0.4]
    target = container(values)
    arm.move_eef_to_goal_pose(target)
    assert backend.events[-1][1] == values
    np.testing.assert_array_equal(target, values)
    backend.state.tcp_pose = values.copy()
    measured = arm.get_current_eef_pose()
    assert isinstance(measured, list) and measured == values
    measured[0] += 1
    assert backend.state.tcp_pose == values
    assert arm.compute_orientation_error(values[3:], [-x for x in values[3:]]) < 1e-12


@pytest.mark.parametrize(
    "target", [[0] * 6, [0] * 8, [0] * 7, [0, 0, 0, np.nan, 0, 0, 1], [0, 0, 0, 2, 0, 0, 0]]
)
def test_invalid_pose_rejected_before_commands(arm, backend, target):
    with pytest.raises(ValueError):
        arm.move_eef_to_goal_pose(target)
    assert backend.events == []


def test_read_current_pose_uses_wxyz_boundary(arm, backend):
    assert arm.get_current_eef_pose() == backend.state.tcp_pose
    assert arm.get_tcp_wrench() == backend.state.ext_wrench_in_world
    assert arm.get_tcp_wrench("TCP", raw=True) == backend.state.ext_wrench_in_tcp_raw
    assert arm.get_joint_torques(external=True) == backend.state.tau_ext


def test_force_setup_order_frame_and_velocity(arm, backend):
    transform = [0.01, 0.0, 0.0, 0.8, 0.2, 0.4, 0.4]
    arm.configure_force_control(AXES_Z, frame="TCP", frame_transform=transform, passive=True)
    names = [e[0] for e in backend.events]
    assert names.index("Stop") < names.index("SetPassiveForceControl") < names.index("SwitchMode")
    assert backend.events[-2] == ("SetForceControlFrame", flexivrdk.CoordType.TCP, transform)
    assert backend.events[-1] == ("SetForceControlAxis", AXES_Z, [0.02] * 3)
    arm.move_eef_to_goal_pose_wrench(arm.get_current_eef_pose(), [0, 0, -3, 0, 0, 0])
    event = backend.events[-1]
    assert event[1] == backend.state.tcp_pose
    assert event[2] == [0, 0, -3, 0, 0, 0]
    assert event[3] == [0.0] * 6  # New SDK velocity vector, not scalar speed.
    assert event[4:] == (0.05, 0.3, 0.2, 0.5)


def test_pure_motion_disables_force_and_does_not_reconfigure_each_sample(arm, backend):
    arm.configure_force_control(AXES_Z, passive=True)
    arm.move_eef_to_goal_pose(arm.get_current_eef_pose())
    assert ("SetPassiveForceControl", False) in backend.events
    assert backend.events[-2][1] == [False] * 6
    count = len(backend.events)
    arm.move_eef_to_goal_pose(arm.get_current_eef_pose())
    assert len(backend.events) == count + 1


def test_unconfigured_force_and_disabled_axis_rejected(arm):
    with pytest.raises(RuntimeError):
        arm.move_eef_to_goal_pose_wrench(arm.get_current_eef_pose(), [0, 0, 3, 0, 0, 0])
    arm.configure_force_control(AXES_Z)
    with pytest.raises(ValueError):
        arm.move_eef_to_goal_pose_wrench(arm.get_current_eef_pose(), [3, 0, 0, 0, 0, 0])


def test_mode_change_invalidates_force_configuration(arm):
    arm.configure_force_control(AXES_Z)
    arm.move_arm_to_joint_values([0.0] * 7)
    with pytest.raises(RuntimeError):
        arm.move_eef_to_goal_pose_wrench(arm.get_current_eef_pose(), [0.0] * 6)


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(axes=[1] * 6),
        dict(axes=AXES_Z, frame="BASE"),
        dict(axes=AXES_Z, max_linear_velocity=[0] * 3),
        dict(axes=AXES_Z, max_linear_velocity=[np.nan] * 3),
        dict(axes=AXES_Z, passive="false"),
        dict(axes=AXES_Z, contact_wrench=[-1] * 6),
        dict(axes=[True] * 6),
    ],
)
def test_invalid_force_config_has_no_side_effects(arm, backend, kwargs):
    with pytest.raises(ValueError):
        arm.configure_force_control(**kwargs)
    assert backend.events == []


def test_moment_control_requires_unlimited_motion_contact_regulation(arm, backend):
    arm.configure_force_control([True] * 6, contact_wrench=[float("inf")] * 6)
    assert ("SetMaxContactWrench", [float("inf")] * 6) in backend.events
    with pytest.raises(ValueError):
        arm.set_max_contact_wrench([10] * 6)


def test_transport_failure_stops_and_propagates(arm, backend):
    arm.configure_force_control(AXES_Z)
    backend.fail_send = True
    with pytest.raises(RuntimeError, match="injected"):
        arm.move_eef_to_goal_pose_wrench(arm.get_current_eef_pose(), [0.0] * 6)
    assert backend.events[-1] == ("Stop",)
    assert arm._force_axes is None


def test_fault_in_wait_stops_and_propagates(arm, backend):
    backend.has_fault = True
    with pytest.raises(RuntimeError, match="fault"):
        arm.wait_for_joints([0.0] * 7)
    assert backend.events[-1] == ("Stop",)


def test_joint_api_has_four_arguments_and_checks_all_joints(arm, backend):
    arm.move_arm_to_joint_values([0.1] * 7, MAX_VEL=[0.2] * 7, MAX_ACC=[0.4] * 7)
    assert backend.events[-1] == ("SendJointPosition", [0.1] * 7, [0.0] * 7, [0.2] * 7, [0.4] * 7)
    for bad in ([0.0] * 6, [0.0] * 8, [0.0] * 6 + [3.0], [np.nan] * 7):
        with pytest.raises(ValueError):
            arm.move_arm_to_joint_values(bad)
    with pytest.raises(ValueError, match="target_acc"):
        arm.move_arm_to_joint_values([0.0] * 7, target_acc=[1.0] * 7)
    with pytest.raises(ValueError, match="velocity"):
        arm.move_arm_to_joint_values([0.0] * 7, target_vel=[1.0] * 7, MAX_VEL=[0.2] * 7)


def test_impedance_bounds_nominal_restore_and_nullspace(arm, backend):
    arm.configure_force_control([False] * 6)
    arm.set_cartesian_impedance([10.0] * 6)
    arm.set_null_space_posture([0.1] * 7)
    arm.set_null_space_objectives(0.2, 0.3, 0.4)
    arm.set_cartesian_impedance()
    assert backend.events[-1] == ("SetCartesianImpedance", backend.info().K_x_nom, [0.7] * 6)
    with pytest.raises(ValueError):
        arm.set_cartesian_impedance([10000.0] * 6)
    with pytest.raises(ValueError):
        arm.set_cartesian_impedance(damping_ratio=[0.9] * 6)
    arm.set_joint_impedance([10.0] * 7)
    arm.move_arm_to_joint_values([0.0] * 7, impedance=True)
    assert backend.mode() == flexivrdk.Mode.NRT_JOINT_IMPEDANCE
    arm.set_max_contact_torque([5.0] * 7)
    arm.set_joint_impedance()
    assert backend.events[-1] == ("SetJointImpedance", backend.info().K_q_nom, [0.7] * 7)


def test_wait_includes_orientation_and_speed_not_only_position(arm, backend):
    target = arm.get_current_eef_pose()
    backend.state.tcp_pose = target[:3] + [float(np.cos(0.25)), 0.0, 0.0, float(np.sin(0.25))]
    assert not arm.wait_for_eef(target, timeout=0.001)
    assert backend.events[-1] == ("Stop",)
    backend.state.tcp_pose = target.copy()
    backend.state.tcp_vel = [0.1, 0, 0, 0, 0, 0]
    assert not arm.wait_for_eef(target, timeout=0.001)
    backend.state.tcp_vel = [0.0] * 6
    assert arm.wait_for_eef(target)


def test_wait_rejects_force_axes(arm):
    arm.configure_force_control(AXES_Z)
    with pytest.raises(RuntimeError, match="convergence"):
        arm.wait_for_eef(arm.get_current_eef_pose())


def test_zero_sensor_dictionary_and_primitive_timeout(arm, backend):
    arm.zero_force_torque_sensors()
    assert backend.events[-1] == ("ExecutePrimitive", "ZeroFTSensor", {}, True)
    backend.terminated = False
    with pytest.raises(TimeoutError):
        arm.zero_force_torque_sensors(timeout=0.001)
    assert backend.events[-1] == ("Stop",)


def test_home_uses_plan_and_plan_timeout(arm, backend):
    arm.go_home()
    assert backend.events[-1] == ("ExecutePlan", "PLAN-Home", False, True)
    backend.plan_busy = True
    with pytest.raises(TimeoutError):
        arm.go_home(timeout=0.001)
    assert backend.events[-1] == ("Stop",)


def test_initialize_timeout_and_fault_recovery(arm, backend):
    backend.has_fault = True
    assert arm.initialize_robot()
    assert [e[0] for e in backend.events] == ["ClearFault", "Enable"]
    backend.is_ready = False
    with pytest.raises(TimeoutError):
        arm.initialize_robot(timeout=0.001)
    assert backend.events[-1] == ("Stop",)


@pytest.mark.parametrize("sn", ["192.168.2.100", "Rizon4s-123456", "Rizon4R-123456"])
def test_wrong_model_or_old_ip_constructor_rejected(backend, sn):
    with pytest.raises(ValueError):
        Bestman_Real_Flexiv(sn, robot=backend)


def test_controller_model_and_dof_must_match(backend):
    backend.information.model_name = "Rizon4s"
    with pytest.raises(ValueError):
        Bestman_Real_Flexiv("Rizon4-123456", robot=backend)


@pytest.mark.parametrize("sdk_version", ["1.9.2", "1.9.4", "1.9.4.1", "2.1.0"])
def test_untested_sdk_rejected_before_constructing_robot(monkeypatch, backend, sdk_version):
    module = importlib.import_module("Robotics_API.Bestman_Real_Flexiv")
    monkeypatch.setattr(module, "version", lambda name: sdk_version)
    with pytest.raises(RuntimeError, match="Rizon 4 requires"):
        Bestman_Real_Flexiv("Rizon4-123456", robot=backend)
    assert backend.events == []


def test_rdk193_accepts_injected_robot(monkeypatch, backend):
    sdk_version = "1.9.3"
    module = importlib.import_module("Robotics_API.Bestman_Real_Flexiv")
    monkeypatch.setattr(module, "version", lambda name: sdk_version)
    arm = Bestman_Real_Flexiv("Rizon4-123456", robot=backend)
    assert arm.robot is backend
    assert arm.rdk_version == sdk_version
    assert backend.events == []


def test_native_sdk_version_handshake_is_not_bypassed(monkeypatch):
    constructor = Mock(spec=flexivrdk.Robot, side_effect=RuntimeError("client is incompatible with robot"))
    monkeypatch.setattr(flexivrdk, "Robot", constructor)
    with pytest.raises(RuntimeError, match="client is incompatible"):
        Bestman_Real_Flexiv("Rizon4-123456")
    constructor.assert_called_once_with("Rizon4-123456", [])


def test_rizon4_fk_ik_with_full_tool_transform(arm, backend):
    tool = [0.01, -0.02, 0.15, 0.8, 0.2, 0.4, 0.4]
    arm.config = replace(arm.config, flange_to_tcp=tuple(tool))
    q = [0.1, -0.3, 0.2, 1.0, -0.1, 0.4, 0.1]
    pose = arm.joints_to_cartesian(q)
    recovered = arm.cartesian_to_joints(pose, q)
    actual = arm.joints_to_cartesian(recovered)
    assert isinstance(pose, list) and len(pose) == 7
    np.testing.assert_allclose(actual[:3], pose[:3], atol=1e-6)
    assert arm.compute_orientation_error(actual[3:], pose[3:]) < 1e-6
    # Check the wxyz output against the independently composed flange/tool transform.
    tool_matrix = np.eye(4)
    tool_matrix[:3, :3] = Rotation.from_quat(tool[3:], scalar_first=True).as_matrix()
    tool_matrix[:3, 3] = tool[:3]
    expected = arm.robot_chain.forward_kinematics([0.0] + q + [0.0]) @ tool_matrix
    np.testing.assert_allclose(pose[:3], expected[:3, 3], atol=1e-6)
    np.testing.assert_allclose(
        Rotation.from_quat(pose[3:], scalar_first=True).as_matrix(), expected[:3, :3], atol=1e-6
    )
    backend.state.q = q.copy()
    assert arm.calculate_IK_error(pose) < 1e-6


def test_config_unknown_keys_invalid_values_and_default_file(tmp_path):
    from pathlib import Path

    assert RobotConfig.load(Path(__file__).parents[1] / "config/rizon4.json") == RobotConfig()
    for kwargs in [dict(frequency=200), dict(max_linear_vel=float("nan")), dict(joint_velocity_scale=2)]:
        with pytest.raises(ValueError):
            RobotConfig(**kwargs)
    path = tmp_path / "bad.json"
    path.write_text('{"unknown": 1}')
    with pytest.raises(TypeError):
        RobotConfig.load(path)


def test_tool_configuration_calls_idle_api_and_updates_fk(arm, backend, monkeypatch):
    events = []

    class Tool:
        def __init__(self, robot):
            assert robot is backend

        def exist(self, name):
            return False

        def Add(self, name, params):
            assert backend.mode() == flexivrdk.Mode.IDLE
            events.append((name, params))

        def Switch(self, name):
            events.append(("Switch", name))

    monkeypatch.setattr(arm.rdk, "Tool", Tool)
    tcp = [0.0, 0.0, 0.12, 0.8, 0.2, 0.4, 0.4]
    arm.configure_tool(
        "measured", mass=0.8, center_of_mass=[0, 0, 0.05], inertia=[0.001] * 3 + [0] * 3, tcp_pose=tcp
    )
    assert events[0][1].mass == 0.8
    assert list(events[0][1].tcp_location) == tcp
    assert arm.config.flange_to_tcp == tuple(tcp)
    assert events[-1] == ("Switch", "measured")


def test_tcp_rotation_uses_wxyz_input_and_output(arm, backend):
    half = float(np.sqrt(0.5))
    backend.state.tcp_pose = [0.4, 0.0, 0.5, half, 0.0, 0.0, half]
    arm.rotate_eef_tcp("x", np.pi / 2)
    np.testing.assert_allclose(backend.events[-1][1], [0.4, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5], atol=1e-12)
