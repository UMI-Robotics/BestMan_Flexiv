"""Rizon 4R selection, command parity and independently composed upstream kinematics."""

import importlib
import json
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

import flexivrdk
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from Robotics_API import Bestman_Real_Flexiv, RobotConfig
from Robotics_API.config import model_from_serial

REFERENCE = json.loads((Path(__file__).parent / "fixtures/rizon4r_reference.json").read_text())


@pytest.fixture
def right_backend(backend):
    backend.information.model_name = "Rizon4R"
    backend.information.serial_num = "Rizon4R-123456"
    limits = REFERENCE["joint_limits"]
    backend.information.q_min = [limits[f"joint{i}"]["soft_lower"] for i in range(1, 8)]
    backend.information.q_max = [limits[f"joint{i}"]["soft_upper"] for i in range(1, 8)]
    return backend


@pytest.fixture
def right(right_backend):
    return Bestman_Real_Flexiv("Rizon4R-123456", robot=right_backend)


@pytest.mark.parametrize(
    "serial,model",
    [
        ("Rizon4-123456", "Rizon4"),
        ("Rizon 4-123456", "Rizon4"),
        ("Rizon4R-123456", "Rizon4R"),
        ("Rizon 4R-123456", "Rizon4R"),
    ],
)
def test_serial_selects_matching_defaults_without_commands(backend, serial, model):
    backend.information.model_name = model.replace("Rizon", "Rizon ")
    backend.information.serial_num = serial.replace(" ", "")
    arm = Bestman_Real_Flexiv(serial, robot=backend)
    assert arm.config == RobotConfig(model=model)
    assert model_from_serial(serial) == model
    assert backend.events == []


@pytest.mark.parametrize("serial", [None, "", "Rizon4-", "Rizon4R-", "Rizon4R", "Rizon4s-123456", "10.0.0.1"])
def test_invalid_serial_is_rejected_before_sdk_connection(monkeypatch, serial):
    constructor = Mock()
    monkeypatch.setattr(flexivrdk, "Robot", constructor)
    with pytest.raises(ValueError, match="serial"):
        Bestman_Real_Flexiv(serial)
    constructor.assert_not_called()


def test_explicit_config_model_mismatch_is_rejected_before_connection(monkeypatch, tmp_path):
    constructor = Mock()
    monkeypatch.setattr(flexivrdk, "Robot", constructor)
    path = tmp_path / "wrong.json"
    path.write_text('{"model":"Rizon4"}')
    for config in (RobotConfig(), path):
        with pytest.raises(ValueError, match="does not match serial model"):
            Bestman_Real_Flexiv("Rizon4R-123456", config=config)
    constructor.assert_not_called()


def test_config_without_model_inherits_serial_model(right_backend, tmp_path):
    path = tmp_path / "shared_limits.json"
    path.write_text('{"frequency":30,"max_linear_vel":0.03}')
    arm = Bestman_Real_Flexiv("Rizon4R-123456", config=path, robot=right_backend)
    assert arm.config.model == "Rizon4R"
    assert arm.frequency == 30
    assert arm.config.max_linear_vel == 0.03
    assert RobotConfig.load(Path(__file__).parents[1] / "config/rizon4r.json") == RobotConfig(model="Rizon4R")


@pytest.mark.parametrize(
    "field,value",
    [
        ("model_name", "Rizon4"),
        ("DoF", 6),
        ("DoF_e", 1),
        ("serial_num", "Rizon4R-999999"),
    ],
)
def test_controller_identity_and_shape_must_match(right_backend, field, value):
    setattr(right_backend.information, field, value)
    with pytest.raises(ValueError):
        Bestman_Real_Flexiv("Rizon4R-123456", robot=right_backend)
    assert right_backend.events == []


def test_rizon4r_keeps_native_version_handshake(monkeypatch):
    constructor = Mock(spec=flexivrdk.Robot, side_effect=RuntimeError("native controller version mismatch"))
    monkeypatch.setattr(flexivrdk, "Robot", constructor)
    with pytest.raises(RuntimeError, match="native controller version mismatch"):
        Bestman_Real_Flexiv("Rizon4R-123456")
    constructor.assert_called_once_with("Rizon4R-123456", [])


def test_rizon4r_keeps_rdk193_pin(monkeypatch, right_backend):
    module = importlib.import_module("Robotics_API.Bestman_Real_Flexiv")
    monkeypatch.setattr(module, "version", lambda name: "2.1.0")
    with pytest.raises(RuntimeError, match="requires flexivrdk==1.9.3"):
        Bestman_Real_Flexiv("Rizon4R-123456", robot=right_backend)
    assert right_backend.events == []


def test_rizon4r_cartesian_force_and_impedance_do_not_mirror_commands(right, right_backend):
    target = [0.4, -0.2, 0.5, 0.8, 0.2, 0.4, 0.4]
    transform = [0.01, -0.02, 0.03, 0.8, 0.2, 0.4, 0.4]
    right.configure_force_control(
        [False, False, True, False, False, False], frame="TCP", frame_transform=transform
    )
    right.servo_cartesian_motion_force(target, [0, 0, 3, 0, 0, 0], [0.0] * 6)
    sent = right_backend.events[-1]
    assert sent[0] == "SendCartesianMotionForce"
    assert sent[1:4] == (target, [0, 0, 3, 0, 0, 0], [0.0] * 6)
    assert ("SetForceControlFrame", flexivrdk.CoordType.TCP, transform) in right_backend.events
    right.set_cartesian_impedance()
    assert right_backend.events[-1][1] == right_backend.information.K_x_nom
    right.servo_cartesian_pose(target)
    assert right_backend.events[-1][1:3] == (target, [0.0] * 6)
    right_backend.state.tcp_pose = target.copy()
    assert right.get_current_eef_pose() == target
    assert target == [0.4, -0.2, 0.5, 0.8, 0.2, 0.4, 0.4]


def test_rizon4r_joint_servo_uses_controller_limits_and_signs(right, right_backend):
    target = [0.1, -0.2, 0.3, -0.4, 0.5, 0.6, -0.7]
    right.servo_joint_position(target)
    assert right_backend.events[-1][0:2] == ("SendJointPosition", target)
    right_backend.information.q_max[6] = 0.2
    with pytest.raises(ValueError):
        right.servo_joint_position([0.0] * 6 + [0.3])
    assert right_backend.events[-1] == ("Stop",)


def reference_fk(joints):
    """Compose upstream numeric origins/axes without reading the packaged URDF or IKPy."""
    transform = np.eye(4)
    for index, name in enumerate([f"joint{i}" for i in range(1, 8)] + ["link7_to_flange"]):
        data = REFERENCE["kinematics"][name]
        origin = np.eye(4)
        origin[:3, :3] = Rotation.from_euler("xyz", [data[k] for k in ("roll", "pitch", "yaw")]).as_matrix()
        origin[:3, 3] = [data[k] for k in ("x", "y", "z")]
        transform = transform @ origin
        if index < 7:
            rotation = np.eye(4)
            axis = np.fromstring(data.get("axis", "0 0 1"), sep=" ")
            rotation[:3, :3] = Rotation.from_rotvec(axis * joints[index]).as_matrix()
            transform = transform @ rotation
    return transform


@pytest.mark.parametrize("seed", range(6))
def test_rizon4r_fk_and_ik_match_independent_upstream_frames(right, seed):
    q = np.random.default_rng(seed).uniform(-0.5, 0.5, 7)
    tool = [0.01, -0.02, 0.15, 0.8, 0.2, 0.4, 0.4]
    right.config = replace(right.config, flange_to_tcp=tuple(tool))
    tool_matrix = np.eye(4)
    tool_matrix[:3, :3] = Rotation.from_quat(tool[3:], scalar_first=True).as_matrix()
    tool_matrix[:3, 3] = tool[:3]
    expected = reference_fk(q) @ tool_matrix
    actual = right.joints_to_cartesian(q)
    np.testing.assert_allclose(actual[:3], expected[:3, 3], atol=1e-10)
    np.testing.assert_allclose(
        Rotation.from_quat(actual[3:], scalar_first=True).as_matrix(), expected[:3, :3], atol=1e-10
    )
    solved = right.cartesian_to_joints(actual, q + 0.02)
    restored = right.joints_to_cartesian(solved)
    np.testing.assert_allclose(restored[:3], actual[:3], atol=1e-5)
    assert right.compute_orientation_error(restored[3:], actual[3:]) < 1e-5


def test_models_have_distinct_flange_kinematics(right, backend):
    # Construct the left backend separately; right_backend has mutated this fixture.
    left_backend = type(backend)()
    left = Bestman_Real_Flexiv("Rizon4-123456", robot=left_backend)
    q = [0.2, -0.3, 0.4, 0.5, -0.2, 0.6, -0.1]
    assert not np.allclose(left.joints_to_cartesian(q), right.joints_to_cartesian(q))
    for joint, link in zip(range(1, 8), right.active_joints):
        limits = REFERENCE["joint_limits"][f"joint{joint}"]
        assert link.bounds == (limits["soft_lower"], limits["soft_upper"])
