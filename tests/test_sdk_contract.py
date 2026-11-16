"""Verify against the actual installed binary, without connecting to any robot."""

import ast
import re
import subprocess
import sys
from importlib.metadata import version
from pathlib import Path

import flexivrdk
import pytest

ROOT = Path(__file__).parents[1]


def test_all_wrapper_robot_methods_exist_in_real_sdk():
    assert version("flexivrdk") == "1.9.4.1"
    tree = ast.parse((ROOT / "Robotics_API/Bestman_Real_Flexiv.py").read_text())
    methods = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Attribute)
        and isinstance(node.func.value.value, ast.Name)
        and node.func.value.value.id == "self"
        and node.func.value.attr == "robot"
    }
    assert methods
    for name in methods:
        assert callable(getattr(flexivrdk.Robot, name)), name


@pytest.mark.parametrize(
    "method,arguments",
    [
        ("SendJointPosition", ["self", "positions", "velocities", "max_vel", "max_acc"]),
        (
            "SendCartesianMotionForce",
            [
                "self",
                "pose",
                "wrench",
                "velocity",
                "max_linear_vel",
                "max_angular_vel",
                "max_linear_acc",
                "max_angular_acc",
            ],
        ),
        ("SetForceControlAxis", ["self", "enabled_axes", "max_linear_vel"]),
        ("SetForceControlFrame", ["self", "root_coord", "T_in_root"]),
        ("SetPassiveForceControl", ["self", "is_enabled"]),
        ("SetJointImpedance", ["self", "K_q", "Z_q"]),
        ("SetCartesianImpedance", ["self", "K_x", "Z_x"]),
    ],
)
def test_binary_signatures_for_control_calls(method, arguments):
    signature = getattr(flexivrdk.Robot, method).__doc__.splitlines()[0]
    assert re.findall(r"\b(\w+): ", signature) == arguments


def test_binary_state_and_tool_fields():
    for name in ("tcp_pose", "tcp_vel", "ext_wrench_in_world", "ext_wrench_in_tcp", "tau_ext"):
        assert hasattr(flexivrdk.RobotStates, name)
    for name in ("q_min", "q_max", "dq_max", "K_x_nom", "K_q_nom", "model_name", "DoF_e"):
        assert hasattr(flexivrdk.RobotInfo, name)
    for name in ("Add", "Update", "Switch", "params", "name", "exist"):
        assert callable(getattr(flexivrdk.Tool, name))


EXAMPLES = sorted(p.stem for p in (ROOT / "Examples").glob("*.py") if not p.name.startswith("_"))


@pytest.mark.parametrize("name", EXAMPLES)
def test_example_help_works_without_ros_gripper_or_robot(name):
    result = subprocess.run(
        [sys.executable, "-m", f"Examples.{name}", "--help"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout


def test_entire_python_robot_inventory_is_documented_and_accounted_for():
    doc = (ROOT / "docs/PYTHON_SDK_COVERAGE.md").read_text()
    section = doc.split("## Robot API inventory\n", 1)[1].split("\n## ", 1)[0]
    documented = set(re.findall(r"^\| `([A-Za-z_]+)` \|", section, re.M))
    actual = {name for name in dir(flexivrdk.Robot) if not name.startswith("_")}
    assert documented == actual
    tree = ast.parse((ROOT / "Robotics_API/Bestman_Real_Flexiv.py").read_text())
    called = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Attribute)
        and isinstance(node.func.value.value, ast.Name)
        and node.func.value.value.id == "self"
        and node.func.value.attr == "robot"
    }
    assert actual - called == {"Brake", "LockExternalAxes", "SyncWithPositioner", "SetTimelinessFailureLimit"}


def test_rizon_1_9_4_1_binding_excludes_rt_and_servo_on_api():
    assert version("flexivrdk") == "1.9.4.1"  # RDK 2.1.0 DOES export RT methods/modes.
    for name in (
        "StreamJointPosition",
        "StreamJointTorque",
        "StreamCartesianMotionForce",
        "ServoOn",
        "Disable",
    ):
        assert not hasattr(flexivrdk.Robot, name)
    assert not any(name.startswith("RT_") for name in dir(flexivrdk.Mode))
    assert not hasattr(flexivrdk, "Scheduler")


@pytest.mark.parametrize(
    "method,arguments",
    [
        ("SendMultiCartesianMotionForce", ["self", "cart_cmds", "joint_pos"]),
        ("SetJointInertiaScale", ["self", "inertia_scales"]),
        ("SetDigitalOutputs", ["self", "digital_outputs"]),
        ("SetGlobalVariables", ["self", "global_vars"]),
        ("PausePlan", ["self", "toggle"]),
        ("SetBreakpointMode", ["self", "is_enabled"]),
        ("SetVelocityScale", ["self", "velocity_scale"]),
        ("ClearFault", ["self", "timeout_sec"]),
        ("ExecutePrimitive", ["self", "primitive_name", "input_params", "block_until_started"]),
    ],
)
def test_additional_sdk_command_signatures(method, arguments):
    signature = getattr(flexivrdk.Robot, method).__doc__.splitlines()[0]
    assert re.findall(r"\b(\w+): ", signature) == arguments
