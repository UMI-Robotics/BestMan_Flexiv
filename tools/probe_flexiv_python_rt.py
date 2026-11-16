"""Inspect Python RDK exports without constructing, connecting to, or enabling a robot."""

from __future__ import annotations

import argparse
import importlib
import json
import platform
import sys
from importlib import metadata
from pathlib import Path

METHODS = (
    "StreamJointPosition",
    "StreamJointTorque",
    "StreamCartesianMotionForce",
    "SendJointPosition",
    "SendCartesianMotionForce",
)
MODES = ("RT_JOINT_POSITION", "RT_JOINT_IMPEDANCE", "RT_JOINT_TORQUE", "RT_CARTESIAN_MOTION_FORCE")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, help="Optional extracted official wheel to inspect")
    parser.add_argument("--output", type=Path, help="Write the JSON report to this path")
    args = parser.parse_args()
    if args.package_dir:
        sys.path.insert(0, str(args.package_dir.resolve()))
    rdk = importlib.import_module("flexivrdk")
    native = importlib.import_module(rdk.Robot.__module__)
    probes = {}
    for name in METHODS:
        try:
            method = getattr(rdk.Robot, name)
        except AttributeError as error:
            probes[name] = {"available": False, "error": str(error)}
        else:
            probes[name] = {
                "available": True,
                "callable": callable(method),
                "signature": (method.__doc__ or "").splitlines()[0],
            }
    command_construction = {}
    if all(hasattr(rdk, name) for name in ("RtJointPositionCmd", "RtJointTorqueCmd", "RtCartesianCmd")):
        # Construct native command data only. No Robot instance or command dispatch.
        positions = rdk.RtJointPositionCmd([0.0] * 7, [0.0] * 7, [0.0] * 7)
        torques = rdk.RtJointTorqueCmd([0.0] * 7)
        pose = [0.4, 0.1, 0.5, 0.8, 0.2, 0.4, 0.4]  # wxyz unit quaternion
        cartesian = rdk.RtCartesianCmd(pose)
        command_construction = {
            "joint_position": list(positions.q_d) == [0.0] * 7,
            "joint_torque": list(torques.tau_d) == [0.0] * 7,
            "cartesian_pose_wxyz_preserved": list(cartesian.pose_d) == pose,
        }
    report = {
        "sdk_distribution_version": metadata.version("flexivrdk"),
        "sdk_module_version": getattr(rdk, "__version__", None),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "module": str(Path(rdk.__file__).resolve()),
        "native_module": str(Path(native.__file__).resolve()),
        "public_robot_is_native_robot": rdk.Robot is native.Robot,
        "methods": probes,
        "rt_modes": {name: hasattr(rdk.Mode, name) for name in MODES},
        "scheduler_available": hasattr(rdk, "Scheduler"),
        "native_rt_methods": {
            name: hasattr(native.Robot, name) for name in METHODS if name.startswith("Stream")
        },
        "native_rt_command_construction": command_construction,
        "robot_constructed": False,
        "scope": "Imports, attribute lookup and native command data construction only; no robot connection, command transmission or timing benchmark",
    }
    result = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result, encoding="utf-8")
    print(result, end="")


if __name__ == "__main__":
    main()
