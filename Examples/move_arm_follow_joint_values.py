"""Replay JSON waypoints: seven joint positions in radians."""

import argparse
import json
from pathlib import Path

from Robotics_API import Bestman_Real_Flexiv
from Robotics_API.config import vector


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument("--config", help="Optional JSON configuration; built-in Rizon 4 defaults otherwise")
    p.add_argument("--trajectory", type=Path, required=True, help="JSON list of seven-element waypoints")
    args = p.parse_args()

    # Read the user-supplied file before connecting. Relative paths use the current directory.
    waypoints = json.loads(args.trajectory.read_text(encoding="utf-8"))
    if not isinstance(waypoints, list) or not waypoints:
        p.error("trajectory must be a nonempty JSON list")
    waypoints = [vector(row, 7, "joint waypoint") for row in waypoints]

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        # Check all waypoints against controller joint bounds before sending the first one.
        bounds = arm.get_joint_bounds()
        for target in waypoints:
            if any(not lower <= value <= upper for value, (lower, upper) in zip(target, bounds)):
                raise ValueError("Joint waypoint exceeds controller limits")
        arm.switch_mode(arm.mode.NRT_JOINT_POSITION)
        for target in waypoints:
            arm.move_arm_to_joint_values(target)
            if not arm.wait_for_joints(target):
                raise TimeoutError("Joint waypoint not reached")
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
