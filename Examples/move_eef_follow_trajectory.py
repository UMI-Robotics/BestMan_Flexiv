"""Replay JSON waypoints: WORLD [x,y,z,qw,qx,qy,qz] (metres, wxyz)."""

import argparse
import json
from pathlib import Path

from Robotics_API import Bestman_Real_Flexiv
from Robotics_API.config import pose_vector


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon4/Rizon4R serial, e.g. Rizon4R-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument(
        "--config", help="Optional JSON configuration; model defaults inferred from the serial otherwise"
    )
    p.add_argument("--trajectory", type=Path, required=True, help="JSON list of seven-element waypoints")
    args = p.parse_args()

    # Read the user-supplied file before connecting. Relative paths use the current directory.
    waypoints = json.loads(args.trajectory.read_text(encoding="utf-8"))
    if not isinstance(waypoints, list) or not waypoints:
        p.error("trajectory must be a nonempty JSON list")
    waypoints = [pose_vector(row) for row in waypoints]

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        arm.configure_force_control([False] * 6)
        # Teaching records already use wxyz; send them without reordering.
        for target in waypoints:
            arm.move_eef_to_goal_pose(target)
            if not arm.wait_for_eef(target):
                raise TimeoutError("TCP waypoint not reached")
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
