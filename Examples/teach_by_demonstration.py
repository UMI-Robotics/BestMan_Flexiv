"""Record WORLD TCP [x,y,z,qw,qx,qy,qz] (metres, wxyz) to a JSON trajectory."""

import argparse
import json
from pathlib import Path

from Robotics_API import Bestman_Real_Flexiv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument("--config", help="Optional JSON configuration; built-in Rizon 4 defaults otherwise")
    p.add_argument("--plan", default="PLAN-FreeDriveAuto", help="Free-drive plan installed on the controller")
    p.add_argument("--output", type=Path, default=Path("taught_poses.json"))
    args = p.parse_args()

    if args.output.exists():
        p.error("output already exists; choose a new file")

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        arm.switch_mode(arm.mode.NRT_PLAN_EXECUTION)
        arm.execute_plan(args.plan, wait=False)
        poses = []
        print("r: record WORLD TCP [x,y,z,qw,qx,qy,qz]; q: stop and save")
        while True:
            command = input("r/q> ").strip().lower()
            if command == "q":
                break
            if command == "r":
                pose = arm.get_current_eef_pose()
                poses.append(pose)  # Store wxyz unchanged; replay accepts the same lists.
                print(f"Recorded {len(poses)}: {pose} (wxyz)")
        # Stop free-drive before writing the recording to disk.
        arm.stop()
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(poses, stream, indent=2)
        print(f"Saved {args.output.resolve()}; replay with move_eef_follow_trajectory.py")
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
