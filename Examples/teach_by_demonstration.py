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
    p.add_argument(
        "--plan",
        default="PLAN-FreeDriveAuto",
        help="Free-drive plan; PLAN-FreeDriveAuto requires holding the Motion Bar enabling button",
    )
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
        if args.plan == "PLAN-FreeDriveAuto":
            print("Keep the robot in AUTO (REMOTE).")
            print("Press and HOLD the enabling button on the Motion Bar while guiding the arm.")
            print("Releasing the button returns this plan to Hold. You may release it before recording.")
        else:
            print(f"Using custom plan {args.plan!r}; follow that plan's free-drive activation conditions.")
        print("r: record WORLD TCP [x,y,z,qw,qx,qy,qz]; s: show current controller status; q: stop and save")
        # Read status at startup and after each command, including after a blocking input().
        command = "s"
        while command != "q":
            status = arm.get_status()
            plan = arm.get_plan_info()
            button = "PRESSED" if status["enabling_button_pressed"] else "RELEASED"
            print(
                f"mode={status['mode']} plan={plan.assigned_plan_name} "
                f"primitive={plan.pt_name} node={plan.node_name} enabling_button={button} "
                f"operational={status['operational']} fault={status['fault']} "
                f"busy={status['busy']} waiting_for_step={plan.waiting_for_step}"
            )
            active = (
                status["connected"]
                and status["operational"]
                and not status["fault"]
                and status["mode"] == arm.mode.NRT_PLAN_EXECUTION
                and status["busy"]
                and plan.assigned_plan_name == args.plan
                and not plan.waiting_for_step
            )
            if not active:
                print("Teaching plan is not active/ready. Check the plan, mode and fault in Elements.")
            elif args.plan == "PLAN-FreeDriveAuto" and not status["enabling_button_pressed"]:
                print("Waiting for the Motion Bar enabling button; hold it to activate free drive.")
            if command == "r":
                if active:
                    pose = arm.get_current_eef_pose()
                    poses.append(pose)  # Store wxyz unchanged; replay accepts the same lists.
                    print(f"Recorded {len(poses)}: {pose} (wxyz)")
                else:
                    print("Pose not recorded. Use s to check status or q to save existing poses and exit.")
            elif command != "s":
                print("Use r to record, s for status, or q to stop and save.")
            command = input("r/s/q> ").strip().lower()
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
