"""Read status, plans, IO and actions without enabling or moving the robot."""

import argparse
import json

from Robotics_API import Bestman_Real_Flexiv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument("--config", help="Optional JSON configuration; built-in Rizon 4 defaults otherwise")

    args = p.parse_args()

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    print(json.dumps(arm.get_status(), default=str, indent=2))
    plan = arm.get_plan_info()
    print(
        "Current plan:",
        json.dumps(
            {
                "name": plan.assigned_plan_name,
                "primitive": plan.pt_name,
                "node": plan.node_name,
                "waiting_for_step": plan.waiting_for_step,
            }
        ),
    )
    print("Plans:", arm.get_plan_list())
    print("Digital inputs (0-17):", arm.get_digital_inputs())
    print("Events:", arm.get_event_log())
    print("Target TCP [x,y,z,qw,qx,qy,qz] (wxyz):", arm.get_robot_actions().tcp_pose_d)


if __name__ == "__main__":
    main()
