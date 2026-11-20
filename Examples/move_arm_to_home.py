"""Move Rizon 4/4R to the controller's Home posture."""

import argparse

from Robotics_API import Bestman_Real_Flexiv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon4/Rizon4R serial, e.g. Rizon4R-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument(
        "--config", help="Optional JSON configuration; model defaults inferred from the serial otherwise"
    )

    args = p.parse_args()

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        # Select plan execution, then run PLAN-Home and wait for completion.
        arm.switch_mode(arm.mode.NRT_PLAN_EXECUTION)
        arm.go_home()
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
