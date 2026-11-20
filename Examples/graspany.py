"""Move to WORLD [x,y,z,qw,qx,qy,qz] (wxyz), wait, then close the Robotiq gripper."""

import argparse

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
    p.add_argument("--pose", type=float, nargs=7, required=True, help="WORLD x y z qw qx qy qz (m, wxyz)")
    p.add_argument("--port", required=True, help="Robotiq serial device, e.g. /dev/ttyUSB0")
    p.add_argument("--speed", type=int, default=64)
    p.add_argument("--gripper-force", type=int, default=64, help="Raw 0-255 command, not Newtons")
    args = p.parse_args()

    target = pose_vector(args.pose)
    if any(not 0 <= value <= 255 for value in [args.speed, args.gripper_force]):
        p.error("gripper commands must be in [0,255]")

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        arm.configure_force_control([False] * 6)
        arm.move_eef_to_goal_pose(target)
        if not arm.wait_for_eef(target):
            raise TimeoutError("Grasp target not reached")
        arm.connect_gripper(args.port)
        arm.close_gripper(args.speed, args.gripper_force)
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
