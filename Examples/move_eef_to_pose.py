"""Move to a WORLD TCP target [x,y,z,qw,qx,qy,qz] (metres, wxyz)."""

import argparse

from Robotics_API import Bestman_Real_Flexiv
from Robotics_API.config import pose_vector


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument("--config", help="Optional JSON configuration; built-in Rizon 4 defaults otherwise")
    p.add_argument("--pose", type=float, nargs=7, required=True, help="WORLD x y z qw qx qy qz (m, wxyz)")
    args = p.parse_args()

    target = pose_vector(args.pose)

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        # Enter NRT Cartesian mode with all six axes under motion control.
        arm.configure_force_control([False] * 6)
        arm.move_eef_to_goal_pose(target)
        if not arm.wait_for_eef(target):
            raise TimeoutError("TCP target not reached")
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
