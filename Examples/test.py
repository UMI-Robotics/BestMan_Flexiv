"""Compare offline base-frame FK/IK with controller WORLD TCP (wxyz); no motion."""

import argparse

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
    joints = arm.get_current_joint_values()
    pose = arm.joints_to_cartesian(joints)  # [x,y,z,qw,qx,qy,qz], metres and wxyz.
    solved = arm.cartesian_to_joints(pose, joints)
    print("Offline base-frame FK [x,y,z,qw,qx,qy,qz]:", pose)
    print("Offline IK roundtrip (radians):", solved)
    print(
        "Controller WORLD TCP (requires matching world/tool calibration to compare):",
        arm.get_current_eef_pose(),
    )


if __name__ == "__main__":
    main()
