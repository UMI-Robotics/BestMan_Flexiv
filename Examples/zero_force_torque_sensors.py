"""Zero force/torque sensors while stationary, with calibrated tools and no contact."""

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
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        arm.switch_mode(arm.mode.NRT_PRIMITIVE_EXECUTION)
        # Run ZeroFTSensor and wait for completion; no automatic Home motion.
        arm.zero_force_torque_sensors()
        print("External WORLD wrench [Fx,Fy,Fz,Mx,My,Mz] (N, Nm):", arm.get_tcp_wrench())
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
