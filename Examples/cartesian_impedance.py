"""Bounded cartesian impedance; TCP poses are [x,y,z,qw,qx,qy,qz] (wxyz)."""

import argparse
import math
import time

import numpy as np

from Robotics_API import Bestman_Real_Flexiv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument("--config", help="Optional JSON configuration; built-in Rizon 4 defaults otherwise")
    p.add_argument("--duration", type=float, default=5.0, help="Control duration in seconds")
    p.add_argument("--max-displacement", type=float, default=0.03, help="Measured TCP travel limit (m)")
    p.add_argument("--abort-force", type=float, default=12.0, help="Measured force norm limit (N)")
    p.add_argument("--abort-moment", type=float, default=3.0, help="Measured moment norm limit (Nm)")

    p.add_argument("--stiffness-scale", type=float, default=0.3, help="Fraction of nominal stiffness, (0,1]")
    args = p.parse_args()

    for name in ("duration", "max_displacement", "abort_force", "abort_moment"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            p.error(f"--{name.replace('_', '-')} must be positive and finite")

    if not math.isfinite(args.stiffness_scale) or not 0 < args.stiffness_scale <= 1:
        p.error("--stiffness-scale must be in (0,1]")

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        hold = arm.get_current_eef_pose()  # WORLD [x,y,z,qw,qx,qy,qz], metres and wxyz.
        # Enter NRT Cartesian mode; all axes remain under motion control.
        arm.configure_force_control([False] * 6)
        arm.set_cartesian_impedance([args.stiffness_scale * k for k in arm.get_robot_info().K_x_nom])
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            states = arm.update_robot_states()
            wrench = np.asarray(states.ext_wrench_in_world)
            displacement = np.linalg.norm(np.asarray(states.tcp_pose[:3]) - hold[:3])
            if not np.isfinite(wrench).all() or not np.isfinite(displacement):
                raise RuntimeError("Invalid force/pose measurement")
            if (
                displacement > args.max_displacement
                or np.linalg.norm(wrench[:3]) > args.abort_force
                or np.linalg.norm(wrench[3:]) > args.abort_moment
            ):
                raise RuntimeError("Measured displacement or wrench exceeded the example limit")
            arm.move_eef_to_goal_pose_wrench(hold, [0.0] * 6)
            time.sleep(1.0 / arm.frequency)
        arm.set_cartesian_impedance()  # Restore nominal gains.
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
