"""Bounded force control; TCP poses are [x,y,z,qw,qx,qy,qz] (wxyz)."""

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

    p.add_argument("--frame", choices=("WORLD", "TCP"), default="WORLD")
    p.add_argument("--axes", type=int, choices=(0, 1), nargs=6, default=[0, 0, 1, 0, 0, 0])
    p.add_argument(
        "--wrench",
        type=float,
        nargs=6,
        default=[0.0, 0.0, 3.0, 0.0, 0.0, 0.0],
        help="Sensed Fx Fy Fz Mx My Mz (N, Nm); WORLD +Fz presses toward -Z",
    )
    p.add_argument("--passive", action="store_true", help="Open-loop wrench feedforward")
    p.add_argument("--disable-contact-regulation", action="store_true", help="Required for moment control")
    args = p.parse_args()

    for name in ("duration", "max_displacement", "abort_force", "abort_moment"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            p.error(f"--{name.replace('_', '-')} must be positive and finite")

    if not np.isfinite(args.wrench).all() or any(w and not axis for w, axis in zip(args.wrench, args.axes)):
        p.error("wrench must be finite and nonzero only on enabled axes")
    if (
        np.linalg.norm(args.wrench[:3]) >= args.abort_force
        or np.linalg.norm(args.wrench[3:]) >= args.abort_moment
    ):
        p.error("target wrench must be below measured abort limits")
    if any(args.axes[3:]) and not args.disable_contact_regulation:
        p.error("moment control requires --disable-contact-regulation")

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        hold = arm.get_current_eef_pose()  # WORLD [x,y,z,qw,qx,qy,qz], metres and wxyz.
        # Configure IDLE-only passive setting, then NRT Cartesian mode, force frame and axes.
        arm.configure_force_control(
            [bool(axis) for axis in args.axes],
            frame=args.frame,
            passive=args.passive,
            contact_wrench=[float("inf")] * 6 if args.disable_contact_regulation else None,
        )
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
            arm.move_eef_to_goal_pose_wrench(hold, args.wrench)
            time.sleep(1.0 / arm.frequency)
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
