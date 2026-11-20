"""Send NRT Cartesian targets directly; WORLD [x,y,z,qw,qx,qy,qz] (metres, wxyz)."""

import argparse
import math
import time

import numpy as np

from Robotics_API import Bestman_Real_Flexiv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon4/Rizon4R serial, e.g. Rizon4R-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument(
        "--config", help="Optional JSON configuration; model defaults inferred from the serial otherwise"
    )
    p.add_argument("--duration", type=float, default=5.0, help="Control duration in seconds")
    p.add_argument("--sine-frequency", type=float, default=0.25, help="Motion frequency in Hz")
    p.add_argument("--max-lag", type=float, default=0.1, help="Observed dispatch/command lag limit (s)")

    p.add_argument("--axis", choices=("x", "y", "z"), default="x")
    p.add_argument("--amplitude", type=float, default=0.0, help="Translation sine amplitude (m); 0 holds")
    p.add_argument(
        "--force-z", type=float, default=0.0, help="Sensed WORLD Fz (N); positive presses toward -Z"
    )
    p.add_argument("--max-displacement", type=float, default=0.03, help="Measured TCP travel limit (m)")
    p.add_argument("--abort-force", type=float, default=12.0, help="Measured force norm limit (N)")
    p.add_argument("--abort-moment", type=float, default=3.0, help="Measured moment norm limit (Nm)")
    args = p.parse_args()

    for name in ("duration", "sine_frequency", "max_lag", "max_displacement", "abort_force", "abort_moment"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            p.error(f"--{name.replace('_', '-')} must be positive and finite")
    if not math.isfinite(args.amplitude):
        p.error("--amplitude must be finite")
    omega = 2.0 * math.pi * args.sine_frequency

    if not math.isfinite(args.force_z):
        p.error("--force-z must be finite")
    if abs(args.amplitude) >= args.max_displacement or abs(args.force_z) >= args.abort_force:
        p.error("Targets must stay below measured displacement/force abort limits")
    if args.force_z and args.axis == "z" and args.amplitude:
        p.error("Choose x/y motion when Z is force-controlled")
    axis = {"x": 0, "y": 1, "z": 2}[args.axis]

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        if (
            abs(args.amplitude) * omega > arm.config.max_linear_vel
            or abs(args.amplitude) * omega**2 > arm.config.max_linear_acc
        ):
            raise ValueError("Sine peak velocity/acceleration exceeds the configured motion limits")
        # Enter NRT Cartesian mode. Z is force-controlled only when --force-z is nonzero.
        arm.configure_force_control([False, False, bool(args.force_z), False, False, False], frame="WORLD")
        hold = arm.get_current_eef_pose()  # [x,y,z,qw,qx,qy,qz], SDK wxyz order.
        # Send one target per tick. This is Python NRT timing, not a 1 kHz RT loop.
        period = 1.0 / arm.frequency
        started = next_tick = time.monotonic()
        deadline = started + args.duration
        while (now := time.monotonic()) < deadline:
            if now - next_tick > args.max_lag:
                raise TimeoutError("Servo loop missed its dispatch deadline")
            elapsed = now - started
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
            target = hold.copy()  # Keep wxyz unchanged; modify only a WORLD translation component.
            target[axis] += args.amplitude * math.sin(omega * elapsed)
            velocity = [0.0] * 6
            velocity[axis] = args.amplitude * omega * math.cos(omega * elapsed)
            if args.force_z:
                arm.servo_cartesian_motion_force(target, [0.0, 0.0, args.force_z, 0.0, 0.0, 0.0], velocity)
            else:
                arm.servo_cartesian_pose(target, velocity)

            after = time.monotonic()
            if after - next_tick > args.max_lag:
                raise TimeoutError("Servo command exceeded max_lag")
            next_tick += period
            if next_tick <= after:
                next_tick = after + period  # Skip missed periods; never send a catch-up burst.
            delay = min(next_tick, deadline) - after
            if delay > 0:
                time.sleep(delay)
    finally:
        # Stop and return to IDLE on completion, exceptions or Ctrl+C.
        arm.stop()


if __name__ == "__main__":
    main()
