"""Send NRT joint targets directly at 1-100 Hz with Rizon RDK 1.9.4.1."""

import argparse
import math
import time

from Robotics_API import Bestman_Real_Flexiv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument("--config", help="Optional JSON configuration; built-in Rizon 4 defaults otherwise")
    p.add_argument("--duration", type=float, default=5.0, help="Control duration in seconds")
    p.add_argument("--sine-frequency", type=float, default=0.25, help="Motion frequency in Hz")
    p.add_argument("--max-lag", type=float, default=0.1, help="Observed dispatch/command lag limit (s)")

    p.add_argument("--joint", type=int, choices=range(7), default=6, help="Zero-based joint index")
    p.add_argument("--amplitude", type=float, default=0.0, help="Sine amplitude (rad); 0 holds position")
    p.add_argument("--impedance", action="store_true", help="Use NRT joint impedance mode")
    args = p.parse_args()

    for name in ("duration", "sine_frequency", "max_lag"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            p.error(f"--{name.replace('_', '-')} must be positive and finite")
    if not math.isfinite(args.amplitude):
        p.error("--amplitude must be finite")
    omega = 2.0 * math.pi * args.sine_frequency

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        # Clear faults, enable the motors and wait until the robot is operational.
        arm.initialize_robot()
        initial = arm.get_current_joint_values()
        lower, upper = arm.get_joint_bounds()[args.joint]
        if (
            initial[args.joint] - abs(args.amplitude) < lower
            or initial[args.joint] + abs(args.amplitude) > upper
        ):
            raise ValueError("Sine trajectory exceeds controller joint limits")
        vmax = [arm.config.joint_velocity_scale * v for v in arm.get_robot_info().dq_max]
        if (
            abs(args.amplitude) * omega > vmax[args.joint]
            or abs(args.amplitude) * omega**2 > arm.config.joint_max_acc
        ):
            raise ValueError("Sine peak velocity/acceleration exceeds the configured motion limits")
        # Finish the blocking mode change before starting the timed loop.
        if args.impedance:
            arm.switch_mode(arm.mode.NRT_JOINT_IMPEDANCE)
            arm.set_joint_impedance()
        else:
            arm.switch_mode(arm.mode.NRT_JOINT_POSITION)
        # Send one target per tick. This is Python NRT timing, not a 1 kHz RT loop.
        period = 1.0 / arm.frequency
        started = next_tick = time.monotonic()
        deadline = started + args.duration
        while (now := time.monotonic()) < deadline:
            if now - next_tick > args.max_lag:
                raise TimeoutError("Servo loop missed its dispatch deadline")
            elapsed = now - started
            target = initial.copy()
            target[args.joint] += args.amplitude * math.sin(omega * elapsed)
            velocity = [0.0] * 7
            velocity[args.joint] = args.amplitude * omega * math.cos(omega * elapsed)
            arm.servo_joint_position(target, velocity, max_vel=vmax, impedance=args.impedance)

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
