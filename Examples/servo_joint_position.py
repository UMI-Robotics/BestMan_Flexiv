"""NRT joint servo at 1–100 Hz. Python RDK does not expose the 1 kHz RT Stream* APIs."""

import math

from Examples._common import parser, positive, robot_session
from Robotics_API.config import vector


def main():
    p = parser(__doc__)
    p.add_argument("--duration", type=positive, default=5.0)
    p.add_argument("--joint", type=int, choices=range(7), default=6, help="Zero-based joint index")
    p.add_argument("--amplitude", type=float, default=0.0, help="Sine amplitude in radians; 0 holds position")
    p.add_argument("--sine-frequency", type=positive, default=0.25, help="Motion frequency in Hz")
    p.add_argument(
        "--impedance", action="store_true", help="Use NRT joint impedance instead of position mode"
    )
    p.add_argument(
        "--max-lag", type=positive, default=0.1, help="Maximum observed loop/callback lag in seconds"
    )
    args = p.parse_args()
    amplitude = vector([args.amplitude], 1, "amplitude")[0]
    omega = 2.0 * math.pi * args.sine_frequency
    with robot_session(args) as arm:
        initial = arm.get_current_joint_values()
        # Validate both extrema and peak derivatives before the first servo command.
        for direction in (-1.0, 1.0):
            target = initial.copy()
            target[args.joint] += direction * abs(amplitude)
            arm._joints(target)
        vmax = [arm.config.joint_velocity_scale * v for v in arm.robot.info().dq_max]
        if abs(amplitude) * omega > vmax[args.joint] or abs(amplitude) * omega**2 > arm.config.joint_max_acc:
            raise ValueError("Sine peak velocity/acceleration exceeds the configured motion limits")
        if args.impedance:
            arm.set_joint_impedance()  # Explicit nominal gains; change them here for a compliant task.
        else:
            # Finish the blocking mode transition before starting the timed loop.
            arm.switch_mode(arm.mode.NRT_JOINT_POSITION)

        def update(elapsed):
            target = initial.copy()
            target[args.joint] += amplitude * math.sin(omega * elapsed)
            velocity = [0.0] * 7
            velocity[args.joint] = amplitude * omega * math.cos(omega * elapsed)
            arm.servo_joint_position(target, velocity, max_vel=vmax, impedance=args.impedance)

        arm.run_servo(update, args.duration, max_lag=args.max_lag)


if __name__ == "__main__":
    main()
