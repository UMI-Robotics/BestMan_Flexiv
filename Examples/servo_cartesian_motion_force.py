"""NRT Cartesian servo: every pose is [x,y,z,qw,qx,qy,qz], WORLD metres and wxyz."""

import math

import numpy as np

from Examples._common import parser, positive, robot_session
from Robotics_API.config import vector


def main():
    p = parser(__doc__)
    p.add_argument("--duration", type=positive, default=5.0)
    p.add_argument("--axis", choices=("x", "y", "z"), default="x")
    p.add_argument(
        "--amplitude", type=float, default=0.0, help="Translation sine amplitude (m); 0 holds position"
    )
    p.add_argument("--sine-frequency", type=positive, default=0.25)
    p.add_argument(
        "--force-z", type=float, default=0.0, help="Sensed WORLD Fz in N; positive presses toward -Z"
    )
    p.add_argument("--max-displacement", type=positive, default=0.03, help="Measured TCP travel limit (m)")
    p.add_argument("--abort-force", type=positive, default=12.0, help="Measured force norm limit (N)")
    p.add_argument("--abort-moment", type=positive, default=3.0, help="Measured moment norm limit (Nm)")
    p.add_argument("--max-lag", type=positive, default=0.1, help="Observed loop/callback lag limit (s)")
    args = p.parse_args()
    amplitude, force_z = vector([args.amplitude, args.force_z], 2, "amplitude/force-z")
    if abs(amplitude) >= args.max_displacement or abs(force_z) >= args.abort_force:
        p.error("Targets must stay below the measured displacement/force abort limits")
    if force_z and args.axis == "z" and amplitude:
        p.error("Choose x/y motion when Z is force-controlled")
    omega = 2.0 * math.pi * args.sine_frequency
    axis = {"x": 0, "y": 1, "z": 2}[args.axis]
    with robot_session(args) as arm:
        if (
            abs(amplitude) * omega > arm.config.max_linear_vel
            or abs(amplitude) * omega**2 > arm.config.max_linear_acc
        ):
            raise ValueError("Sine peak velocity/acceleration exceeds the configured motion limits")
        # Configure once. No Home, contact search, or sensor zeroing is performed.
        arm.configure_force_control([False, False, bool(force_z), False, False, False])
        initial = arm.get_current_eef_pose()  # [x,y,z,qw,qx,qy,qz], unchanged SDK order.

        def update(elapsed):
            states = arm.update_robot_states()
            measured = np.asarray(states.ext_wrench_in_world)
            distance = np.linalg.norm(np.asarray(states.tcp_pose[:3]) - initial[:3])
            if not np.isfinite(measured).all() or not np.isfinite(distance):
                raise RuntimeError("Invalid force/pose measurement")
            if (
                distance > args.max_displacement
                or np.linalg.norm(measured[:3]) > args.abort_force
                or np.linalg.norm(measured[3:]) > args.abort_moment
            ):
                raise RuntimeError("Measured displacement or wrench exceeded the example limit")
            target = initial.copy()  # Preserve wxyz; only change one WORLD position component.
            target[axis] += amplitude * math.sin(omega * elapsed)
            velocity = [0.0] * 6
            velocity[axis] = amplitude * omega * math.cos(omega * elapsed)
            if force_z:
                arm.servo_cartesian_motion_force(target, [0.0, 0.0, force_z, 0.0, 0.0, 0.0], velocity)
            else:
                arm.servo_cartesian_pose(target, velocity)

        arm.run_servo(update, args.duration, max_lag=args.max_lag)


if __name__ == "__main__":
    main()
