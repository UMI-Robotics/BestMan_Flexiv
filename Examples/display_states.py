"""Read joint states and WORLD TCP [x,y,z,qw,qx,qy,qz] (wxyz), without enabling motion."""

import argparse
import json
import math
import time

from Robotics_API import Bestman_Real_Flexiv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument("--config", help="Optional JSON configuration; built-in Rizon 4 defaults otherwise")
    p.add_argument("--duration", type=float, default=5.0, help="Read duration in seconds")
    args = p.parse_args()

    if not math.isfinite(args.duration) or args.duration <= 0:
        p.error("--duration must be positive and finite")

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        states = arm.update_robot_states()
        # q: radians; dq: rad/s; tau: Nm; tcp_pose: metres and wxyz.
        print(
            json.dumps(
                {
                    name: list(getattr(states, name))
                    for name in (
                        "q",
                        "dq",
                        "tau",
                        "tau_ext",
                        "tcp_pose",
                        "tcp_vel",
                        "ext_wrench_in_world",
                        "ext_wrench_in_tcp",
                        "ft_sensor_raw",
                    )
                }
            )
        )
        time.sleep(1.0 / arm.frequency)


if __name__ == "__main__":
    main()
