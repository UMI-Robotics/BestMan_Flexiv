"""Shared CLI and lifecycle for runnable Rizon 4 examples."""

import argparse
import logging
import math
from contextlib import contextmanager

from Robotics_API import Bestman_Real_Flexiv


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be positive and finite")
    return value


def parser(description):
    result = argparse.ArgumentParser(description=description)
    result.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    result.add_argument(
        "--frequency", type=positive, default=None, help="NRT Hz, 1-100; default from config (20)"
    )
    result.add_argument(
        "--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable"
    )
    result.add_argument("--config", help="Rizon 4 JSON configuration; default: built-in Rizon 4 profile")
    return result


@contextmanager
def robot_session(args, *, enable=True):
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    robot = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    try:
        if enable:
            robot.initialize_robot()
        yield robot
    finally:
        if enable:
            robot.stop()


def require_completed(result):
    if not result:
        raise TimeoutError("Motion did not converge; robot was stopped")
