"""Move Rizon 4 arms to the embedded stay-with-me INIT TCP targets (Linux/macOS).

Preview without connecting:
  python -m Examples.move_stay_with_me_init
Move one arm, using an already enabled controller:
  python -m Examples.move_stay_with_me_init --arm Left --left-sn Rizon4-123456 --run
Move two Rizon 4 arms sequentially, Left then Right:
  python -m Examples.move_stay_with_me_init --left-sn Rizon4-123456 --right-sn Rizon4-654321 --run

Add --enable to enable a fault-free controller if needed. The active tool must
already describe the modified tip. Positions are metres, quaternions are wxyz.
The target numbers use each controller's native WORLD frame as the corresponding
arm's base frame. No frame conversion or flange/tool offset is applied here.
This is a speed-limited direct move, with no obstacle or dual-arm path planning.
"""

import argparse
import fcntl
import hashlib
import json
import math
import signal
import sys
import tempfile
import time
from contextlib import ExitStack
from pathlib import Path

from Robotics_API import Bestman_Real_Flexiv
from Robotics_API.config import pose_vector, positive, vector

# Snapshot supplied on 2026-11-20: [x, y, z, qw, qx, qy, qz]. Update these values
# explicitly when the demo INIT changes; this file does not import DJ-demo data.
# Source control-table SHA256: 93b65c2cfe201724340e66e7196f7761db78d56022c1d571cc2bc1c1767ede8b
# Left/Right select target presets, not robot models. Both connections require
# actual Rizon4 serials; the source script's Rizon4R model is not supported here.
ROBOTS = {
    "Left": {
        "sn": "",
        "init_tcp": [
            0.8229666886592979,
            0.2500000000005482,
            -0.10500000000006526,
            0.4619397662465448,
            0.331413574054252,
            0.8001031451884719,
            -0.1913417161838717,
        ],
    },
    "Right": {
        "sn": "",
        "init_tcp": [
            0.8229666886591721,
            -0.25000000000070616,
            -0.10500000000020626,
            0.4619397662467163,
            -0.3314135740536158,
            0.8001031451885913,
            0.19134171618406087,
        ],
    },
}
LIMITS = dict(max_linear_vel=0.03, max_angular_vel=0.2, max_linear_acc=0.05, max_angular_acc=0.3)
FREQUENCY_HZ = 30.0
TIMEOUT_S = 90.0  # Per arm.
POSITION_TOLERANCE_M = 0.001
ANGLE_TOLERANCE_RAD = math.radians(0.5)
SETTLE_TIME_S = 0.3


def pose_error(actual, target):
    """Measure translation/orientation error without modifying the sent wxyz values."""
    actual, target = pose_vector(actual), pose_vector(target)
    dot = abs(sum(a * b for a, b in zip(actual[3:], target[3:])))
    dot /= math.hypot(*actual[3:]) * math.hypot(*target[3:])
    return math.dist(actual[:3], target[:3]), 2 * math.acos(min(1.0, dot))


def active_tool(arm):
    name, params = arm.get_tool()
    if not name or name == "Flange":
        raise ValueError("Select the already configured modified-tip Tool before running")
    # Check the existing tool; never add tcp_location to the target a second time.
    return (
        name,
        tuple(pose_vector(params.tcp_location)),
        vector([params.mass], 1, "tool mass", lower=0)[0],
        tuple(vector(params.CoM, 3, "tool CoM")),
        tuple(vector(params.inertia, 6, "tool inertia")),
    )


def ready(arm, side, require_operational=True):
    status = arm.get_status()
    if not status["connected"]:
        raise RuntimeError(side + ": disconnected")
    if status["fault"]:
        raise RuntimeError(side + ": controller fault")
    if require_operational and not status["operational"]:
        raise RuntimeError(side + ": not operational; enable at controller or add --enable")
    return status


def move_to_init(arm, side, target, limits, frequency, timeout, tool):
    """All dispatch, feedback, timing and stopping are visible in this file."""
    target = pose_vector(target)
    period = 1.0 / frequency
    started = next_tick = next_tool_check = time.monotonic()
    deadline = started + timeout
    settled_since = None
    status = ready(arm, side)
    if status["busy"] or status["mode"] != arm.mode.IDLE:
        raise RuntimeError(side + ": controller must be IDLE before starting INIT")
    if active_tool(arm) != tool:
        raise RuntimeError(side + ": active tool changed")
    try:
        # Pure position control: WORLD frame, all force axes off, zero wrench.
        arm.configure_force_control([False] * 6, frame="WORLD")
        while True:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(max(0.0, min(next_tick, deadline) - now))
            if time.monotonic() >= deadline:
                raise TimeoutError(side + ": INIT timeout")
            status = ready(arm, side)
            if status["mode"] != arm.mode.NRT_CARTESIAN_MOTION_FORCE:
                raise RuntimeError(side + ": Cartesian mode changed during motion")
            if time.monotonic() >= next_tool_check:
                if active_tool(arm) != tool:
                    raise RuntimeError(side + ": active tool changed")
                next_tool_check = time.monotonic() + 0.5
            # Tool/status reads can block; do not dispatch after the overall timeout.
            if time.monotonic() >= deadline:
                raise TimeoutError(side + ": INIT timeout")

            arm.servo_cartesian_pose(target, velocity=[0.0] * 6, **limits)
            state = arm.update_robot_states()
            actual = pose_vector(state.tcp_pose)
            velocity = vector(state.tcp_vel, 6, "TCP velocity")
            distance, angle = pose_error(actual, target)
            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError(side + ": INIT timeout")
            still = math.hypot(*velocity[:3]) <= 0.002 and math.hypot(*velocity[3:]) <= 0.01
            if distance <= POSITION_TOLERANCE_M and angle <= ANGLE_TOLERANCE_RAD and still:
                settled_since = now if settled_since is None else settled_since
                if now - settled_since >= SETTLE_TIME_S:
                    status = ready(arm, side)
                    if status["mode"] != arm.mode.NRT_CARTESIAN_MOTION_FORCE:
                        raise RuntimeError(side + ": Cartesian mode changed during motion")
                    if active_tool(arm) != tool:
                        raise RuntimeError(side + ": active tool changed")
                    if time.monotonic() >= deadline:
                        raise TimeoutError(side + ": INIT timeout")
                    return dict(
                        status="INIT_REACHED",
                        arm=side,
                        frame="WORLD",
                        quaternion_order="wxyz",
                        actual_tcp=actual,
                        error_mm=distance * 1000,
                        error_deg=math.degrees(angle),
                        elapsed_s=time.monotonic() - started,
                    )
            else:
                settled_since = None
            next_tick += period
            if next_tick <= now:
                next_tick += (math.floor((now - next_tick) / period) + 1) * period
    finally:
        # Stop on completion, timeout, fault, Ctrl+C or SIGTERM. A failed Stop
        # propagates, so the caller cannot report INIT_REACHED after a stop failure.
        arm.stop()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--arm", choices=("Left", "Right", "both"), default="both")
    parser.add_argument("--left-sn", default=ROBOTS["Left"]["sn"], help="Actual left Rizon4 serial")
    parser.add_argument("--right-sn", default=ROBOTS["Right"]["sn"], help="Actual right Rizon4 serial")
    parser.add_argument(
        "--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable"
    )
    parser.add_argument("--config", help="Optional BestMan JSON configuration")
    parser.add_argument(
        "--run", action="store_true", help="Move; otherwise preview targets without connecting"
    )
    parser.add_argument("--enable", action="store_true", help="Enable a fault-free controller if needed")
    parser.add_argument(
        "--frequency", "--hz", type=float, default=FREQUENCY_HZ, help="NRT Hz, 1-100; default: 30"
    )
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S, help="Per-arm timeout (s)")
    for key, value in LIMITS.items():
        parser.add_argument("--" + key.replace("_", "-"), type=float, default=value)
    args = parser.parse_args(argv)
    sides = ("Left", "Right") if args.arm == "both" else (args.arm,)
    old_sigterm = None
    try:
        limits = {key: positive(getattr(args, key), key) for key in LIMITS}
        positive(args.timeout, "timeout")
        if not math.isfinite(args.frequency) or not 1 <= args.frequency <= 100:
            raise ValueError("--frequency must be in [1,100]")
        targets = {side: pose_vector(ROBOTS[side]["init_tcp"]) for side in sides}
        print(
            json.dumps(
                dict(
                    status="INIT_TARGETS",
                    frame="WORLD",
                    units="m",
                    quaternion_order="wxyz",
                    targets=targets,
                    frequency_hz=args.frequency,
                    limits=limits,
                ),
                indent=2,
            )
        )
        if not args.run:
            return 0
        serials = {side: args.left_sn if side == "Left" else args.right_sn for side in sides}
        for side, serial in serials.items():
            if not serial.startswith("Rizon4-") or not serial.removeprefix("Rizon4-").strip():
                raise ValueError(side + ": supply an actual Rizon4-... serial; Rizon4R is not supported")
        if len(set(serials.values())) != len(sides):
            raise ValueError("Arm serial numbers must be distinct")

        def interrupted(signum, frame):
            raise KeyboardInterrupt

        old_sigterm = signal.signal(signal.SIGTERM, interrupted)
        with ExitStack() as stack:
            arms, tools = {}, {}
            for side in sides:
                # Compatible with the DJ scripts' local serial-number lock. Other
                # processes must honor this lock too; controller IDLE is also checked.
                key = hashlib.sha256(serials[side].encode()).hexdigest()[:24]
                lock_path = Path(tempfile.gettempdir()) / ("dj-real-" + key + ".lock")
                handle = stack.enter_context(lock_path.open("a+"))
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError(side + ": another local process owns this arm") from exc
                arm = Bestman_Real_Flexiv(
                    serials[side],
                    args.frequency,
                    config=args.config,
                    network_interface_whitelist=args.interface,
                )
                arms[side] = arm
                if arm.get_arm_id() != serials[side]:
                    raise ValueError(side + ": connected controller serial mismatch")
                status = ready(arm, side, require_operational=not args.enable)
                if status["busy"] or status["mode"] != arm.mode.IDLE:
                    raise RuntimeError(side + ": controller must be IDLE before starting INIT")
                tools[side] = active_tool(arm)
                print(side, "Tool:", tools[side][0], "current TCP (wxyz):", arm.get_current_eef_pose())
            # Check both arms before any motion, then execute Left -> Right.
            for side in sides:
                arm = arms[side]
                status = ready(arm, side, require_operational=not args.enable)
                if args.enable and not status["operational"]:
                    arm.initialize_robot()
                result = move_to_init(
                    arm, side, targets[side], limits, args.frequency, args.timeout, tools[side]
                )
                print(json.dumps(result))
        return 0
    except KeyboardInterrupt:
        print("INIT interrupted.")
        return 130
    except Exception as exc:
        print("INIT failed:", str(exc), file=sys.stderr)
        return 2
    finally:
        if old_sigterm is not None:
            signal.signal(signal.SIGTERM, old_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
