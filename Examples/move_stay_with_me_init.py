"""Single-file stay-with-me INIT, using BestMan JOINT POSITION control.

Preview degrees/radians without connecting:
  python -m Examples.move_stay_with_me_init --arm Left
Read current joints without enabling or moving:
  python -m Examples.move_stay_with_me_init --arm Left --left-sn Rizon4-123456 --read-state
Move the left Rizon4 to the embedded demo INIT joint posture:
  python -m Examples.move_stay_with_me_init --arm Left --left-sn Rizon4-123456 --run
Move the right Rizon4R to its own embedded INIT joint posture:
  python -m Examples.move_stay_with_me_init --arm Right --right-sn Rizon4R-654321 --run

All loops, sends, feedback and stopping are in this file. Only BestMan is used.
Edit ROBOTS[side]['init_joint_deg'] to change the joint target; SDK uses radians.
No TCP target, tool-frame conversion or IK is used for motion. Current tool
settings are not changed. This is a limited-speed direct joint move, not a
collision-checked entry path. Add --enable if the controller is not yet enabled.
The Left snapshot is for Rizon4 and the Right snapshot is for Rizon4R.
Use the matching serial prefix for each arm; --arm both moves Left then Right.
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
from Robotics_API.config import model_from_serial, positive, vector

# Verified 2026-11-20 against DJ-Sim/configs/robot_models.json: home_degrees.
# This is inward45_down30_lower_link2_link3 INIT, NOT horizontal-reference HOME.
# Native joint order J1..J7; no signs or joint-coordinate offsets are added.
ROBOTS = {
    "Left": {
        "sn": "",
        "source_model": "Rizon4",
        "init_joint_deg": [
            23.839993772894275,
            103.11853942611782,
            -63.325904744874336,
            116.76360790640103,
            82.66197167425484,
            -22.298794228275693,
            55.77233573427813,
        ],
    },
    "Right": {
        "sn": "",
        "source_model": "Rizon4R",
        "init_joint_deg": [
            23.84085495748669,
            103.11843814564148,
            -63.3263895424075,
            116.7636063860727,
            82.66143189909643,
            -22.299072098980506,
            55.771868496297046,
        ],
    },
}
FREQUENCY_HZ = 30.0
MAX_JOINT_VEL_DEG_S = 5.0
MAX_JOINT_ACC_DEG_S2 = 10.0
TIMEOUT_S = 90.0
JOINT_TOLERANCE_DEG = 0.5
JOINT_SPEED_TOLERANCE_DEG_S = 0.5
SETTLE_TIME_S = 0.3


def joint_target(side):
    return [math.radians(x) for x in vector(ROBOTS[side]["init_joint_deg"], 7, "INIT degrees")]


def ready(arm, side, require_operational=True):
    status = arm.get_status()
    if not status["connected"]:
        raise RuntimeError(side + ": disconnected")
    if status["fault"]:
        raise RuntimeError(side + ": controller fault")
    if require_operational and not status["operational"]:
        raise RuntimeError(side + ": not operational; enable at controller or add --enable")
    return status


def check_target(arm, target, max_vel):
    """Check native limits before enabling or sending; never clamp a target."""
    target = vector(target, 7, "joint target")
    info = arm.get_robot_info()
    low = vector(info.q_min, 7, "q_min")
    high = vector(info.q_max, 7, "q_max")
    native_vel = vector(info.dq_max, 7, "dq_max", lower=0, strict=True)
    for i, (q, lo, hi, vel) in enumerate(zip(target, low, high, native_vel), 1):
        if not lo < hi or not lo <= q <= hi:
            raise ValueError(
                f"J{i} target {math.degrees(q):.3f} deg outside controller limits "
                f"[{math.degrees(lo):.3f}, {math.degrees(hi):.3f}]"
            )
        if max_vel > vel:
            raise ValueError(f"J{i} requested speed exceeds controller limit")
    return target


def read_diagnostics(arm, side, target):
    state, info, status = arm.update_robot_states(), arm.get_robot_info(), arm.get_status()
    q = vector(state.q, 7, "measured joints")
    dq = vector(state.dq, 7, "measured joint speeds")
    rows = []
    for i, (actual, goal, lo, hi) in enumerate(zip(q, target, info.q_min, info.q_max), 1):
        rows.append(
            dict(
                joint=i,
                actual_deg=math.degrees(actual),
                target_deg=math.degrees(goal),
                error_deg=math.degrees(actual - goal),
                min_deg=math.degrees(lo),
                max_deg=math.degrees(hi),
                nearest_margin_deg=math.degrees(min(actual - lo, hi - actual)),
            )
        )
    return dict(
        arm=side,
        robot_sn=arm.get_arm_id(),
        control_space="joint",
        target_joint_rad=list(target),
        target_joint_deg=[math.degrees(v) for v in target],
        actual_joint_rad=q,
        actual_joint_deg=[math.degrees(v) for v in q],
        actual_joint_velocity_deg_s=[math.degrees(v) for v in dq],
        max_joint_error_deg=max(abs(row["error_deg"]) for row in rows),
        joints=rows,
        near_joint_limits=[r for r in rows if r["nearest_margin_deg"] < 2.0],
        controller_status={
            k: str(v) if k in ("mode", "operational_status") else v for k, v in status.items()
        },
    )


def move_to_init(arm, side, target, max_vel, max_acc, frequency, timeout):
    """Single-file joint send/feedback loop; BestMan wraps one send per call."""
    target = check_target(arm, target, max_vel)
    status = ready(arm, side)
    if status["busy"] or status["mode"] != arm.mode.IDLE:
        raise RuntimeError(side + ": controller must be IDLE before starting INIT")
    period = 1.0 / frequency
    started = next_tick = time.monotonic()
    deadline = started + timeout
    settled_since, timeout_failure = None, None
    first = True
    try:
        while True:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(max(0.0, min(next_tick, deadline) - now))
            if time.monotonic() >= deadline:
                raise TimeoutError(side + ": joint INIT timeout")
            status = ready(arm, side)
            if not first and status["mode"] != arm.mode.NRT_JOINT_POSITION:
                raise RuntimeError(side + ": joint-position mode changed during motion")
            if time.monotonic() >= deadline:
                raise TimeoutError(side + ": joint INIT timeout")

            # Actual JOINT command, radians and rad/s. No Cartesian/force call.
            arm.servo_joint_position(
                target, velocities=[0.0] * 7, max_vel=[max_vel] * 7, max_acc=[max_acc] * 7, impedance=False
            )
            first = False
            state = arm.update_robot_states()
            q = vector(state.q, 7, "measured joints")
            dq = vector(state.dq, 7, "measured joint speeds")
            # Do not wrap angles: these joints have finite native limits.
            max_error = max(abs(a - b) for a, b in zip(q, target))
            max_speed = max(abs(v) for v in dq)
            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError(side + ": joint INIT timeout")
            if max_error <= math.radians(JOINT_TOLERANCE_DEG) and max_speed <= math.radians(
                JOINT_SPEED_TOLERANCE_DEG_S
            ):
                settled_since = now if settled_since is None else settled_since
                if now - settled_since >= SETTLE_TIME_S:
                    status = ready(arm, side)
                    if status["mode"] != arm.mode.NRT_JOINT_POSITION:
                        raise RuntimeError(side + ": joint-position mode changed during motion")
                    if time.monotonic() >= deadline:
                        raise TimeoutError(side + ": joint INIT timeout")
                    return dict(
                        status="INIT_REACHED",
                        arm=side,
                        control_space="joint",
                        actual_joint_rad=q,
                        actual_joint_deg=[math.degrees(v) for v in q],
                        max_joint_error_deg=math.degrees(max_error),
                        elapsed_s=time.monotonic() - started,
                    )
            else:
                settled_since = None
            next_tick += period
            if next_tick <= now:
                next_tick += (math.floor((now - next_tick) / period) + 1) * period
    except TimeoutError as exc:
        timeout_failure = exc
    finally:
        arm.stop()  # Success, interruption and errors all Stop; failure propagates.
    if timeout_failure is not None:
        try:
            print(json.dumps(dict(status="INIT_TIMEOUT", **read_diagnostics(arm, side, target))))
        except Exception as exc:
            print("Timeout diagnostic unavailable: " + str(exc), file=sys.stderr)
        raise timeout_failure


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--arm", choices=("Left", "Right", "both"), default="Left")
    parser.add_argument("--left-sn", default=ROBOTS["Left"]["sn"], help="Actual left Rizon4 serial")
    parser.add_argument("--right-sn", default=ROBOTS["Right"]["sn"], help="Actual right Rizon4R serial")
    parser.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist")
    parser.add_argument("--config", help="Optional BestMan JSON configuration")
    parser.add_argument("--run", action="store_true", help="Move using joint position control")
    parser.add_argument(
        "--read-state", action="store_true", help="Read joints only; no mode change/enable/stop"
    )
    parser.add_argument("--enable", action="store_true", help="Enable a fault-free controller if needed")
    parser.add_argument("--frequency", "--hz", type=float, default=FREQUENCY_HZ)
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S)
    parser.add_argument("--max-joint-vel-deg-s", type=float, default=MAX_JOINT_VEL_DEG_S)
    parser.add_argument("--max-joint-acc-deg-s2", type=float, default=MAX_JOINT_ACC_DEG_S2)
    args = parser.parse_args(argv)
    sides = ("Left", "Right") if args.arm == "both" else (args.arm,)
    old_sigterm = None
    try:
        if args.read_state and (args.run or args.enable):
            raise ValueError("--read-state cannot be combined with --run or --enable")
        positive(args.timeout, "timeout")
        max_vel = math.radians(positive(args.max_joint_vel_deg_s, "joint speed"))
        max_acc = math.radians(positive(args.max_joint_acc_deg_s2, "joint acceleration"))
        if not math.isfinite(args.frequency) or not 1 <= args.frequency <= 100:
            raise ValueError("--frequency must be in [1,100]")
        targets = {side: joint_target(side) for side in sides}
        print(
            json.dumps(
                dict(
                    status="INIT_TARGETS",
                    control_space="joint",
                    joint_order="J1..J7",
                    targets_deg={s: ROBOTS[s]["init_joint_deg"] for s in sides},
                    targets_rad=targets,
                    source_models={s: ROBOTS[s]["source_model"] for s in sides},
                    frequency_hz=args.frequency,
                    max_joint_vel_deg_s=args.max_joint_vel_deg_s,
                    max_joint_acc_deg_s2=args.max_joint_acc_deg_s2,
                ),
                indent=2,
            )
        )
        if not args.run and not args.read_state:
            return 0
        serials = {s: args.left_sn if s == "Left" else args.right_sn for s in sides}
        for side, sn in serials.items():
            if model_from_serial(sn) != ROBOTS[side]["source_model"]:
                raise ValueError(f"{side}: this INIT target requires {ROBOTS[side]['source_model']}")
            serials[side] = sn.replace(" ", "")  # Use the same local lock for SDK-style spaced serials.
        if len(set(serials.values())) != len(sides):
            raise ValueError("Arm serials must be distinct")

        def interrupted(signum, frame):
            raise KeyboardInterrupt

        old_sigterm = signal.signal(signal.SIGTERM, interrupted)
        with ExitStack() as stack:
            arms = {}
            for side in sides:
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
                if arm.get_arm_id().replace(" ", "") != serials[side]:
                    raise ValueError(side + ": connected controller serial mismatch")
                if args.read_state:
                    print(
                        json.dumps(
                            dict(
                                status="INIT_DIAGNOSTIC",
                                motion_commands_sent=False,
                                **read_diagnostics(arm, side, targets[side]),
                            )
                        )
                    )
                    continue
                info = arm.get_robot_info()
                if info.model_name.replace(" ", "").lower() != ROBOTS[side]["source_model"].lower():
                    raise ValueError(side + ": controller model differs from joint preset")
                check_target(arm, targets[side], max_vel)
                status = ready(arm, side, require_operational=not args.enable)
                if status["busy"] or status["mode"] != arm.mode.IDLE:
                    raise RuntimeError(side + ": controller must be IDLE before starting INIT")
                print(side, "current J1..J7 (deg):", [math.degrees(v) for v in arm.update_robot_states().q])
            if args.read_state:
                return 0
            for side in sides:
                arm = arms[side]
                status = ready(arm, side, require_operational=not args.enable)
                if args.enable and not status["operational"]:
                    arm.initialize_robot()
                print(
                    json.dumps(
                        move_to_init(arm, side, targets[side], max_vel, max_acc, args.frequency, args.timeout)
                    )
                )
        return 0
    except KeyboardInterrupt:
        print("INIT interrupted.")
        return 130
    except Exception as exc:
        print("INIT failed: " + str(exc), file=sys.stderr)
        return 2
    finally:
        if old_sigterm is not None:
            signal.signal(signal.SIGTERM, old_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
