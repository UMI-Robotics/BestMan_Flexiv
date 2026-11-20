"""Hand-guide in a Linux/macOS terminal; record WORLD TCP [x,y,z,qw,qx,qy,qz] (metres, wxyz)."""

import argparse
import json
import os
import select
import sys
import time
from pathlib import Path

from Robotics_API import Bestman_Real_Flexiv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("robot_sn", help="Rizon 4 serial number, e.g. Rizon4-123456")
    p.add_argument("--frequency", type=float, default=None, help="NRT Hz, 1-100; default: 20")
    p.add_argument("--interface", action="append", default=[], help="Local NIC IPv4 whitelist; repeatable")
    p.add_argument("--config", help="Optional JSON configuration; built-in Rizon 4 defaults otherwise")
    p.add_argument(
        "--primitive",
        choices=("FloatingCartesian", "FloatingJoint"),
        default="FloatingCartesian",
        help="Licensed RDK free-drive primitive; default: FloatingCartesian",
    )
    p.add_argument("--output", type=Path, default=Path("taught_poses.json"))
    args = p.parse_args()
    if args.output.exists():
        p.error("output already exists; choose a new file")
    if sys.platform == "win32":
        p.error("This interactive example requires a Linux/macOS terminal")
    input_fd = sys.stdin.fileno()

    # Connect by serial number; construction does not enable or move the robot.
    arm = Bestman_Real_Flexiv(
        args.robot_sn, args.frequency, network_interface_whitelist=args.interface, config=args.config
    )
    poses = []
    try:
        arm.initialize_robot()
        arm.stop()
        print("Keep the robot in AUTO (REMOTE). Release the enabling button once before starting.")
        print("Then press and HOLD the enabling button on the Motion Bar while guiding the arm.")
        print("Release the button to stop, then enter r to record WORLD TCP [x,y,z,qw,qx,qy,qz].")
        print("s: show current controller status; q: stop and save. Press Enter after each command.")
        # The primitive itself ignores the button. Poll it at 50 Hz, including while
        # waiting for keyboard input. This Python gate does not replace robot safety functions.
        armed = floating = confirmed = False
        requested_at = 0.0
        previous_poll = time.monotonic()
        previous_display = None
        pending_input = ""
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            now = time.monotonic()
            if floating and now - previous_poll > 0.25:
                raise RuntimeError("Button monitoring was delayed; stopping free drive")
            previous_poll = now
            if readable:
                chunk = os.read(input_fd, 4096)
                if not chunk:  # EOF: stop and preserve recorded poses.
                    break
                pending_input += chunk.decode("utf-8", errors="replace")
            command = ""
            if "\n" in pending_input:
                command, pending_input = pending_input.split("\n", 1)
                command = command.strip().lower()
            if command == "q":
                break

            status = arm.get_status()
            if not status["connected"] or not status["operational"] or status["fault"]:
                raise RuntimeError(f"Robot is not ready for teaching: {status}")
            expected_mode = arm.mode.NRT_PRIMITIVE_EXECUTION if floating else arm.mode.IDLE
            if status["mode"] != expected_mode:
                raise RuntimeError(f"Control mode changed unexpectedly: {status['mode']}")

            if not status["enabling_button_pressed"]:
                armed = True
                if floating:
                    arm.stop()
                    floating = confirmed = False
                    status = arm.get_status()
            elif armed and not floating and command != "r":
                arm.switch_mode(arm.mode.NRT_PRIMITIVE_EXECUTION)
                # A button release during SwitchMode must not start free drive.
                status = arm.get_status()
                if time.monotonic() - previous_poll > 0.25:
                    raise RuntimeError("Button monitoring was delayed during mode switch")
                if status["enabling_button_pressed"]:
                    # Official RDK 1.x API. Do not use the Elements internal PLAN-FreeDriveAuto.
                    arm.execute_primitive(args.primitive, {}, wait=False, block_until_started=False)
                    floating, armed = True, False
                    requested_at = time.monotonic()
                else:
                    arm.stop()
                    status = arm.get_status()

            primitive = arm.get_primitive_states() if floating else {}
            if floating:
                running = (
                    primitive.get("primitiveName") == args.primitive
                    and primitive.get("terminated") == 0
                    and status["busy"]
                )
                if running:
                    confirmed = True
                elif confirmed:
                    raise RuntimeError(f"Free-drive primitive stopped unexpectedly: {primitive}")
                elif time.monotonic() - requested_at > 2.0:
                    raise RuntimeError(f"Free-drive primitive did not start: {primitive}; check its license")

            phase = "ACTIVE" if confirmed else "STARTING" if floating else "HOLD"
            display = (
                phase,
                status["mode"],
                status["enabling_button_pressed"],
                primitive.get("primitiveName"),
            )
            if display != previous_display or command == "s":
                button = "PRESSED" if status["enabling_button_pressed"] else "RELEASED"
                print(
                    f"{phase} mode={status['mode']} enabling_button={button} "
                    f"primitive={primitive.get('primitiveName', '-')} fault={status['fault']}"
                )
                if not armed and not floating:
                    print("Release the enabling button once, then press and hold it to guide.")
                previous_display = display
            if command == "r":
                status = arm.get_status()
                if not status["connected"] or not status["operational"] or status["fault"]:
                    raise RuntimeError(f"Robot became unavailable before recording: {status}")
                if status["enabling_button_pressed"] or floating:
                    print("Release the enabling button before recording.")
                elif status["mode"] != arm.mode.IDLE:
                    raise RuntimeError("Control mode changed before recording")
                else:
                    pose = arm.get_current_eef_pose()
                    poses.append(pose)  # Store wxyz unchanged; replay accepts the same lists.
                    print(f"Recorded {len(poses)}: {pose} (wxyz)")
            elif command not in ("", "s"):
                print("Use r to record, s for status, or q to stop and save.")
    except KeyboardInterrupt:
        print("Interrupted; stopping and saving recorded poses.")
    finally:
        # Preserve records on q, EOF, Ctrl+C, controller errors and even Stop() errors.
        try:
            arm.stop()
        finally:
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(poses, stream, indent=2)
            print(f"Saved {args.output.resolve()}; replay with move_eef_follow_trajectory.py")


if __name__ == "__main__":
    main()
