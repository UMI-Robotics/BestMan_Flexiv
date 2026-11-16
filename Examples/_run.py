"""Implementation shared by the small, directly runnable example entry points."""

import json
import time
from pathlib import Path

import numpy as np

from Examples._common import parser, positive, require_completed, robot_session
from Robotics_API import Pose


def main(name):
    p = parser(name.replace("_", " "))
    if name in ("display_states", "force_control", "cartesian_impedance", "joint_impedance"):
        p.add_argument("--duration", type=positive, default=5.0, help="Finite duration in seconds")
    if name == "move_arm_to_joint_values":
        p.add_argument(
            "--joints", type=float, nargs=7, required=True, help="Seven joint positions in radians"
        )
    if name in ("move_eef_to_pose", "graspany"):
        p.add_argument("--pose", type=float, nargs=7, required=True, help="WORLD x y z qx qy qz qw (m, xyzw)")
    if name in ("move_arm_follow_joint_values", "move_eef_follow_trajectory"):
        p.add_argument(
            "--trajectory", type=Path, required=True, help="JSON list of 7-element waypoints (poses use xyzw)"
        )
    if name in ("open_gripper", "close_gripper", "open_gripper_width", "graspany"):
        p.add_argument("--port", required=True, help="Robotiq USB serial device, e.g. /dev/ttyUSB0")
        p.add_argument("--speed", type=int, default=64)
        p.add_argument("--gripper-force", type=int, default=64, help="Robotiq 0-255 command, not Newtons")
    if name == "open_gripper_width":
        p.add_argument(
            "--position", type=int, default=100, help="Raw Robotiq 0=open to 255=closed, not width in mm"
        )
    if name in ("force_control", "cartesian_impedance", "joint_impedance"):
        p.add_argument(
            "--max-displacement", type=positive, default=0.03, help="Abort beyond this TCP distance (m)"
        )
        p.add_argument("--abort-force", type=positive, default=12.0, help="Measured force norm limit (N)")
        p.add_argument("--abort-moment", type=positive, default=3.0, help="Measured moment norm limit (Nm)")
    if name == "force_control":
        p.add_argument("--frame", choices=("WORLD", "TCP"), default="WORLD")
        p.add_argument("--axes", type=int, choices=(0, 1), nargs=6, default=[0, 0, 1, 0, 0, 0])
        p.add_argument(
            "--wrench",
            type=float,
            nargs=6,
            default=[0.0, 0.0, 3.0, 0.0, 0.0, 0.0],
            help="Desired sensed wrench; WORLD +Fz presses toward -Z",
        )
        p.add_argument(
            "--passive",
            action="store_true",
            help="Open-loop feedforward; active velocity guard does not apply",
        )
        p.add_argument(
            "--disable-contact-regulation",
            action="store_true",
            help="Required for moment control; measured abort limits still apply",
        )
    if name in ("cartesian_impedance", "joint_impedance"):
        p.add_argument(
            "--stiffness-scale", type=positive, default=0.3, help="Fraction of controller nominal, <=1"
        )
    if name == "teach_by_demonstration":
        p.add_argument(
            "--plan", default="PLAN-FreeDriveAuto", help="Free-drive plan installed on your controller"
        )
        p.add_argument("--output", type=Path, default=Path("taught_poses.json"))
    args = p.parse_args()

    # Validate user data before enabling or changing modes.
    if hasattr(args, "stiffness_scale") and args.stiffness_scale > 1:
        p.error("--stiffness-scale must be <=1")
    if hasattr(args, "pose"):
        target = Pose(args.pose[:3], args.pose[3:])
    if hasattr(args, "trajectory"):
        waypoints = json.loads(args.trajectory.read_text())
        if not isinstance(waypoints, list) or not waypoints:
            p.error("trajectory must be a nonempty JSON list")
        from Robotics_API.config import vector

        waypoints = [vector(row, 7, "waypoint") for row in waypoints]
        if name == "move_eef_follow_trajectory":
            waypoints = [Pose(row[:3], row[3:]) for row in waypoints]
    if hasattr(args, "gripper_force"):
        if any(
            not 0 <= value <= 255 for value in [args.speed, args.gripper_force, getattr(args, "position", 0)]
        ):
            p.error("gripper commands must be in [0,255]")
    if name == "force_control":
        if not np.isfinite(args.wrench).all() or any(
            w and not axis for w, axis in zip(args.wrench, args.axes)
        ):
            p.error("wrench must be finite and nonzero only on enabled axes")
        if (
            np.linalg.norm(args.wrench[:3]) >= args.abort_force
            or np.linalg.norm(args.wrench[3:]) >= args.abort_moment
        ):
            p.error("target wrench must be below measured abort limits")
        if any(args.axes[3:]) and not args.disable_contact_regulation:
            p.error("moment control requires --disable-contact-regulation")
    if name == "teach_by_demonstration" and args.output.exists():
        p.error("output already exists; choose a new file")

    with robot_session(args, enable=name not in ("display_states", "test")) as arm:
        if name == "display_states":
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                s = arm.update_robot_states()
                print(
                    json.dumps(
                        {
                            k: list(getattr(s, k))
                            for k in (
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
        elif name == "move_arm_to_home":
            arm.go_home()
        elif name == "move_arm_to_joint_values":
            arm.move_arm_to_joint_values(args.joints)
            require_completed(arm.wait_for_joints(args.joints))
        elif name in ("move_eef_to_pose", "graspany"):
            arm.move_eef_to_goal_pose(target)
            require_completed(arm.wait_for_eef(target))
            if name == "graspany":
                arm.connect_gripper(args.port)
                arm.close_gripper(args.speed, args.gripper_force)
        elif name == "move_arm_follow_joint_values":
            # Validate every waypoint before executing the first.
            for q in waypoints:
                arm._joints(q)
            for q in waypoints:
                arm.move_arm_to_joint_values(q)
                require_completed(arm.wait_for_joints(q))
        elif name == "move_eef_follow_trajectory":
            for pose in waypoints:
                arm.move_eef_to_goal_pose(pose)
                require_completed(arm.wait_for_eef(pose))
        elif name in ("open_gripper", "close_gripper", "open_gripper_width"):
            arm.connect_gripper(args.port)
            value = {
                "open_gripper": 0,
                "close_gripper": 255,
                "open_gripper_width": getattr(args, "position", 100),
            }[name]
            arm.gripper_goto(value, args.speed, args.gripper_force)
        elif name == "zero_force_torque_sensors":
            arm.zero_force_torque_sensors()
            print("External WORLD wrench:", arm.get_tcp_wrench())
        elif name in ("force_control", "cartesian_impedance", "joint_impedance"):
            hold = arm.get_current_eef_pose()
            q = arm.get_current_joint_values()
            if name == "force_control":
                arm.configure_force_control(
                    [bool(a) for a in args.axes],
                    frame=args.frame,
                    passive=args.passive,
                    contact_wrench=[float("inf")] * 6 if args.disable_contact_regulation else None,
                )
            elif name == "cartesian_impedance":
                arm.configure_force_control([False] * 6)
                arm.set_cartesian_impedance([args.stiffness_scale * k for k in arm.robot.info().K_x_nom])
            else:
                arm.set_joint_impedance([args.stiffness_scale * k for k in arm.robot.info().K_q_nom])
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                states = arm.update_robot_states()
                wrench = np.asarray(states.ext_wrench_in_world)
                displacement = np.linalg.norm(np.array(states.tcp_pose[:3]) - hold.position)
                if not np.isfinite(wrench).all() or not np.isfinite(displacement):
                    raise RuntimeError("Invalid force/pose measurement")
                if (
                    displacement > args.max_displacement
                    or np.linalg.norm(wrench[:3]) > args.abort_force
                    or np.linalg.norm(wrench[3:]) > args.abort_moment
                ):
                    raise RuntimeError("Measured displacement or wrench exceeded the example limit")
                if name == "joint_impedance":
                    arm.move_arm_to_joint_values(q, impedance=True)
                else:
                    arm.move_eef_to_goal_pose_wrench(
                        hold, args.wrench if name == "force_control" else [0.0] * 6
                    )
                time.sleep(1.0 / arm.frequency)
            if name == "cartesian_impedance":
                arm.set_cartesian_impedance()
            elif name == "joint_impedance":
                arm.set_joint_impedance()
        elif name == "teach_by_demonstration":
            poses = []
            arm.execute_plan(args.plan, wait=False)
            print(
                "r: record current WORLD TCP pose, q: stop and save; replay using move_eef_follow_trajectory"
            )
            while True:
                command = input("r/q> ").strip().lower()
                if command == "q":
                    break
                if command == "r":
                    pose = arm.get_current_eef_pose()
                    poses.append(pose.position + pose.orientation)
                    print(f"Recorded {len(poses)} poses (xyzw)")
            with args.output.open("x") as stream:
                json.dump(poses, stream, indent=2)
        elif name == "test":
            q = arm.get_current_joint_values()
            pose = arm.joints_to_cartesian(q)
            solved = arm.cartesian_to_joints(pose, q)
            print("Offline base-frame FK/IK roundtrip (no motion):", solved)
            print(
                "Controller WORLD TCP (requires matching world/tool calibration to compare):",
                arm.get_current_eef_pose().to_rdk(),
            )
        else:
            raise ValueError(f"Unknown example {name}")
