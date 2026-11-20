<img src="docs/BestMan_logo.png" alt="BestMan Logo" width="700"/>

# BestMan Flexiv — Rizon 4 / RDK 1.9.3

A Python interface for non-real-time control of **Rizon 4 (7 joints, no external axes)**.
It supports joint and Cartesian motion, hybrid motion-force control, active and passive
force control, Cartesian and joint impedance, external wrench readings, sensor zeroing,
tool configuration, and serial Robotiq grippers.

## SDK version

This project pins **`flexivrdk==1.9.3`** for **Rizon 4 with controller software
v3.11.2**. Package installation, setup scripts and runtime checks all enforce this
version. An existing 1.9.4.1 installation is replaced by 1.9.3 when installing this
project; no version flag or dependency-file edit is needed.

The native SDK still checks the controller pairing during connection. RDK 2.x
supports Enlight and is not compatible with this Rizon 4 profile. In 1.9.3, waypoint
batches use controller-selected timing: omit `durations` or pass only zeros.
Nonzero per-waypoint durations are rejected before any command is sent.

- [RDK 1.9.3 release notes](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v1.9.3)
- [RDK 2.1 Enlight compatibility](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v2.1)
- [Migration guide and force-control APIs](docs/SDK_MIGRATION.md)
- [Validation results and simulation limitations](docs/VALIDATION.md)
- [Python SDK coverage and servo APIs](docs/PYTHON_SDK_COVERAGE.md)

## Installation

Ubuntu 22.04+ with Python 3.12 is recommended. Official Python 3.10 wheels are also
available for Linux, Windows, and macOS arm64. Python 3.8 is no longer supported.

```bash
git clone https://github.com/UMI-Robotics/BestMan_Flexiv.git
cd BestMan_Flexiv
bash Install/setup.sh
.venv/bin/python Examples/display_states.py Rizon4-123456
```

For a downloaded ZIP, extract it and run the same setup command inside the project.
The script creates `.venv`, installs this project and RDK 1.9.3, and checks imports.
It uses a regular package install; rerun setup after updating the checkout.
It locates the project from its own file, so it also works when launched from another
directory. Use `--extras gripper`, `--extras camera`, or `--extras dev` when needed;
extras can be combined as `--extras dev,gripper`.
`--rdk-version 1.9.3` remains accepted for existing commands, but is optional.

For the existing `dj-stuff` Conda environment and controller **3.11.2**, install into
that environment directly; no additional `.venv` is needed:

```bash
conda activate dj-stuff
python -m pip install -e .
```

After `source .venv/bin/activate`, use `python Examples/display_states.py ...` or
`python -m Examples.display_states ...`. The installed package and its URDF resources
work from any working directory; neither source edits nor `sys.path`/`PYTHONPATH`
changes are needed. Replace `Rizon4-123456` with the actual robot serial number.

ROS, `roscore`, and manual SDK path configuration are not required. If you previously
used the old installation scripts, remove any `PYTHONPATH` entries pointing to
`Install/flexiv_rdk/lib_py` from your shell configuration. Legacy SDK binaries and
Python caches have been removed from this branch.
See [Install/install.md](Install/install.md) for detailed instructions.

## Connection and motion

Connect using the robot's **serial number**, shown on the robot or controller,
instead of the old `robot_ip, local_ip` arguments. Optional network interface
restrictions use the local NIC's IPv4 address. Construction connects and reads
information; it does not enable or move the robot.

```python
from Robotics_API import Bestman_Real_Flexiv

arm = Bestman_Real_Flexiv("Rizon4-123456")  # Built-in defaults; no config path required.
try:
    arm.initialize_robot()
    # Inputs and outputs: [x, y, z, qw, qx, qy, qz], with position in metres.
    goal = arm.get_current_eef_pose()
    goal[2] += 0.01
    arm.move_eef_to_goal_pose(goal)
    if not arm.wait_for_eef(goal):
        raise TimeoutError("Target not reached; the robot has been stopped")
finally:
    arm.stop()
```

**All poses are plain seven-element lists `[x, y, z, qw, qx, qy, qz]`, with wxyz quaternion order.**
The same convention applies to inputs, outputs, CLI `--pose` arguments, teaching JSON,
trajectory replay, tool TCPs, force-control frame transforms, and FK/IK. No pose objects
are needed. Tuples and NumPy arrays in the same order are also accepted as inputs;
outputs are plain lists. Commands validate vector length, finite values, and unit
quaternions without reordering components or automatically normalizing them.

## NRT servo control

Use `servo_joint_position()` for successive joint targets, `servo_cartesian_pose()`
for pure Cartesian motion, or `servo_cartesian_motion_force()` for hybrid motion-force
targets. Cartesian poses remain plain `[x,y,z,qw,qx,qy,qz]` lists (wxyz).
These calls do not wait for arrival; the controller replans and smooths each target.

Each servo example shows its own 1–100 Hz `while` loop, target calculation, direct
`servo_*` call, timing checks and `finally: arm.stop()`. Finish mode/force configuration
before entering the loop. Python timing is best-effort; lag can only be detected
after a blocked API call returns.

```bash
# Hold the current joint positions at 50 Hz for five seconds
python -m Examples.servo_joint_position Rizon4-123456 --frequency 50 --duration 5
# Small Cartesian sine motion; all poses and outputs use wxyz
python -m Examples.servo_cartesian_motion_force Rizon4-123456 --amplitude 0.005
# Add 3 N sensed WORLD Z force; prepare tool calibration and contact conditions first
python -m Examples.servo_cartesian_motion_force Rizon4-123456 --force-z 3
# Read status, plans, digital inputs and commanded actions without enabling motion
python -m Examples.sdk_status Rizon4-123456
```

The Python RDK 1.9.3 wheel exposes NRT `Send*` calls, **not** RT `Stream*` calls or
RT modes. This is version-specific: **Python RDK 2.1.0 does export RT calls**, but it
supports Enlight, not this Rizon 4 profile. See the [actual binary probe results](docs/PYTHON_RT_PROBE.md).
For Rizon 4 on 1.9.3, RT execution requires a compatible C++ backend, which can
also be exposed to Python. `Enable()` is the Rizon motor-enable API; this version
has no Python `ServoOn()`/`Disable()` robot API, and `Brake()` is restricted to medical models.

The [SDK coverage guide](docs/PYTHON_SDK_COVERAGE.md) accounts for all 53 Python Robot
members: 49 covered, four excluded for the Rizon 4/NRT profile. It also documents
onboard waypoint batches, plan pause/resume and breakpoints, asynchronous primitives,
joint inertia shaping, digital IO, global variables, tool/work-coordinate management,
and access to the eight native SDK service interfaces.

## Force control

Configure the actual tool mass, centre of mass, inertia, and TCP before zeroing sensors
with the robot stationary and free of environmental contact. The `flange_to_tcp` field
in `config/rizon4.json` is used only for offline FK/IK and does not overwrite the
controller's tool configuration. Use `configure_tool()` to set controller tool parameters
from actual measurements.

```python
arm.initialize_robot()
try:
    # Run only while stationary and free of contact: arm.zero_force_torque_sensors()
    hold = arm.get_current_eef_pose()  # [x,y,z,qw,qx,qy,qz], wxyz
    arm.configure_force_control(
        [False, False, True, False, False, False],
        frame="WORLD", passive=False,
        max_linear_velocity=[0.02, 0.02, 0.02],
    )
    # Target wrench is sensed at the TCP; WORLD +Fz presses toward world -Z.
    arm.move_eef_to_goal_pose_wrench(hold, [0, 0, 3, 0, 0, 0])
    # The command persists; monitor contact, travel and measured wrench, then stop explicitly.
finally:
    arm.stop()
```

`configure_force_control()` first stops the robot in IDLE, then configures active or
passive force control, the reference frame, and the controlled axes.
`SetMaxContactWrench` **limits only the motion-control component**; it is not global
collision detection. The SDK does not allow this regulation while rotational moment
control is enabled; explicitly pass `[float("inf")]*6` for that combination.
Passive force control uses open-loop wrench feedforward, so the SDK's active
force-axis velocity protection does not apply. Python polling limits are not
controller-level real-time protection either.

## Examples

Every example is a complete, independent script. Open one file to see argument parsing,
connection, initialization, mode/force configuration, API calls, waiting or polling,
and cleanup. Read-only examples only connect and read. Servo loops are written directly
in their files, without a dispatcher, shared lifecycle helper or callback runner.

After installation and environment activation, the module commands below work from
any directory. Alternatively, run a file directly, e.g.
`python Examples/force_control.py Rizon4-123456` from the checkout. Replace the example
serial number with your robot's actual serial number. The default frequency is 20 Hz; use
`--frequency` to select 1–100 Hz. All robot examples support
an optional `--config` file and repeatable `--interface 192.168.2.108` arguments.
Built-in defaults need no configuration file. User-supplied config/trajectory/output
paths are relative to the current directory, or may be absolute.

| Independent example | What the file demonstrates |
| --- | --- |
| [display_states.py](Examples/display_states.py) | Read joint, TCP and force states. |
| [sdk_status.py](Examples/sdk_status.py) | Read status, plans, IO, events and commanded TCP. |
| [move_arm_to_home.py](Examples/move_arm_to_home.py) | Enable, select plan mode and run Home. |
| [move_arm_to_joint_values.py](Examples/move_arm_to_joint_values.py) | Send a joint target and wait for arrival. |
| [move_arm_follow_joint_values.py](Examples/move_arm_follow_joint_values.py) | Read and execute joint waypoints from JSON. |
| [move_eef_to_pose.py](Examples/move_eef_to_pose.py) | Send a WORLD wxyz TCP target and wait. |
| [move_eef_follow_trajectory.py](Examples/move_eef_follow_trajectory.py) | Replay WORLD wxyz waypoints from JSON. |
| [servo_joint_position.py](Examples/servo_joint_position.py) | Compute and send joint targets in an explicit timed loop. |
| [servo_cartesian_motion_force.py](Examples/servo_cartesian_motion_force.py) | Stream NRT Cartesian targets with optional Z force. |
| [force_control.py](Examples/force_control.py) | Configure force frame/axes and send a bounded wrench. |
| [cartesian_impedance.py](Examples/cartesian_impedance.py) | Set Cartesian stiffness and hold the current TCP. |
| [joint_impedance.py](Examples/joint_impedance.py) | Set joint stiffness and hold the current joint positions. |
| [zero_force_torque_sensors.py](Examples/zero_force_torque_sensors.py) | Select primitive mode and run sensor zeroing. |
| [teach_by_demonstration.py](Examples/teach_by_demonstration.py) | Run free-drive, record wxyz lists and save JSON. |
| [open_gripper.py](Examples/open_gripper.py) | Connect and open a serial Robotiq gripper. |
| [close_gripper.py](Examples/close_gripper.py) | Connect and close the gripper. |
| [open_gripper_width.py](Examples/open_gripper_width.py) | Send a raw 0–255 gripper position. |
| [graspany.py](Examples/graspany.py) | Move to a TCP target, wait and close the gripper. |
| [get_rgbd.py](Examples/get_rgbd.py) | Display RealSense RGBD (camera extra required). |
| [test.py](Examples/test.py) | Read joints and compare offline FK/IK, without motion. |

```bash
# Read states without enabling the robot
python -m Examples.display_states Rizon4-123456 --duration 5
# Move to the controller's Home posture
python -m Examples.move_arm_to_home Rizon4-123456
# Zero sensors while stationary and free of contact; no automatic Home motion
python -m Examples.zero_force_torque_sensors Rizon4-123456
# Apply 3 N along Z from the current pose, for 5 seconds, with a 3 cm travel limit
python -m Examples.force_control Rizon4-123456 --duration 5
# TCP frame: choose the force direction according to the actual tool orientation
python -m Examples.force_control Rizon4-123456 --frame TCP --wrench 0 0 -3 0 0 0
# Hold the current pose compliantly at 30% of the controller's nominal stiffness
python -m Examples.cartesian_impedance Rizon4-123456 --stiffness-scale 0.3
python -m Examples.joint_impedance Rizon4-123456 --stiffness-scale 0.3
# Record [x,y,z,qw,qx,qy,qz] directly; replay uses the same wxyz order
python -m Examples.teach_by_demonstration Rizon4-123456 --output taught_poses.json
python -m Examples.move_eef_follow_trajectory Rizon4-123456 --trajectory taught_poses.json
# Gripper position uses raw 0–255 commands, not millimetres
python -m Examples.open_gripper_width Rizon4-123456 --port /dev/ttyUSB0 --position 100
```

Force-control and impedance examples enforce limits on duration, TCP displacement,
and measured force/moment. They do not perform contact search or automatic sensor
zeroing; start from a checked working pose. All motion examples call `Stop()` on normal
exit, exceptions, and Ctrl+C. Joint targets, Cartesian targets, and trajectories must
be supplied explicitly through the CLI instead of using hardcoded paths from the
original experimental setup. See `python -m Examples.<name> --help` for parameters.

`Demos/` and `TODO/` are retained as historical experiments that depend on old APIs
and ROS; they are not maintained entry points for this version. See
[Demos/README.md](Demos/README.md). `Sensor/Camera.py` is the original camera module;
camera calibration and hardware behavior have not been validated in this migration.

## Validation and scope

Tests check symbols and signatures against the real RDK wheel and use a strict
command recorder to verify arguments, call order, and failure paths. These checks
do not establish physical force-control performance. DJ-Sim on 5090-2 currently uses
Isaac/PhysX control interfaces and lacks a Flexiv RDK network service and closed-loop
force-control implementation, so it cannot validate force tracking on the real controller.

The Python interface provides NRT control only. Use the official C++ API for 1000 Hz
real-time joint torque or Cartesian control. The offline Rizon 4 URDF is a kinematic
resource with blurred dynamic parameters; it must not be used for torque control or
gravity compensation.

## Contact and references

- dingyan at pjlab.org.cn
- zhaxizhuoma at pjlab.org.cn
- [Flexiv RDK](https://www.flexiv.com/software/rdk)
- [IKPy](https://ikpy.readthedocs.io/en/latest/)
