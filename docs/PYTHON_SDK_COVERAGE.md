# Python SDK coverage and servo control

Audit date: 2026-11-16. Target: **Rizon 4, flexivrdk 1.9.4.1**, controller 3.11.3/3.11.4.
The inventory is checked against the installed official Python binary, not inferred
from C++ documentation. The Robot class exports **53 public methods/accessors**:
**49 are covered**, with four documented exclusions for this robot/Python profile.

## What “servo” means in this Python interface

The Python wheel exposes NRT modes and `SendJointPosition` / `SendCartesianMotionForce`.
It does **not** export `StreamJointPosition`, `StreamJointTorque`,
`StreamCartesianMotionForce`, `Scheduler`, or RT modes. Do not use a C++ API name
as if it existed in Python. This remains true of the Linux wheel tested on 5090-02.

The servo methods send successive setpoints to the controller's internal motion
generator. They return after dispatch, without waiting for each target to be reached.
Updating a target replans the trajectory. This is suitable for NRT target following;
it is not the SDK's 1 kHz real-time streaming mode.

| BestMan method | Inputs and behavior |
| --- | --- |
| `servo_joint_position(positions, velocities=None, ...)` | Seven joint positions in radians, optional rad/s velocity, per-joint velocity/acceleration limits; `impedance=True` retains joint impedance mode. |
| `servo_cartesian_pose(pose, velocity=None, ...)` | Pure motion; WORLD `[x,y,z,qw,qx,qy,qz]`, velocity `[vx,vy,vz,wx,wy,wz]`. |
| `servo_cartesian_motion_force(pose, wrench, velocity=None, ...)` | Same WORLD wxyz pose/velocity; sensed wrench in the force frame selected by `configure_force_control()`. |
| `run_servo(callback, duration, frequency=None, max_lag=0.1)` | Calls `callback(elapsed_seconds)` at 1–100 Hz and stops on normal exit, exception, fault or observed excessive lag. |

Complete blocking mode changes and force/impedance configuration **before** starting
the timed loop. Repeated commands in the same mode do not switch modes or reset force
axes. The loop skips missed periods instead of issuing a burst of catch-up commands.
Timing uses the monotonic clock. A callback that blocks cannot be interrupted by the
same Python thread; lag is detected on return. Process termination, disconnected
network behavior and physical stopping remain controller/hardware concerns. This
helper is not a hardware watchdog or a guarantee of hard real-time timing.

```python
import math
from Robotics_API import Bestman_Real_Flexiv

arm = Bestman_Real_Flexiv("Rizon4-123456", frequency=50)
try:
    arm.initialize_robot()
    arm.configure_force_control([False] * 6)
    initial = arm.get_current_eef_pose()  # [x,y,z,qw,qx,qy,qz], wxyz

    def update(elapsed):
        target = initial.copy()
        target[0] += 0.005 * math.sin(2 * math.pi * 0.25 * elapsed)
        arm.servo_cartesian_pose(target)

    arm.run_servo(update, duration=5)
finally:
    arm.stop()
```

The runnable Cartesian example additionally monitors measured force/moment and TCP
travel. Both servo examples hold position by default (`--amplitude 0`) and never
move Home or zero sensors automatically:

```bash
python -m Examples.servo_joint_position Rizon4-123456 --frequency 50 --duration 5
python -m Examples.servo_joint_position Rizon4-123456 --amplitude 0.05 --joint 6
python -m Examples.servo_cartesian_motion_force Rizon4-123456 --amplitude 0.005
python -m Examples.servo_cartesian_motion_force Rizon4-123456 --force-z 3 --axis x
python -m Examples.sdk_status Rizon4-123456
```

## Robot API inventory

| Python SDK member | BestMan coverage / reason for exclusion |
| --- | --- |
| `Brake` | Not wrapped: medical models only; unavailable for the configured Rizon 4. |
| `ClearFault` | `clear_fault()`; also used by `initialize_robot()`. |
| `Enable` | `enable()` / `initialize_robot()`; no Python `ServoOn` or `Disable` method in this version. |
| `ExecutePlan` | `execute_plan()`; name/index overload, async start, continue_execution and start synchronization. |
| `ExecutePrimitive` | `execute_primitive()`; async start plus `wait_for_primitive(state, value)`. |
| `LockExternalAxes` | Not wrapped: this Rizon 4 profile rejects external axes. |
| `PausePlan` | `pause_plan(True/False)`. |
| `RunAutoRecovery` | `run_auto_recovery()`; explicit recovery state required, reboot afterwards. |
| `SendCartesianMotionForce` | `servo_cartesian_pose()` / `servo_cartesian_motion_force()` and existing move methods. |
| `SendJointPosition` | `servo_joint_position()` and `move_arm_to_joint_values()`. |
| `SendMultiCartesianMotionForce` | `send_cartesian_waypoints()` in NRT_SUPER_PRIMITIVE. |
| `SetBreakpointMode` | `set_breakpoint_mode()`. |
| `SetCartesianImpedance` | `set_cartesian_impedance()`. |
| `SetDigitalOutputs` | `set_digital_outputs({port: bool})`, ports 0–17. |
| `SetForceControlAxis` | `configure_force_control()`. |
| `SetForceControlFrame` | `configure_force_control()`; WORLD/TCP and wxyz transform. |
| `SetGlobalVariables` | `set_global_variables()`; variables must already exist in Elements. |
| `SetJointImpedance` | `set_joint_impedance()`. |
| `SetJointInertiaScale` | `set_joint_inertia_scale()`, seven scales in [0.75, 1]. |
| `SetMaxContactTorque` | `set_max_contact_torque()`, finite values in [0, RobotInfo.tau_max]. |
| `SetMaxContactWrench` | `set_max_contact_wrench()` / force configuration. |
| `SetNullSpaceObjectives` | `set_null_space_objectives()`. |
| `SetNullSpacePosture` | `set_null_space_posture()`. |
| `SetPassiveForceControl` | `configure_force_control(passive=...)`, applied in IDLE. |
| `SetTimelinessFailureLimit` | Not wrapped: applies to RT commands, which are absent from the Python binding. It is not an NRT watchdog. |
| `SetVelocityScale` | `set_velocity_scale()`, integer percent 0–100, plan/primitive modes only. |
| `StepBreakpoint` | `step_breakpoint()`. |
| `Stop` | `stop()` and servo-loop cleanup; stops and enters IDLE, does not disable motors. |
| `StopPlan` | `stop_plan()`; stops the plan without promising a mode transition. |
| `SwitchMode` | `switch_mode()` and automatic mode selection by motion methods. |
| `SyncWithPositioner` | Not wrapped: no positioner/external axes in this profile. |
| `actions` | `get_robot_actions()`; target TCP uses wxyz. |
| `busy` | `get_status()["busy"]`; plan completion waits. |
| `connected` | `get_status()["connected"]`. |
| `digital_inputs` | `get_digital_inputs()`, 18 bits. |
| `enabling_button_pressed` | `get_status()["enabling_button_pressed"]`. |
| `estop_released` | `get_status()["estop_released"]`. |
| `event_log` | `get_event_log()`. |
| `fault` | `get_status()["fault"]`; command/wait fault checks. |
| `global_variables` | `get_global_variables()`. |
| `info` | `get_robot_info()` and existing joint/model accessors. |
| `lite` | `get_status()["lite"]`; this wrapper constructs normal instances. |
| `mode` | `get_status()["mode"]`; internal mode checks. |
| `operational` | `get_status()["operational"]`; initialization and command checks. |
| `operational_status` | `get_status()["operational_status"]`. |
| `plan_info` | `get_plan_info()`; SDK requires plan mode. |
| `plan_list` | `get_plan_list()`. |
| `primitive_states` | `get_primitive_states()` / `wait_for_primitive()`. |
| `reached_timeliness_failure_limit` | `get_status()["reached_timeliness_failure_limit"]`; RT diagnostic only. |
| `recovery` | `get_status()["recovery"]` and recovery-state check. |
| `reduced` | `get_status()["reduced"]`. |
| `states` | `update_robot_states()` and existing state/pose/wrench accessors. |
| `stopped` | `get_status()["stopped"]`. |

## Onboard waypoint batches

`send_cartesian_waypoints()` passes an entire sequence to the controller rather
than using a Python timing loop. It requires NRT_SUPER_PRIMITIVE, one Cartesian
wxyz pose **and** one seven-joint target for every waypoint. These targets must be
consistent with the actual tool, WORLD/base transform and desired posture. Do not
substitute uncalibrated offline IK results for the joint targets.

```python
arm.configure_force_control([False] * 6, super_primitive=True)
arm.send_cartesian_waypoints(
    poses, joint_positions, durations=[1.0] * len(poses),
    max_linear_vel=0.03,
)
```

Each pose is a plain `[x,y,z,qw,qx,qy,qz]` list. Optional `wrenches` and `velocities`
have one six-element vector per waypoint. Duration 0 lets the SDK choose timing.
The SDK `NrtCartesianCmd` objects are constructed internally; users do not need to
create pose objects. Configure force axes explicitly before nonzero wrench targets.
Dispatch does not wait for batch completion. Monitor the task and stop explicitly;
a final pose alone is not proof that all intermediate waypoints executed.

## Plans, primitives and IO

`execute_plan()` accepts a plan name or zero-based index. `wait=False` starts a plan
without waiting for completion. `continue_execution=True` is an explicit opt-in to
execution after losing the RDK connection; robot faults still stop execution.
`pause_plan(False)` resumes user plans; the SDK does not allow resuming internal plans.

Many primitives never terminate automatically. Use `execute_primitive(..., wait=False)`
and `wait_for_primitive("reachedTarget", 1, timeout=...)` where that primitive documents
this state. Keep `block_until_started=True` before any completion/state wait, so an
old state snapshot cannot be mistaken for the newly requested primitive's result.

Primitive parameter maps and Elements global variables retain their **native SDK
schemas and units**. In particular, the SDK's `Coord` primitive-parameter type uses
Euler angles, not a seven-element robot pose; consult the primitive documentation.
All BestMan pose inputs/outputs and recording/servo/waypoint paths remain wxyz.

Digital IO uses port indices 0–17 and boolean output values. Global variables must
already exist in Elements; use integer 0/1 where that schema represents booleans.
`set_velocity_scale()` applies only to plans/primitives, not direct servo commands.

## SDK service interfaces

`arm.sdk_interface(name, ...)` constructs the native service below and returns it
without invoking any operation. Keep the object for reuse outside time-sensitive
loops. This is native SDK access, not a claim that every service operation has a
custom BestMan wrapper or has been exercised against hardware.

| Service | Access and scope |
| --- | --- |
| `Tool` | `list_tools`, `get_tool`, `configure_tool`, `switch_tool`, `remove_tool`; native interface also exposes payload calibration. Wrapped changes use IDLE; switching/updating synchronizes the offline TCP. |
| `WorkCoord` | `list_work_coordinates`, `get_work_coordinate`, `configure_work_coordinate`, `remove_work_coordinate`; WORLD wxyz lists throughout. |
| `Gripper` | `sdk_interface("Gripper")`: native Enable/Disable/Init/Move/Grasp/Stop/params/states. This is the controller-connected gripper, distinct from existing serial Robotiq methods; native widths/velocities/forces use m, m/s and N. |
| `Device` | `sdk_interface("Device")`: configured external devices, status and commands. |
| `Model` | `sdk_interface("Model", gravity_vector=[0,0,-9.81])`: native model dynamics and kinematics; requires RDK Professional. |
| `Safety` | `sdk_interface("Safety", password=...)`: native safety settings; explicit password required and not stored on the wrapper. No limits are changed by constructing it. |
| `FileIO` | `sdk_interface("FileIO")`: native project/trajectory exchange and collision meshes. |
| `Maintenance` | `sdk_interface("Maintenance")`: native torque-sensor calibration and PTP configuration. |

Native calls obey their own SDK mode/license restrictions. Directly changing mode,
force settings or the active tool through `arm.robot` or a native service bypasses
BestMan's cached configuration; reconfigure before returning to high-level commands.
Native `Tool.Switch/Update` does not synchronize offline FK/IK; use the wrapped tool
methods for that. Recovery, calibration, safety changes and physical IO were not
executed during development.

## Sources

- [RDK v1.9.4.1 Robot header](https://github.com/flexivrobotics/flexiv_rdk/blob/v1.9.4.1/include/flexiv/rdk/robot.hpp)
- [RDK data types and waypoint commands](https://github.com/flexivrobotics/flexiv_rdk/blob/v1.9.4.1/include/flexiv/rdk/data.hpp)
- [Official Python joint NRT example](https://github.com/flexivrobotics/flexiv_rdk/blob/v1.9.4.1/example_py/intermediate1_non_realtime_joint_position_control.py)
- [Official Python motion-force example](https://github.com/flexivrobotics/flexiv_rdk/blob/v1.9.4.1/example_py/intermediate4_non_realtime_cartesian_motion_force_control.py)
- [RDK 1.9.4.1 package](https://pypi.org/project/flexivrdk/1.9.4.1/)
