# Rizon 4 SDK migration and force-control API

Baseline: repository main commit `68e94e8`; target official RDK `v1.9.4.1`, commit
`5338cc0316f34d7704be295bbc24b6e6f082de12`. Sources checked on 2026-11-16:

- [1.9.4.1 compatibility](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v1.9.4.1): controller 3.11.3/3.11.4.
- [2.1 compatibility](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v2.1): Enlight only.
- [2.2](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v2.2): WIP.
- [Versioned Robot API header](https://github.com/flexivrobotics/flexiv_rdk/blob/v1.9.4.1/include/flexiv/rdk/robot.hpp).
- [Official motion-force example](https://github.com/flexivrobotics/flexiv_rdk/blob/v1.9.4.1/example_py/intermediate4_non_realtime_cartesian_motion_force_control.py).

The highest PyPI version is not the correct SDK for this robot. Dependencies and
runtime checks pin 1.9.4.1, rather than using a range which could install 2.x.

## Existing problems addressed

| Original code | Current behavior |
| --- | --- |
| `Robot(robot_ip, local_ip)` | `Robot(robot_sn, network_interface_whitelist)` |
| camelCase state/motion API, `flexivrdk.Log` | PascalCase commands, snake_case accessors/data, Python logging |
| `getRobotStates(output)` | `states()` returns a snapshot |
| `sendJointPosition(q,dq,ddq,vmax,amax)` | `SendJointPosition(q,dq,vmax,amax)`; nonzero legacy target_acc rejected |
| Cartesian command omitted the new velocity argument | Explicit pose, wrench, 6D velocity, velocity and acceleration limits |
| Wrench command without enabling force axes | Required `configure_force_control()` and explicit axis mask |
| Pose objects and inconsistent quaternion conventions | Plain `[x,y,z,qw,qx,qy,qz]` lists throughout; input, output, teaching and replay all use wxyz |
| FK/IK hardcoded 0.15m gripper offset | Full configurable flange-to-TCP translation and rotation |
| IK limit check included fixed links and missed final joint | Validate exactly seven active joints and IK residual |
| EEF wait checked only position | Position, orientation and linear/angular velocity checked |
| Errors logged then swallowed; waits defaulted to 0.2/0.3s | Exceptions propagate; bounded monotonic waits; stop on failure/timeout |
| Mandatory ROS and gripper imports for arm-only use | Lazy optional gripper/IK loading; no ROS dependency in maintained code |
| Vendored old binaries and tracked caches | Official pinned wheel, package metadata, generated files ignored |

High-level method names are retained where practical. Constructor/CLI arguments
require migration. All pose parameters and results are now plain seven-element
`[x,y,z,qw,qx,qy,qz]` vectors, including `calculate_IK_error(goal_pose)`.
Teaching records and replay use this same wxyz order without conversion.
The previous 0.2.0 xyzw records must be reordered explicitly before replay; there is
no automatic guessing of quaternion order. The Pose class and its conversion methods
have been removed. SciPy operations also explicitly use `scalar_first=True` (SciPy >=1.14). The `target_acc` joint argument only tolerates all-zero
legacy input with a deprecation warning, because the new NRT API has no such field.

## Force, impedance and tool coverage

| BestMan API | Official RDK API / semantics |
| --- | --- |
| `configure_force_control(axes, frame, frame_transform, passive, max_linear_velocity, contact_wrench)` | Stop → IDLE-only `SetPassiveForceControl` → Cartesian mode → `SetForceControlFrame`, `SetForceControlAxis`, `SetMaxContactWrench` |
| `move_eef_to_goal_pose_wrench()` | `SendCartesianMotionForce`; targets TCP sensed wrench, N/Nm |
| `move_eef_to_goal_pose()` | Disables explicit force axes before pure motion |
| `set_cartesian_impedance(K, Z)` | `SetCartesianImpedance`; K ≤ controller K_x_nom, Z 0.3–0.8 |
| `set_joint_impedance(K, Z)` | NRT_JOINT_IMPEDANCE + `SetJointImpedance`; K ≤ K_q_nom |
| `move_arm_to_joint_values(..., impedance=True)` | Send joint position in joint impedance mode |
| `set_max_contact_wrench()` | Regulates motion-controlled Cartesian axes only |
| `set_max_contact_torque()` | Joint impedance contact-torque regulation |
| `set_null_space_posture()` / `set_null_space_objectives()` | Redundancy posture and objective weights for 7-DoF Rizon 4 |
| `get_tcp_wrench(frame, raw)` | Filtered or raw external wrench in WORLD/TCP |
| `get_joint_torques(external=True)` | External joint torques; default returns measured actuator torques |
| `zero_force_torque_sensors()` | `ExecutePrimitive("ZeroFTSensor", {})`, wait on structured `terminated` |
| `get_tool()` / `configure_tool()` | Read active tool; add/update mass, CoM, inertia and TCP in IDLE, then activate |
| `execute_primitive()` / `execute_plan()` | Structured primitive API / plans with bounded waits |
| `stop()` | Blocking `Stop()` to IDLE |

Units: joint angle rad, Cartesian position m, wrench N/Nm, joint stiffness Nm/rad,
Cartesian stiffness N/m and Nm/rad; damping inputs are dimensionless ratios, not
physical damping coefficients. All command vectors must be finite except deliberate
positive infinity in contact limits. Positive WORLD Fz commands motion toward WORLD
-Z until the environment applies the requested positive sensed force.

Motion poses always use WORLD. Force uses WORLD or moving TCP, optionally with a
flange-independent offset/rotation relative to that root. These axes need not be
orthogonal to the WORLD motion axes; the SDK projects motion accordingly. Offline
URDF FK/IK uses robot base coordinates and is not automatically WORLD-calibrated.

The force-axis velocity limit is 0.02m/s by default, accepted SDK range 0.005–2m/s.
It applies only to active linear force control. It does not bound passive force mode
or rotational moment-control axes. Finite contact-wrench regulation is incompatible
with any enabled moment axis; the wrapper rejects that combination before mutation.

A mode change invalidates the wrapper's force configuration. Configure again before
sending wrench commands. Do not change mode or force settings directly through
`arm.robot` while using the high-level wrapper, as same-mode direct mutations cannot
be detected reliably. Set impedance explicitly for each task; the wrapper does not
claim to restore all controller state after an exception.

## Tool configuration

Tool `tcp_pose` and force-control `frame_transform` both use
`[x,y,z,qw,qx,qy,qz]` with metres and a unit wxyz quaternion.
No actual tool mass/CoM/inertia is assumed. `config/rizon4.json` defaults to an
identity offline TCP transform, and it does not overwrite the controller's active
tool on connection. Use measured values, for example through:

```python
# Variables below come from measurement/calibration, not example defaults.
arm.configure_tool(
    "measured_tool", mass=measured_mass_kg,
    center_of_mass=measured_com_in_flange_m,
    inertia=measured_inertia_at_com_kg_m2,
    tcp_pose=measured_flange_relative_pose,  # [x,y,z,qw,qx,qy,qz], wxyz
)
```

This API changes controller tool parameters and also updates this instance's offline
FK/IK transform. `get_tool()` reads the active controller tool. Sensor zeroing must
occur at rest without environment contact and after correct payload configuration.
The supplied URDF contains blurred dynamics, so it is not a torque-control model.

## Python and historical scope

Maintained Python examples are NRT. Real-time 1kHz joint torque streaming requires
the official C++ scheduler/RT API; no misleading Python torque loop is provided.
`Demos/` and `TODO/` are archived applications tied to old ROS APIs and experiment
calibrations. Only `Robotics_API`, `Examples`, configuration and installation paths
are migrated. RealSense and serial gripper hardware behavior was not validated.
