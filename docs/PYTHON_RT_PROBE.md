# Python RT export verification — 2026-11-16

**Official Python RDK 2.1.0 does export the three RT `Stream*` methods and RT modes.**
The absence reported for this repository applies specifically to its pinned,
Rizon-compatible **1.9.4.1** wheel. It must not be generalized to every Python RDK
version. RDK 2.1 officially supports **Enlight with robot software v3E.1**, not the
Rizon 4 / controller 3.11.3–3.11.4 profile used here.

## Actual binary results

Official CPython 3.12 wheels were downloaded from PyPI and checked against their
published SHA-256 hashes. Each was extracted into a separate directory and imported
in a fresh process. The existing application environment was not upgraded.

| SDK version | macOS arm64 / Python 3.12.13 | 5090-02 Linux x86_64 / Python 3.12.14 |
| --- | --- | --- |
| 1.8.0 | Import failed: legacy wheel requires a missing Homebrew OpenSSL 3 library; no export conclusion on macOS | All three Stream methods and RT modes absent |
| 1.9.0 | All three Stream methods and RT modes absent | All three Stream methods and RT modes absent |
| 1.9.4.1 | All three Stream methods and RT modes absent | All three Stream methods and RT modes absent |
| 2.1.0 | All three Stream methods and RT modes present | All three Stream methods and RT modes present |

The checked methods are:

- `Robot.StreamJointPosition`
- `Robot.StreamJointTorque`
- `Robot.StreamCartesianMotionForce`

The checked modes are `RT_JOINT_POSITION`, `RT_JOINT_IMPEDANCE`, `RT_JOINT_TORQUE`,
and `RT_CARTESIAN_MOTION_FORCE`. `Scheduler` is not exported in any of the
successfully imported wheels, including 2.1.0.

For 1.9.4.1, direct Python attribute lookup produces, for example:

```text
type object 'flexivrdk._flexivrdk.Robot' has no attribute 'StreamJointPosition'
```

The public `Robot` class is the same class as the extension's native `Robot` class;
the methods are not merely hidden by the package-level import. This check bypasses
the BestMan wrapper entirely.

## RDK 2.1.0 signatures and command construction

The actual 2.1.0 binding exposes these signatures on both tested platforms:

```text
StreamJointPosition(self, cmds: dict[JointGroup, RtJointPositionCmd]) -> None
StreamJointTorque(self, cmds: dict[JointGroup, RtJointTorqueCmd]) -> None
StreamCartesianMotionForce(self, cmds: dict[JointGroup, RtCartesianCmd]) -> None
```

These are callable native method descriptors. Native command construction also
succeeded on both platforms:

```python
# RDK 2.1.0 only: data construction, without creating or commanding a robot.
import flexivrdk

q_cmd = flexivrdk.RtJointPositionCmd([0.0] * 7, [0.0] * 7, [0.0] * 7)
tau_cmd = flexivrdk.RtJointTorqueCmd([0.0] * 7)
pose = [0.4, 0.1, 0.5, 0.8, 0.2, 0.4, 0.4]  # [x,y,z,qw,qx,qy,qz], wxyz
cart_cmd = flexivrdk.RtCartesianCmd(pose)
assert list(cart_cmd.pose_d) == pose
```

This verifies native data types and wxyz preservation. It does **not** establish
that a specific robot accepts these commands, that a particular seven-joint target
matches a configured joint group, or that a Python loop satisfies RT deadlines.
No `Robot` instance was constructed; no network connection, robot motion or
1 kHz timing test was performed.

## Implications for this Rizon 4 repository

The installed dependency remains `flexivrdk==1.9.4.1`. The 2.1.0 exports are not a
reason to install an Enlight-only SDK on a Rizon 4 deployment. The 1.x manual's
Python access table explicitly excludes RT functions and modes, consistent with
the tested 1.x binaries. As checked on this date, `v1.10` is a Draft/WIP release and
no 1.10 wheel is published on PyPI; it was not runtime-tested.

Python applications can also drive Rizon RT control through an additional native
backend. One concrete example is XenseRobotics-AI's `libpyflexiv`, whose source at
commit `f3b60475237580f5cc488cc2dd7614674e599307` exposes Python target-setting APIs
and calls `StreamJointPosition` / `StreamCartesianMotionForce` from a C++ scheduler.
Its dependency is RDK v1.8. This source was inspected, but the extension was not
built, integrated or tested against hardware here. A backend for the current
repository must match RDK 1.9.4.1 and the controller version; compatibility must not
be assumed from the fact that another version's wrapper exists.

## Reproduce the export probe

The probe is independent of BestMan and never connects to a robot:

```bash
python tools/probe_flexiv_python_rt.py --output rt_exports.json
# Inspect an extracted official wheel in isolation, using a matching Python ABI:
python tools/probe_flexiv_python_rt.py --package-dir /path/to/extracted/wheel --output rt_exports.json
```

It records the imported version and binary paths, the public/native class identity,
method signatures or AttributeError details, RT mode availability, Scheduler
availability, and native RT command construction when those data types are present.

## Primary sources

- [Official RDK 1.x API access matrix](https://www.flexiv.com/software/rdk/manual/v1.x/api_overview.html)
- [Official RDK 2.1 release compatibility](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v2.1)
- [Official RDK 1.10 Draft/WIP](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v1.10)
- [PyPI 1.9.4.1](https://pypi.org/project/flexivrdk/1.9.4.1/) and [PyPI 2.1.0](https://pypi.org/project/flexivrdk/2.1.0/)
- [libpyflexiv source](https://github.com/XenseRobotics-AI/libpyflexiv/tree/f3b60475237580f5cc488cc2dd7614674e599307)
