# Validation

## 2026-11-20 — pressed button still leaves the internal plan in Hold

An initial read-only connection to a Rizon 4 using RDK 1.9.3 observed the enabling
button as released. A follow-up observation while the operator held the button
captured **300 samples over 30.47 seconds**: every sample reported
`enabling_button_pressed=True`, READY, no fault, NRT_PLAN_EXECUTION, and an active
`PLAN-FreeDriveAuto` stuck at `rootNode::hold` / `Hold`. TCP position ranges were
below 0.014 mm per axis. There was no breakpoint wait or timeliness failure, and the
event log was empty. The button signal was working; adding button instructions did
not resolve the failure. The internal plan's transition conditions were not exposed
by the available read-only APIs, and FileIO could not export it as a user project.

The example now uses the documented RDK 1.x free-drive entry point:
`ExecutePrimitive("FloatingCartesian", {})` in NRT_PRIMITIVE_EXECUTION, with
`FloatingJoint` available as an option. See the
[official free-drive manual](https://www.flexiv.com/software/rdk/manual/v1.x/free_drive.html).
It no longer invokes `PLAN-FreeDriveAuto` or accepts `--plan`.

The primitives do not enforce the enabling button themselves. The example requires
an observed release before the first activation, polls the button at 50 Hz, and
calls Stop on release. Nonblocking terminal reads keep incomplete/no keyboard input
from suspending that loop. Primitive loading is requested without waiting for start;
the loop checks for the actual primitive name and running state before reporting
ACTIVE. This Python gate is not a replacement for the robot's safety functions and
does not provide a real-time stopping guarantee.

Regression coverage includes press/release transitions without keyboard input,
startup with the button held, release during mode switching or primitive loading,
faults, disconnect, mode changes, primitive termination/start timeout, polling delay,
license/command failure, EOF/Ctrl+C, partial terminal input, and exact wxyz
recording/replay. Existing records are saved even on controller errors.

- macOS arm64 / Python 3.12.13 / official RDK 1.9.3: **205 tests passed**, including
  18 teaching regression cases. The old default failed the primitive-dispatch regression.
- Ruff and `git diff --check`: passed; the existing IKPy warning remains.

All hardware observations were read-only and preserved the existing teaching
process. The revised primitive path has not been executed on the physical robot;
actual free-drive behavior and release-to-stop response still need an operator test.

## 0.3.1 — RDK 1.9.3 / controller 3.11.2 (2026-11-20)

Package metadata, runtime checks, the Conda YAML and both SDK installation scripts
now pin `flexivrdk==1.9.3`. This pairing targets controller software **3.11.2**.
The independent examples and plain wxyz input/output conventions are unchanged.

- macOS arm64 / Python 3.12.13 / official RDK 1.9.3: **188 tests passed**.
- `djdemo-desktop` Linux x86_64 / Python 3.10.22 / RDK 1.9.3: **268 tests passed**
  (the 188 maintained checks plus 80 existing local example-entry-point checks).
  The `dj-stuff` environment has BestMan 0.3.1 with the exact 1.9.3 SDK dependency;
  isolated imports succeed without a checkout path or `PYTHONPATH`.
- Ruff, shell syntax and `git diff --check`: passed.
- The actual 1.9.3 binary still exports 53 public Robot members. Its
  `NrtCartesianCmd` has no duration field: batch construction uses seven arguments;
  omitted/all-zero durations work, and nonzero durations fail before dispatch.
- Version mismatches are rejected before connection. The native SDK/controller
  handshake remains enabled and its errors propagate unchanged.
- Python RT methods and modes are absent in both the macOS wheel and the Linux
  x86_64 wheel installed in `djdemo-desktop`'s Python 3.10.22 `dj-stuff` environment.
- `Maintenance.SetPtpConfig` and `ptp_config` are absent in 1.9.3; the SDK coverage
  guide no longer advertises them for this profile.
- The setup command completed with 1.9.3 and verified installed imports with
  `python -I`. It now uses a regular package install, avoiding editable `.pth`
  hooks. Local diagnosis found macOS's hidden-file flag caused Python to skip
  the generated editable hook; the ordinary install passed outside-checkout tests.

The existing IKPy fixed-link-mask warning remains. No robot was connected, enabled
or moved for these checks. Earlier results below retain the SDK versions actually
tested at the time; they are not evidence of 1.9.3 controller behavior.

## 2026-11-20 — independent examples and installation paths

Removed `Examples/_run.py` and `Examples/_common.py`. Each robot example now owns
its argument parsing, connection, initialization, control calls and cleanup. The
servo examples show their timed loops and directly call `servo_*` APIs. All pose
inputs, outputs, teaching records and replay waypoints remain plain wxyz lists.

Verified on macOS arm64 / Python 3.12.13 with official RDK 1.9.4.1:

- **179 tests passed**; Ruff and `git diff --check` passed. The existing IKPy
  fixed-link-mask FutureWarning remains.
- All 20 examples start with `--help` as installed modules, direct files and copied
  standalone files from outside the checkout, with `PYTHONPATH` removed.
- Both independent servo loops stop on injected fault, disconnect, excessive command
  lag and Ctrl+C; measured-force aborts and exact wxyz teaching/replay still pass.
- `Install/setup.sh` installed a fresh project copy in a path containing spaces,
  launched from another directory, and successfully reused its created environment.
- A separate environment installed a regular wheel, without editable/source-path
  configuration: all **60 example entry-point checks passed**. Installed imports
  resolved to that environment's `site-packages`; built-in defaults and the packaged
  Rizon 4 URDF loaded, and offline FK returned a finite transform.
- A clean wheel contains neither deleted helper module. CI now installs a regular
  distribution and runs the outside-checkout example and URDF checks.

These are installation, interface and offline kinematic checks. No controller
connection or robot motion was attempted, and they do not resolve the simulation
controller prerequisite described below.

## Historical migration results

The results below describe the initial SDK migration (0.2.0). The subsequent
0.2.1 update removes the Pose class and uses raw wxyz vectors throughout; see
the added regression results below.

## Results

| Environment | SDK | Checks | Result |
| --- | --- | --- | --- |
| macOS arm64, Python 3.12.13 | official `flexivrdk==1.9.4.1` macOS wheel | `python -m pytest -q` | 64 passed, 2.57s |
| 5090-02, Linux x86_64, glibc 2.39, Python 3.12.14 | official 1.9.4.1 manylinux wheel | `python -m pytest -q` | 64 passed, 6.30s |
| Both environments | same source | `ruff check Robotics_API Examples tests` | passed |
| macOS | ordinary built wheel | imports from outside repository, packaged Rizon 4 URDF, direct example `--help` | passed |
| Working tree | source | `git diff --check` | passed |

Both runs emitted one IKPy 4.5.0 FutureWarning about boolean fixed-link masks.
This is not a control/IK failure; the existing mask remains compatible with IKPy 3.x.
Dependencies used: numpy 2.5.5, scipy 1.18.1, ikpy 4.5.0, pytest 9.2.0.
GitHub Actions adds Ubuntu 22.04 / Python 3.10 and 3.12 jobs; those jobs are separate
from the local and 5090-02 results above.

Tests cover:

- Existence of every wrapped Robot method in the real installed binary.
- Actual pybind signatures for joint/Cartesian commands and force/impedance configuration.
- Real RobotStates, RobotInfo and Tool attribute availability.
- Initial SDK boundary conversion and quaternion sign equivalence (superseded
  by direct wxyz transfer in 0.2.1).
- Force-axis/frame configuration, IDLE-only passive-mode setup, velocity protection,
  moment/contact-regulation incompatibility and clearing force state on mode changes.
- Correct joint command arity, all-seven-joint bounds, velocity bounds and NaN rejection.
- Impedance limits, nominal gain restore and null-space commands.
- Fault, transport error, initialization/primitive/plan timeout and motion-wait stopping.
- Position plus orientation/velocity convergence, instead of position-only success.
- Rizon 4 FK/IK roundtrip with a rotated and translated tool TCP.
- Tool parameter dispatch through the real SDK ToolParams data structure.
- All 17 maintained examples' `--help`, without ROS, peripherals or a live robot.
- Bounded force example execution, measured force abort before sending, invalid CLI
  rejection before connection, and no automatic Home/ZeroFTSensor operation.

The command recorder is deliberately not called a simulator: it validates interface
semantics and command ordering, not mechanical response. No live Robot constructor
was used to connect to hardware during these checks.

## 5090-02 simulation coverage

Read-only inspection found:

- DJ-Sim revision `420d8c5513b5b76dc55235e99677e1c6a82efe4a` uses Isaac/PhysX articulation
  controllers and direct joint gain/effort commands for its compliance behavior.
- DJ-demo revision `e8029ddbe0da339fcd1b946e8682bb9174865850` has a `FlexivRobot` subclass
  without a complete Flexiv RDK implementation.
- The existing `dj-sim` Python environment had no `flexivrdk` module.
- No matching Flexiv RDK server or `SetForceControlAxis`/`SendCartesianMotionForce`
  implementation was found in the inspected simulation modules.

Therefore this environment cannot validate the Rizon controller's active force loop,
contact search, sensor zeroing, real tool compensation, network timing or 1kHz RT API.
No Isaac physics run was presented as verification of these features.

Linux checks ran in the independent directory
`/root/Users/Alfred/code/BestMan_Flexiv_rdk194_validation_20261116`, with a separate
`.venv`. Official wheels were transferred for offline installation because the host's
public PyPI download was slow. DJ-Sim/DJ-demo source and environments were not modified.
No robot was enabled or commanded.

For that historical 1.9.4.1 profile, controller 3.11.3/3.11.4 is required. Verify the actual serial number, active tool
parameters and WORLD/base calibration. Start with read-only states and a checked
workspace, then perform contact-free zeroing and a bounded low-force test. Actual
force tracking, overshoot, contact loss and stop response remain hardware checks.

## 0.2.1 — plain wxyz vectors throughout

The Pose class is removed. Every pose input/output, teaching record, replay waypoint,
force-control frame transform and tool TCP is `[x,y,z,qw,qx,qy,qz]`. FK/IK and TCP
rotation use SciPy's explicit `scalar_first=True`; no boundary reordering is needed.

- macOS / Python 3.12.13: **70 passed**, 2.62s.
- 5090-02 isolated Linux / Python 3.12.14: **70 passed**, 7.20s.
- Ruff and `git diff --check`: passed.
- The same IKPy fixed-link-mask FutureWarning remains; no new warnings.

New/updated regressions exercise nontrivial wxyz quaternions through commands,
state reads, force frames and tool parameters; list/tuple/NumPy input without
mutation; copied list outputs; malformed/zero/nonunit quaternion rejection;
known-axis TCP rotation; independent FK transform comparison; and teaching JSON
save followed by replay with exactly the original SDK-order values.

These are interface and offline kinematic tests. The hardware/simulation limitations
above are unchanged.

## 0.3.0 — servo helpers and Python SDK alignment

- macOS / Python 3.12.13: **130 passed**, 3.00s.
- 5090-02 isolated Linux / Python 3.12.14: **130 passed**, 8.47s.
- Ruff, documentation link checks and `git diff --check`: passed.
- The same IKPy fixed-link-mask FutureWarning remains.

Both official 1.9.4.1 wheels expose the documented 53 Robot members. Tests account
for 49 wrapped members and the four deliberate exclusions, and confirm that neither
wheel exposes RT Stream* methods, RT modes, Scheduler, Robot.ServoOn or Robot.Disable.
New command signatures and the native NrtCartesianCmd constructor are checked against
the actual binary. Native service construction is tested with injected factories;
service operations are not exercised on a controller.

Added regressions cover bounded servo scheduling, skipped missed periods without
command bursts, mode reuse, wxyz/velocity/wrench forwarding, connection loss,
callback errors and Ctrl+C, lag detection, and stopping on invalid servo targets even
without the loop helper. They also cover the new servo/status examples, measured
force aborts, multi-waypoint mode/shape/axis checks, asynchronous primitive states,
plan options/pause/breakpoints, IO bounds, inertia/contact-torque limits, recovery-state
requirements, native service arguments, and tool/work-coordinate management.

No robot was enabled or moved. These results validate API contracts, control flow and
offline behavior, not actual servo tracking, real-time timing, stopping distance,
force tracking, physical IO, recovery, calibration or safety-controller operations.
The existing DJ-Sim/DJ-demo environments were not modified.
