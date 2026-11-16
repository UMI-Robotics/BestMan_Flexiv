# Validation — 2026-11-16

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

Before hardware use, verify controller 3.11.3/3.11.4, actual serial number, active tool
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
