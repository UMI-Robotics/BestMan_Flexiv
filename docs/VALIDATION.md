# Validation — 2026-11-16

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
- Explicit xyzw ↔ wxyz conversion and quaternion sign equivalence.
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
