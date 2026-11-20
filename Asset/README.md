# Offline kinematic models

`flexiv_rizon4_kinematics.urdf` is the existing Rizon 4 model. Its blurred inertial
parameters are not calibrated dynamics. `flexiv_rizon4r_kinematics.urdf` is the
separate Rizon 4R model; selecting the 4R model does not mirror or reorder commands.
Both describe the arm's base-to-flange transform. The configured wxyz tool transform
is composed afterward. No mounting/world transform is inferred.

## Rizon 4R source

Derived from [Flexiv's official robot descriptions](https://github.com/flexivrobotics/flexiv_description)
at commit `2fc4f62329fa95e0280b7a1e45c4392397b401fb`:

- `config/Rizon4R/default_kinematics.yaml`: joint origins, axes and flange transform.
- `config/Rizon4R/joint_limits.yaml`: joint limits, velocity and effort fields.
- `urdf/common/flexiv_common.xacro`: unspecified axes default to `0 0 1`.

The derived file retains the seven revolute joints and fixed flange joint with
unprefixed names. It omits the optional mounting joint, meshes, collision geometry,
FT sensor and inertial parameters. Its position bounds use the upstream **soft**
limits for offline IK. Live motion uses limits read from the actual controller.
These are the modifications made for BestMan's standalone kinematics package.

The source is Apache-2.0; its license is included in `LICENSE.flexiv_description`.
The normalized upstream values and SHA256 hashes are retained in
`tests/fixtures/rizon4r_reference.json` for independent transform checks. Tests do
not need ROS, xacro, network access or a simulator.
