# Rizon 4 installation

Use Python 3.12 (or 3.10) on Ubuntu 22.04+, Windows x86_64, or macOS 12+ arm64.
The repository CI targets Python 3.10 and 3.12. Python 3.8 is unsupported.

## One-command setup on Linux/macOS

After cloning or extracting the ZIP:

```bash
cd BestMan_Flexiv
bash Install/setup.sh
.venv/bin/python Examples/display_states.py Rizon4-123456
```

`setup.sh` locates the project from its own location, creates `.venv`, installs the
project as a regular package with official RDK 1.9.3, and checks installed imports
in isolated Python mode (without relying on the checkout or `PYTHONPATH`). It also
works when invoked by absolute path from another directory, including paths with spaces.
It does not change shell profiles or `PYTHONPATH`, and does not connect to a robot.
An existing `.venv` is reused. Python 3.12 is preferred; 3.10 is also accepted.
The supported RDK wheels use Python 3.10 or 3.12, so the script skips Python 3.11.
All installation paths pin **1.9.3** for controller **3.11.2**, including when upgrading
an existing environment. The optional `--rdk-version 1.9.3` flag is retained for existing
commands; other SDK versions are rejected.

Rerun setup after updating the checkout to refresh the installed package. For active
library development, the editable pip installation below is also supported.

Options:

```bash
bash Install/setup.sh --python python3.12
bash Install/setup.sh --rdk-version 1.9.3
bash Install/setup.sh --extras dev
bash Install/setup.sh --extras gripper
bash Install/setup.sh --extras dev,gripper,camera
```

The gripper extra is for serial Robotiq devices; the camera extra is for RealSense.
Camera wheel availability depends on OS/Python. These extras are unnecessary for
robot motion and force-control examples. Supply actual device serials/ports via CLI.

## Standard pip installation (including Windows)

From the project directory, in an activated Python environment:

```bash
python -m pip install -e .  # Includes flexivrdk==1.9.3
python -m Examples.display_states Rizon4-123456
```

For a new Windows environment, first run `py -3.12 -m venv .venv` and activate with
`.venv\Scripts\Activate.ps1` in PowerShell. For a new Linux/macOS environment, use
`python3.12 -m venv .venv` and `source .venv/bin/activate`.

For the existing `dj-stuff` Conda environment, use `conda activate dj-stuff` and the
pip command above from this project directory. It installs the matching SDK and
project together; a new `.venv` is unnecessary. Reinstalling the project enforces
the exact 1.9.3 dependency, even if a different SDK was previously installed.

Conda alternative: `conda env create -f Install/basic_env_py312.yaml`, then
`conda activate bestman-flexiv` and `python -m pip install -e .`.
The YAML already pins RDK 1.9.3; no path or version edits are required.
A regular non-editable `python -m pip install .` also packages the examples and Rizon 4 URDF.

## Imports, resources and example entry points

After installation, use the same environment's Python. Each example is independent:

```bash
# Direct file entry point from the checkout:
python Examples/force_control.py Rizon4-123456 --help
# Installed module entry point from any directory:
python -m Examples.force_control Rizon4-123456 --help
```

You can also copy one example elsewhere and run it with the installed environment;
it does not import another example. Every control script shows the connection,
initialization, mode/configuration, commands and stop sequence in that file.
The two servo examples contain their own timed loops and directly call `servo_*` APIs.

The default Rizon 4 profile is built into the package. Offline FK/IK resolves the
packaged URDF relative to the installed package, not the current working directory.
No config path or asset path needs editing. Optional `--config`, `--trajectory` and
`--output` paths are supplied by the caller and resolve against the current directory.
Teaching and replay use plain `[x,y,z,qw,qx,qy,qz]` lists throughout (metres, wxyz).

## Controller requirements

This project requires **RDK 1.9.3** and **controller software 3.11.2**, as documented
in the official [1.9.3 release](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v1.9.3).
The controller must have an RDK license and be reachable in Auto (Remote) mode.
The SDK checks compatibility at connection time. Keep that check enabled and install
the matching client; this fix does not require changing controller firmware.
RDK 2.x targets Enlight and is not compatible with this Rizon 4 profile.
Waypoint timing is selected by the controller. Pass `durations=None` or all zeros;
nonzero values are rejected before mode changes or commands.

To install only the SDK into the active environment:

```bash
bash Install/load_flexivrdk.sh
```

Use the actual robot serial number in place of `Rizon4-123456`. Optional `--interface`
is a local NIC IPv4 address, not the robot IP. Python NRT frequency is 1–100 Hz.

ROS and manual SDK path setup are not required. For existing legacy installations,
remove old `Install/flexiv_rdk/lib_py` entries from `PYTHONPATH`; fresh downloads need
no such changes. The older `pythonpath.sh` installs the package instead of editing
`.bashrc`, but new installations should use `setup.sh` or standard pip.

Read-only examples do not enable or stop the robot. Motion examples always stop
before exit. Run one motion owner at a time.

## Development checks

```bash
bash Install/setup.sh --extras dev
.venv/bin/python -m pytest -q
.venv/bin/ruff check Robotics_API Examples tests
```

The tests check every example as a module, a direct file and a copied file from outside
the checkout with `PYTHONPATH` removed. They do not connect to hardware.
