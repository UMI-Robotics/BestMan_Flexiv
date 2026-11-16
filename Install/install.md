# Rizon 4 installation

Use Python 3.12 (or 3.10) on Ubuntu 22.04+, Windows x86_64, or macOS 12+ arm64.
The RDK 1.9.4.1 release also ships Python 3.14 Linux wheels, but this repository's
CI targets 3.10/3.12. Do not use the old Python 3.8 environment.

```bash
git clone https://github.com/UMI-Robotics/BestMan_Flexiv.git
cd BestMan_Flexiv
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install '.[dev]'
python -c "from importlib.metadata import version; import flexivrdk; print(version('flexivrdk'))"
python -m pytest -q
```

Expected SDK: **1.9.4.1**. RDK 2.1 is for Enlight and is not compatible with Rizon 4.
The connected Rizon controller must run Flexiv software **3.11.3 or 3.11.4** and
have a valid RDK license. Check controller software and remote mode in Elements.
The SDK enforces robot/software compatibility at connection time.

Conda alternative: `conda env create -f Install/basic_env_py312.yaml`, then
`conda activate bestman-flexiv` and `python -m pip install '.[dev]'` from the repository root.

Optional extras:

- `python -m pip install '.[gripper]'`: serial Robotiq gripper. Set the exact serial port.
- `python -m pip install '.[camera]'`: original RealSense camera module. Wheel availability depends on OS/Python.

Old `Install/flexiv_rdk` vendor binaries are removed in favor of the pinned official wheel.
Remove old references to `Install/flexiv_rdk/lib_py` from `PYTHONPATH` in shell startup files.
`load_flexivrdk.sh` now installs the pinned wheel; `pythonpath.sh` installs this package
instead of editing `.bashrc`. ROS is optional for historical applications and is not
required by Robotics_API or the maintained Examples.

Connection uses the serial number, e.g. `Rizon4-123456`. The optional `--interface`
parameter is a local NIC IPv4 address, not the robot IP. Python NRT command frequency
is limited by this wrapper to 1–100 Hz, following the official Python examples.

Run from the repository root with `python -m Examples.display_states Rizon4-123456`.
The read-only example does not enable or stop the robot. Motion examples own the
connection and always stop before exit. Do not run multiple motion owners concurrently.
