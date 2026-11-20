#!/usr/bin/env bash
# Install from any working directory. Never edit shell profiles or PYTHONPATH.
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin=""
extras=""
while (( $# )); do
    case "$1" in
        --python)
            [[ $# -ge 2 ]] || { echo "--python requires an interpreter" >&2; exit 2; }
            python_bin="$2"
            shift 2
            ;;
        --extras)
            [[ $# -ge 2 ]] || { echo "--extras requires dev, gripper, camera or a comma-separated list" >&2; exit 2; }
            extras="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: bash Install/setup.sh [--python python3.12] [--extras dev,gripper,camera]"
            exit 0
            ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done

if [[ -n "$extras" && ! "$extras" =~ ^(dev|gripper|camera)(,(dev|gripper|camera))*$ ]]; then
    echo "Supported extras: dev, gripper, camera" >&2
    exit 2
fi

if [[ -x "$project_dir/.venv/bin/python" ]]; then
    if [[ -n "$python_bin" ]]; then
        echo "Using the existing .venv; --python applies only when creating a new environment."
    fi
    python_bin="$project_dir/.venv/bin/python"
elif [[ -z "$python_bin" ]]; then
    for candidate in python3.12 python3.10 python3; do
        if command -v "$candidate" >/dev/null 2>&1 && \
            "$candidate" -c 'import sys; sys.exit(sys.version_info[:2] not in ((3, 10), (3, 12)))'; then
            python_bin="$candidate"
            break
        fi
    done
fi

if [[ -z "$python_bin" ]]; then
    echo "Install Python 3.12, then rerun this command (or pass --python python3.12)." >&2
    exit 1
fi
"$python_bin" -c 'import sys; assert sys.version_info[:2] in ((3, 10), (3, 12)), "Use Python 3.10 or 3.12; RDK 1.9.4.1 has no Python 3.11 wheel"'
if [[ ! -x "$project_dir/.venv/bin/python" ]]; then
    "$python_bin" -m venv "$project_dir/.venv"
fi
venv_python="$project_dir/.venv/bin/python"
if ! "$venv_python" -m pip --version >/dev/null 2>&1; then
    "$venv_python" -m ensurepip --upgrade
fi
install_target="$project_dir"
if [[ -n "$extras" ]]; then
    install_target="$install_target[$extras]"
fi
"$venv_python" -m pip install -e "$install_target"
"$venv_python" -c 'from importlib.metadata import version; from Robotics_API import Bestman_Real_Flexiv; import flexivrdk; assert version("flexivrdk") == "1.9.4.1"; print("Ready: BestMan Flexiv / RDK 1.9.4.1")'
printf '\nRun an independent example:\n  "%s" "%s" Rizon4-123456\n' \
    "$venv_python" "$project_dir/Examples/display_states.py"
