#!/usr/bin/env bash
set -euo pipefail
# Pin the Rizon 4/4R client to controller software 3.11.2.
rdk_version="1.9.3"
if [[ $# -gt 0 ]]; then
    case "$1" in
        --rdk-version)
            [[ $# -eq 2 && -n "$2" ]] || { echo "--rdk-version requires 1.9.3" >&2; exit 2; }
            rdk_version="$2"
            ;;
        -h|--help)
            echo "Usage: bash Install/load_flexivrdk.sh [--rdk-version 1.9.3]"
            exit 0
            ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
fi
if [[ "$rdk_version" != "1.9.3" ]]; then
    echo "This project pins RDK 1.9.3 for controller software 3.11.2." >&2
    exit 2
fi
controller_versions="3.11.2"
echo "Installing RDK $rdk_version for controller software $controller_versions."
python -m pip install "flexivrdk==$rdk_version"
