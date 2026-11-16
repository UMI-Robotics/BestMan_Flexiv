#!/usr/bin/env bash
set -euo pipefail
# Pin the last Rizon-compatible release; unpinned pip installs select Enlight RDK 2.x.
python -m pip install 'flexivrdk==1.9.4.1'
