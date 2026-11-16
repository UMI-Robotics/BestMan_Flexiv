#!/usr/bin/env python3
"""move eef to pose for Rizon 4 / RDK 1.9.4.1. All pose inputs/outputs/records: [x,y,z,qw,qx,qy,qz] (wxyz). Use --help for arguments."""

from Examples._run import main

if __name__ == "__main__":
    main("move_eef_to_pose")
