#!/usr/bin/env python3
"""Display RealSense RGBD without moving or enabling the robot."""

import argparse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-id", required=True, help="RealSense serial number")
    args = parser.parse_args()
    from Sensor import Camera

    camera = Camera(device_id=args.device_id)
    camera.display("rgbd")


if __name__ == "__main__":
    main()
