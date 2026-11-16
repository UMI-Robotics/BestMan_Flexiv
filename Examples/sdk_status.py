"""Read Python RDK status, actions, IO and plans without enabling or moving the robot."""

import json

from Examples._common import parser, robot_session


def main():
    args = parser(__doc__).parse_args()
    with robot_session(args, enable=False) as arm:
        print(json.dumps(arm.get_status(), default=str, indent=2))
        print("Plans:", arm.get_plan_list())
        print("Digital inputs (0-17):", arm.get_digital_inputs())
        print("Events:", arm.get_event_log())
        print("Target TCP [x,y,z,qw,qx,qy,qz]:", arm.get_robot_actions().tcp_pose_d)


if __name__ == "__main__":
    main()
