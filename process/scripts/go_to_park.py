# go_to_park.py
"""Move robot to park position (safe storage position)."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import compas_rrc as rrc
import _skills.custom_motion as cm
from globals import ROBOT_NAME, TOOL_GRIPPER
from joint_positions import jp_park


def go_to_park():
    ros = rrc.RosClient()
    ros.run()

    r1 = rrc.AbbClient(ros, ROBOT_NAME)
    print("Connected.")

    r1.send(rrc.SetTool(TOOL_GRIPPER))

    print("Moving to park position...")
    r1.send_and_wait(cm.MoveToJoints(jp_park.robax, jp_park.extax, 1, rrc.Zone.FINE))
    print("At park position.")

    ros.close()
    ros.terminate()


if __name__ == "__main__":
    go_to_park()
