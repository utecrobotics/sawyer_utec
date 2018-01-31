#!/usr/bin/python
#
# Move the robot to its initial position and open the gripper
#
# To execute:
#     rosrun intera_interface enable_robot.py -e
#     rosrun sawyer_utec goto_initial.py
#     rosrun intera_interface enable_robot.py -d
# 

import rospy
import intera_interface

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from utils import *
import numpy as np
import time


def main():
    # Initialize the node
    rospy.init_node('GoToInitial')

    # Initialize interfaces
    limb = intera_interface.Limb('right')
    try:
        gripper = intera_interface.Gripper('right')
    except ValueError:
        rospy.logerr("Could not detect a gripper")
        return
    
    # Move arm to the initial position
    limb.move_to_neutral()
    # Open the gripper
    gripper.open()


if __name__ == '__main__':
    main()
