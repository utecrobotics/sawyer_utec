#!/usr/bin/python
#
# Oscar E. Ramos
# Department of Mechatronics Engineering
# Universidad de Ingenieria y Tecnologia - UTEC
# Lima, Peru
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


def main():
    # Initialize the node
    rospy.init_node('GoToInitial')

    # Initialize limb interface
    limb = intera_interface.Limb('right')
    # Move arm to the initial position
    limb.move_to_neutral()
    # Initialize gripper interface
    try:
        gripper = intera_interface.Gripper('right_gripper')
    except ValueError:
        rospy.logerr("Could not detect a gripper")
        return
    # Open the gripper
    gripper.open()


if __name__ == '__main__':
    main()
