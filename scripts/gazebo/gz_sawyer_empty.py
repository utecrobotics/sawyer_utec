#!/usr/bin/env python
#
# Department of Mechatronics Engineering
# Universidad de Ingenieria y Tecnologia - UTEC
# Lima, Peru
#
# Set Sawyer to its initial configuration and open the gripper
#

import sys
import rospy
import intera_interface

class RobotInterface(object):
    def __init__(self):
        # Intera interface for the arm and the gripper
        self.limb_ = intera_interface.Limb('right')
        self.gripper_ = intera_interface.Gripper()
        # Verify if sawyer is enabled
        print("Getting robot state... ")
        self.rs_ = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self.init_state_ = self.rs_.state().enabled
        print("Enabling robot... ")
        self.rs_.enable()

    def move_to_initial(self):
        initial_angles = {'right_j0': -0.041662954890248294,
                          'right_j1': -1.0258291091425074,
                          'right_j2': 0.0293680414401436,
                          'right_j3': 2.17518162913313,
                          'right_j4':  -0.06703022873354225,
                          'right_j5': 0.3968371433926965,
                          'right_j6': 1.7659649178699421}
        print("Moving the arm to the initial pose...")
        # Set initial joint configuration
        self.limb_.move_to_joint_positions(initial_angles, timeout=5.0)
        # Open the gripper
        self.gripper_.open()
        rospy.sleep(1.0)

def main():
    rospy.init_node("gz_sawyer_initial")
    robot = RobotInterface()
    robot.move_to_initial()


if __name__ == '__main__':
    sys.exit(main())
