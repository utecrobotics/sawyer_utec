#!/usr/bin/python
#
# Oscar E. Ramos
# Department of Mechatronics Engineering
# Universidad de Ingenieria y Tecnologia - UTEC
# Lima, Peru
#
# Wave Sawyer's arm (adapted from the Sawyer tutorials)
#
# To execute:
#     rosrun intera_interface enable_robot.py -e
#     rosrun sawyer_utec wave_hello.py
#     rosrun intera_interface enable_robot.py -d
# 

import rospy
import intera_interface


# Initialize the node
rospy.init_node('wave_hello')
# Instance of the intera_interface's Limb class
limb = intera_interface.Limb('right')
# Current joint angles
angles = limb.joint_angles()
print 'Current joint angles:\n', angles

# Move to neutral pose
limb.move_to_neutral()

if (False):
    # Set all angles to zero
    angles['right_j0']=0.0
    angles['right_j1']=0.0
    angles['right_j2']=0.0
    angles['right_j3']=0.0
    angles['right_j4']=0.0
    angles['right_j5']=0.0
    angles['right_j6']=0.0
    print angles
    # Move the arm
    limb.move_to_joint_positions(angles)
    
# Wave the arm
wave_1 = {'right_j0': -0.4259, 'right_j1': 0.3526, 'right_j2': 0.03726,
          'right_j3': -1.3833, 'right_j4': 1.5126, 'right_j5': -0.3438,
          'right_j6': -1.5126 }
wave_2 = {'right_j0': -0.4281, 'right_j1': 0.3940, 'right_j2': -0.2609,
          'right_j3': -1.4038, 'right_j4': 1.5103, 'right_j5': -0.3806,
          'right_j6': -1.5101 }
for _move in range(3):
    limb.move_to_joint_positions(wave_1)
    rospy.sleep(0.5)
    limb.move_to_joint_positions(wave_2)
    rospy.sleep(0.5)

# Move to neutral pose
limb.move_to_neutral()
