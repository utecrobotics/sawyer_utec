#!/usr/bin/env python
#
# Oscar E. Ramos
# Department of Mechatronics Engineering
# Universidad de Ingenieria y Tecnologia - UTEC
# Lima, Peru
#

import numpy as np
import time

import rospy
from utils import *


if __name__ == "__main__":

    rospy.init_node("JoystickControl")
    # Instance for interface with Sawyer
    robot = SawyerRobot()
    # Joystick
    joystick = JoystickInterface()
    time.sleep(0.5)

    # Get initial pose for the end effector
    T = robot.pose_wrist()
    # Get the initial position
    x = T[0:3,3]
    # Initial position
    xdes = copy(x)
   
    # Loop rate (in Hz)
    rate = rospy.Rate(30)
    # Continuous execution loop
    while not rospy.is_shutdown():
        # Joystick values
        joystick_axis   = joystick.get_values().axes
        joystick_button = joystick.get_values().buttons
        # Forward/backward
        if (joystick_button[0] == 0):
            dx = joystick_axis[1]
            dy = joystick_axis[0]
            dz = 0
        else:
            dx = 0
            dy = 0
            dz = joystick_axis[1]
            
        # Update the desired position
        kx = 0.1
        ky = 0.1
        kz = 0.1
        xdes[0] += kx*dx
        xdes[1] += ky*dy
        xdes[2] += kz*dz

        print xdes
        
        # Update the desired position
        q  = robot.ikine_wrist(xdes)
        # Verify joint limits
        for k in xrange(7):
            if (q[k] < robot.qmin[k]):
                q[k] = robot.qmin[k]
            if (q[k] > robot.qmax[k]):
                q[k] = robot.qmax[k]
        
        # Verify the workspace

        # Publish the message to the robot
        robot.set_joints(q)
        
        rate.sleep()

