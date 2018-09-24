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

import intera_interface

newton = False


if __name__ == "__main__":

    rospy.init_node("JoystickControl")
    # Instance for interface with Sawyer
    robot = SawyerRobot(mode="sim")
    # Joystick
    joystick = JoystickInterface()
    time.sleep(0.5)
    # Gripper
    gripper = intera_interface.Gripper('right_gripper')
    gripper.calibrate()
    
    # Get initial pose for the end effector
    T = robot.wrist_pose()
    # Get the initial position
    if (newton):
        robot.wrist_ball(T)
        x = T[0:3,3]
        # Initial position
        xdes = copy(x)
        xdes[1] = xdes[1]+0.20
        robot.wrist_ball_d(xdes)

    robot.wrist_frame(T)
    x = TF2xyzquat(T)
    #initial pose
    xdes = copy(x)
    # xdes[1] = xdes[1]+0.20
    robot.wrist_frame_d(xdes)

    # Initial joint configuration
    q0  = robot.get_joint_state()
    q = copy(q0)
    # Initialize the derror vector (derivative of the error)                          
    derror = np.zeros(7)

    # previous value for the gripper
    jgripper_old = 0
    
    # Loop rate (in Hz)
    freq = 100
    dt = 1.0/freq
    rate = rospy.Rate(freq)
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
            dx = 0; dy = 0
            dz = joystick_axis[1]
        # Gripper
        jgripper = joystick_axis[3]
            
        # Update the desired position
        kx = 0.1
        ky = 0.1
        kz = 0.1
        xdes[0] += kx*dx
        xdes[1] += ky*dy
        xdes[2] += kz*dz

        # Verify the workspace
        x0=0.0; y0=0.0; z0=0.317
        rdes = (xdes[0]-x0)**2 + (xdes[1]-y0)**2 + (xdes[2]-z0)**2
        # rmax = 1.04  # Considering only position (1.02^2)
        rmax = 0.7744  # Considering pose (0.88^2)
        if (rdes > rmax):
            xdes[0] -= kx*dx
            xdes[1] -= ky*dy
            xdes[2] -= kz*dz

        # Show desired position and current position
        robot.wrist_frame_d(xdes)
        
        # Compute inverse kinematics using Newton's method
        if (newton):
            # Show desired position and current position
            robot.wrist_ball_d(xdes)
            robot.wrist_ball()
            # Update the desired position
            q = robot.wrist_ikine_newton(xdes)
            if (False):
                print 'desired:', xdes
                print 'invkine:', robot._fkine(q)[0:3,3]

        # Differential kinematics
        k = 1.0
        T = robot.wrist_pose()
        robot.wrist_frame(T)
        x = TF2xyzquat(T)
        print x, xdes
        derror[0:3] = -k*( x[0:3] - xdes[0:3] )
        derror[3]  = k*( xdes[3]*x[3] + np.dot(xdes[4:],x[4:]) - 1.0 )
        derror[4:] = k*( x[3]*xdes[4:] - xdes[3]*x[4:] - np.dot(skew(xdes[4:]),x[4:]) )

        J = robot.wrist_jacobian(q)
        if (np.linalg.matrix_rank(J, 0.01)<7):
            v = 0.01
            pinv = np.dot(J.transpose(),
                          np.linalg.inv(J.dot(J.transpose())+v*np.identity(7)))
        else:
            pinv = np.linalg.pinv(J)
        dq = np.dot(pinv, derror)
        q = q + dq*dt
                
        # Verify joint limits
        for k in xrange(7):
            if (q[k] < robot.qmin[k]):
                q[k] = robot.qmin[k]
            if (q[k] > robot.qmax[k]):
                q[k] = robot.qmax[k]
        
        # Publish the message to the robot
        robot.set_joints(q)
        # Open/close the gripper
        if (jgripper == jgripper_old):
            pass
        else:
            vgripper = 0.5*gripper.MAX_POSITION*(jgripper+1.0)
            gripper.set_position(vgripper)
        jgripper_old = jgripper
            
        rate.sleep()

