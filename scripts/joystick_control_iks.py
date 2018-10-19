#!/usr/bin/env python
#
# Oscar E. Ramos
# Department of Mechatronics Engineering
# Universidad de Ingenieria y Tecnologia - UTEC
# Lima, Peru
#
# Demo of a joystick with Sawyer controlling position and keeping the end
# effector pointing downwards.
#
# To execute with the real robot:
#    rosrun joy joy_node
#    rosrun sawyer_utec joystick_control_iks.py
#
# To execute in simulation: launch the simulator at the very beginning
#    roslaunch sawyer_utec gz_sawyer_empty.launch
#    rosrun joy joy_node
#    rosrun sawyer_utec joystick_control_iks.py --mode sim
#

import numpy as np
import time
from optparse import OptionParser
import rospy
from geometry_msgs.msg import PoseStamped

import intera_interface
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from utils import *


class JointResult:
    jangles = {}
    valid = False


def pose_to_dict_message(pose):
    """
    Create a PoseStamped message using the given position and orientation

    Arguments:

      pose - (x, y, z, ew, ex, ey, ez)

    Returns:

      Dictionary containing the PoseStamped message
      
    """
    # Stamped pose message
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = 'base'
    pose_msg.pose.position.x = pose[0]
    pose_msg.pose.position.y = pose[1]
    pose_msg.pose.position.z = pose[2]
    pose_msg.pose.orientation.w = pose[3]
    pose_msg.pose.orientation.x = pose[4]
    pose_msg.pose.orientation.y = pose[5]
    pose_msg.pose.orientation.z = pose[6]
    # Dictionary containing the pose message
    pose = {'right': pose_msg}
    return pose


def get_ik(pose, q_initial, verbose=True):
    """
    Get the inverse kinematics of the robot for a given pose

    Arguments:

      pose - desired pose in the format (x,y,z,ew,ex,ey,ez), containing
             the position and orientation (quaternion)
      q_initial - list of initial joint configuration used as initial point
                  when computing the inverse kinematics

    Returns:

      result - structure containing the joint angles from inverse kinematics and an
               indication of whether the values are valid

    """
    pose_msg = pose_to_dict_message(pose)
    limb = "right"
    # Structure that the function returns
    result = JointResult()
    # Service name
    serv_name = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    # Client for the inverse kinematics server
    ik_client = rospy.ServiceProxy(serv_name, SolvePositionIK)
    # Message for the request
    ik_request = SolvePositionIKRequest()
    # Add desired pose for inverse kinematics
    ik_request.pose_stamp.append(pose_msg[limb])
    # Request inverse kinematics from base to "right_hand" link
    ik_request.tip_names.append('right_hand')

    # Start the IK optimization from this joint configuration (seed)
    ik_request.seed_mode = ik_request.SEED_USER
    seed = JointState()
    seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                 'right_j4', 'right_j5', 'right_j6']
    seed.position = [q_initial['right_j0'], q_initial['right_j1'],
                     q_initial['right_j2'], q_initial['right_j3'],
                     q_initial['right_j4'], q_initial['right_j5'],
                     q_initial['right_j6']]
    ik_request.seed_angles.append(seed)

    try:
        # Block until the service is available
        rospy.wait_for_service(serv_name, 3.0)
        # Service request
        ik_response = ik_client(ik_request)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return result

    # Check if result is valid
    if (ik_response.result_type[0] > 0):
        seed_str = {
            ik_request.SEED_USER: 'User Provided Seed',
            ik_request.SEED_CURRENT: 'Current Joint Angles',
            ik_request.SEED_NS_MAP: 'Nullspace Setpoints',
        }.get(ik_response.result_type[0], 'None')
        if (verbose):
            rospy.loginfo("Valid joints from seed: %s" % (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(ik_response.joints[0].name,
                               ik_response.joints[0].position))
        if (verbose):
            rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
    else:
        rospy.logerr("INVALID POSE - No valid joint solution found.")
        rospy.logerr("Result Error %d", ik_response.result_type[0])
        return result
    # Return the joint configuration
    result.jangles = limb_joints
    result.valid = True
    return result


if __name__ == "__main__":

    parser = OptionParser()
    parser.add_option("--mode", dest="mode", default="real",
                      help="mode can be sim (simulated) or real (default)")
    (options, args) = parser.parse_args()
    mode = options.mode

    rospy.init_node("JoystickControl")
    # Instance for interface with Sawyer
    robot = SawyerRobot(mode=mode)
    # Joystick
    joystick = JoystickInterface()
    time.sleep(0.5)

    # Move to initial position
    if (mode=="real"):
        limb = intera_interface.Limb("right")
        limb.move_to_neutral()
    # Gripper
    print 'calibrating gripper ...'
    gripper = intera_interface.Gripper('right_gripper')
    gripper.calibrate()
    # previous value for the gripper
    jgripper_old = 0
   
    # Get initial joint configuration
    q = robot.get_joint_state()
    # Get initial pose for the end effector
    T = robot.wrist_pose()
    robot.wrist_frame(T)
    # Get the initial pose
    x = TF2xyzquat(T)
    # Initial desired pose and its copy (its initial previous value)
    xdes = copy(x)
    xdes_previous = copy(xdes)
    # xdes[1] = xdes[1]+0.05
    robot.wrist_frame_d(xdes)
    # Loop rate (in Hz)
    freq = 30
    rate = rospy.Rate(freq)
    print 'ready to start motion ...'
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
            # Move up/down
            dx = 0; dy = 0
            dz = -joystick_axis[1]
        # Gripper
        jgripper = joystick_axis[3]
            
        # Gains for the joystick motion
        kx = 0.02
        ky = 0.02
        kz = 0.02
        # Update the desired position
        xdes[0] += kx*dx
        xdes[1] += ky*dy
        xdes[2] += kz*dz

        # Stop the motion
        if (joystick_button[1]==1):
            print 'stopping the motion'
            T = robot.wrist_pose()
            xdes[0] = T[0,3]
            xdes[1] = T[1,3]
            xdes[2] = T[2,3]
        
        # Verify the workspace
        x0=0.0; y0=0.0; z0=0.317
        rdes = (xdes[0]-x0)**2 + (xdes[1]-y0)**2 + (xdes[2]-z0)**2
        #rmax = 1.04  # Considering only position (1.02^2)
        rmax = 0.7744  # Considering pose (0.88^2)
        if (rdes > rmax):
            xdes[0] -= kx*dx
            xdes[1] -= ky*dy
            xdes[2] -= kz*dz

        # Show desired pose and current pose
        robot.wrist_frame_d(xdes)
        robot.wrist_frame()
       
        # Update the desired position
        print 'xdes delta: ', np.linalg.norm(xdes_previous-xdes)
        if (np.linalg.norm(xdes_previous-xdes)>1e-6):
            qdict = q_to_dict(q)
            result = get_ik(xdes, qdict, verbose=True)
            if (result.valid):
                q = q_from_dict(result.jangles)
        if (False):
            print 'desired:', xdes
            print 'invkine:', robot._fkine(q)[0:3,3]
        # Verify joint limits
        for k in xrange(7):
            if (q[k] < robot.qmin[k]):
                q[k] = robot.qmin[k]
            if (q[k] > robot.qmax[k]):
                q[k] = robot.qmax[k]

        # Publish the message to the robot
        robot.set_joints(q)
        # The current xdes becomes the previous xdes
        xdes_previous = copy(xdes)

        # Open/close the gripper
        if (jgripper == jgripper_old):
            pass
        else:
            vgripper = 0.5*gripper.MAX_POSITION*(jgripper+1.0)
            print 'gripper:', vgripper
            gripper.set_position(vgripper)
        jgripper_old = jgripper
        
        rate.sleep()

