#!/usr/bin/python
#
# Move Robot in the Cartesian spacec
#
# To execute:
#     rosrun intera_interface enable_robot.py -e
#     rosrun sawyer_utec move_cartesian.py
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


class result:
    jangles = {}
    valid = False


def pose_to_dict_message(position, orientation):
    """
    Create a PoseStamped message using the given position and orientation

    Arguments:

      position - tuple (x, y, z)
      orientation - quaternion in tuple (ew, ex, ey, ez)

    Returns:

      Dictionary containing the PoseStamped message
      
    """
    # Stamped pose message
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = 'base'
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    pose_msg.pose.orientation.w = orientation[0]
    pose_msg.pose.orientation.x = orientation[1]
    pose_msg.pose.orientation.y = orientation[2]
    pose_msg.pose.orientation.z = orientation[3]
    # Dictionary containing the pose message
    pose = {'right': pose_msg}
    return pose


def get_ik(pose, q_initial):
    """
    Get the inverse kinematics of the robot for a given pose

      pose - desired pose in the format ((x,y,z), (ew,ex,ey,ez)), containing
             the position and orientation (quaternion)
      q_initial - list of initial joint configuration used as initial point
                  when computing the inverse kinematics

    """
    pose_msg = pose_to_dict_message(pose[0], pose[1])
    limb = "right"
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
    seed.position = [q_initial['right_j0'], q_initial['right_j1'], q_initial['right_j2'],
                     q_initial['right_j3'], q_initial['right_j4'], q_initial['right_j5'],
                     q_initial['right_j6']]
    ik_request.seed_angles.append(seed)

    try:
        # Block until the service is available
        rospy.wait_for_service(serv_name, 5.0)
        # Service request
        ik_response = ik_client(ik_request)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return ({}, False)

    # Check if result is valid
    if (ik_response.result_type[0] > 0):
        seed_str = {
            ik_request.SEED_USER: 'User Provided Seed',
            ik_request.SEED_CURRENT: 'Current Joint Angles',
            ik_request.SEED_NS_MAP: 'Nullspace Setpoints',
        }.get(ik_response.result_type[0], 'None')
        rospy.loginfo("Valid joints from seed: %s" % (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(ik_response.joints[0].name,
                               ik_response.joints[0].position))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
    else:
        rospy.logerr("INVALID POSE - No valid joint solution found.")
        rospy.logerr("Result Error %d", ik_response.result_type[0])
        return ({}, False)
    # Return the joint configuration
    return (limb_joints, True)


def main():
    # Initialize the node
    rospy.init_node('MoveCartesian')

    # Initialize interfaces
    limb = intera_interface.Limb('right')
    try:
        gripper = intera_interface.Gripper('right')
        gripper.calibrate()
        nsteps = 5.0 # Increase it for a finer motion
        dgripper = gripper.MAX_POSITION / nsteps
    except ValueError:
        rospy.logerr("Could not detect a gripper")
        return
    
    # Move arm to initial position
    limb.move_to_neutral()
    jangles_neutral = limb.joint_angles()
    print jangles_neutral

    # Object 1
    # --------
    # Open the gripper
    gripper.open()
    # gripper.set_position(0*dgripper) # closed
    # gripper.set_position(5*dgripper) # open
    # Pre-grasp object 1
    #quat = quaternionFromAxisAngle(90.0, (1.0, 0.0, 0.0))
    quat = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose = ((0.65, 0.20, 0.20), quat)
    jangles = get_ik(pose, jangles_neutral)
    if (jangles[1]):
        limb.move_to_joint_positions(jangles[0])
    # Grasp object 1
    # quat = rotationToQuaternion(
    #     np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]) )
    pose = ((0.65, 0.20, 0.0), quat)
    jangles = get_ik(pose, jangles[0])
    if (jangles[1]):
        limb.move_to_joint_positions(jangles[0])
    # Close the gripper
    gripper.set_position(1*dgripper)
    time.sleep(1.0)
    # Intermediate 1 (up)
    pose = ((0.65, 0.20, 0.20), quat)
    jangles = get_ik(pose, jangles[0])
    if (jangles[1]):
        limb.move_to_joint_positions(jangles[0])
    # Intermediate 2 (move)
    pose = ((0.65, -0.30, 0.20), quat)
    jangles = get_ik(pose, jangles[0])
    if (jangles[1]):
        limb.move_to_joint_positions(jangles[0])
    # Final pose (release)
    pose = ((0.65, -0.30, 0.0), quat)
    jangles = get_ik(pose, jangles[0])
    if (jangles[1]):
        limb.move_to_joint_positions(jangles[0])
    # Open the gripper
    gripper.open()
    time.sleep(1.0)
    # Release
    pose = ((0.65, -0.30, 0.20), quat)
    jangles = get_ik(pose, jangles[0])
    if (jangles[1]):
        limb.move_to_joint_positions(jangles[0])


if __name__ == '__main__':
    main()
