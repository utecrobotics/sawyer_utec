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
    pose_msg.frame_id = 'base'
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
    seed.position = q_initial
    ik_request.seed_angles.append(seed)

    # # After the primary IK task, the solver tries to bias the the joint angles
    # # toward the goal joint configuration (using the null space)
    # ik_request.use_nullspace_goal.append(True)
    # # Nullspace goal: full set or subset of joint angles
    # goal = JointState()
    # goal.name = ['right_j1', 'right_j2', 'right_j3']
    # goal.position = [0.1, -0.3, 0.5]
    # ik_request.nullspace_goal.append(goal)
    # # The gain to bias towards the nullspace goal.
    # ikreq.nullspace_gain.append(0.4)

    try:
        # Block until the service is available
        rospy.wait_for_service(serv_name, 5.0)
        # Service request
        ik_response = ik_client(ik_request)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

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
        return False
    # Return the joint configuration
    return limb_joints


def main():
    # Initialize the node
    rospy.init_node('MoveCartesian')

    # Initialize interfaces
    limb = intera_interface.Limb('right')
    try:
        gripper = intera_interface.Gripper('right')
        gripper.calibrate()
        gripper_maxpos = gripper.MAX_POSITION
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
    # Pre-grasp object 1
    quat = quaternionFromAxisAngle(0.0, (1.0, 0.0, 0.0))
    pose = ((x, y, z), quat)
    jangles = get_ik(pose, jangles_neutral)
    limb.move_to_joint_positions(jangles)
    # Grasp object 1
    # quat = rotationToQuaternion(
    #     np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]) )
    pose = ((x, y, z), (ew, ex, ey, ez))
    jangles = get_ik(pose, jangles)
    limb.move_to_joint_positions(jangles)
    # Close the gripper (0.0: close, 100.0: open)
    gripper.set_position(50.0)
    # Intermediate 1 (up)
    pose = ((x, y, z), (ew, ex, ey, ez))
    jangles = get_ik(pose, jangles_neutral)
    limb.move_to_joint_positions(jangles)
    # Intermediate 2 (move)
    pose = ((x, y, z), (ew, ex, ey, ez))
    jangles = get_ik(pose, jangles_neutral)
    limb.move_to_joint_positions(jangles)
    # Final pose (release)
    pose = ((x, y, z), (ew, ex, ey, ez))
    jangles = get_ik(pose, jangles_neutral)
    limb.move_to_joint_positions(jangles)
    # Open the gripper
    griper.open()


if __name__ == '__main__':
    main()
