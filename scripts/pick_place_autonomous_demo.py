#!/usr/bin/python
#
# Sara Vasquez, Frank Barja, Oscar Ramos
# Department of Mechatronics Engineering
# Universidad de Ingenieria y Tecnologia - UTEC
# Lima, Peru
#
# Demo of an open loop pick and place motion using several cups
#
# To execute:
#     rosrun intera_interface enable_robot.py -e
#     rosrun sawyer_utec pick_place_demo.py
#     rosrun intera_interface enable_robot.py -d
# 

import numpy as np

import rospy
from roslib import packages
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Image

import intera_interface
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import time
from utils import *

from apriltags_ros.msg import AprilTagDetectionArray

from tf.transformations import euler_from_quaternion


class JointResult:
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

    Arguments:

      pose - desired pose in the format ((x,y,z), (ew,ex,ey,ez)), containing
             the position and orientation (quaternion)
      q_initial - list of initial joint configuration used as initial point
                  when computing the inverse kinematics

    Returns:

      result - structure containing the joint angles from inverse kinematics and an
               indication of whether the values are valid

    """
    pose_msg = pose_to_dict_message(pose[0], pose[1])
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

    # ik_request.use_nullspace_goal.append(True)

    #     # The nullspace goal can either be the full set or subset of joint angles

    # goal = JointState()

    # goal.name = ['right_j6']

    # goal.position = [0.]

    # ik_request.nullspace_goal.append(goal)
    # # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
    # # If empty, the default gain of 0.4 will be used

    # ik_request.nullspace_gain.append(0.4)

    try:
        # Block until the service is available
        rospy.wait_for_service(serv_name, 5.0)
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
        rospy.loginfo("Valid joints from seed: %s" % (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(ik_response.joints[0].name,
                               ik_response.joints[0].position))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
    else:
        rospy.logerr("INVALID POSE - No valid joint solution found.")
        rospy.logerr("Result Error %d", ik_response.result_type[0])
        return result
    # Return the joint configuration
    result.jangles = limb_joints
    result.valid = True
    return result


def pick_place_autonomous(limb, gripper, pose_initial_estimate, pose_final, gripper_opening,
               zpre_grasp, qinit, tags):
    """
    Pick and place an object

      limb - robot limb object
      gripper - robot gripper object
      pose_initial_estimate - tuple ((x,y,z),(ew,ex,ey,ez))
      pose_final - tuple ((x,y,z),(ew,ex,ey,ez))
      gripper_opening - opening of the gripper (double)
      zpre_grasp - height for the pre-grasping
      qinit - joint configuration for starting the computation of the inverse
              kinematics

    """

    # Get initial and final position/orientation
    xi = pose_initial_estimate[0][0]
    yi = pose_initial_estimate[0][1]
    zi = pose_initial_estimate[0][2]
    quat_i = pose_initial_estimate[1]
    xf = pose_final[0][0]
    yf = pose_final[0][1]
    zf = pose_final[0][2]
    quat_f = pose_final[1]

    # 
    quat_camera = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    
    # Move to top of recipients
    pose = ((xi, yi, zi+zpre_grasp), quat_camera)
    result = get_ik(pose, qinit)
    result.jangles['right_j5'] = 0.0
    if (result.valid):
        limb.move_to_joint_positions(result.jangles)

    mypose = limb.joint_angles_to_cartesian_pose(result.jangles, 'right_l5')
    [_, pitch, _] = euler_from_quaternion([mypose.orientation.x,mypose.orientation.y,mypose.orientation.z,mypose.orientation.w])

    quat_camera = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))

    # Move perpendicular
    pose = ((xi, yi, zi+zpre_grasp), quat_camera)

    result = get_ik(pose, result.jangles)
    result.jangles['right_j5'] = pitch - np.pi/2
    if (result.valid):
        limb.move_to_joint_positions(result.jangles)
    time.sleep(2)



    # Search for a tag

    pose_camera = limb.joint_angles_to_cartesian_pose(result.jangles, 'right_hand_camera')
    mytag = tags.one_tag_search()

    R_camera = quaternionToRotation([pose_camera.orientation.w, pose_camera.orientation.x, pose_camera.orientation.y, pose_camera.orientation.z])
    T = np.eye(4)
    T[0:3,0:3] = R_camera
    T[0:3,3] = [pose_camera.position.x, pose_camera.position.y, pose_camera.position.z]

    P_tag = [mytag.pose.pose.position.x, mytag.pose.pose.position.y, mytag.pose.pose.position.z,1]
    P_tag = np.array(P_tag).reshape(4,1)

    P_tag_base = np.matmul(T, P_tag)



    # Go to top
    result = get_ik(pose, result.jangles)    
    if (result.valid):
        limb.move_to_joint_positions(result.jangles)

    # Move to a pre-grasping position

    pose = ((P_tag_base[0], P_tag_base[1], zi + zpre_grasp ), quat_i)
    result = get_ik(pose, result.jangles)

    if (result.valid):
        limb.move_to_joint_positions(result.jangles)

    #rospy.sleep(20.0)

    # Move to grasp the object
    pose = ((P_tag_base[0], P_tag_base[1], zi), quat_i)
    result = get_ik(pose, result.jangles)

    if (result.valid):
        limb.move_to_joint_positions(result.jangles)


    # Close the gripper (0*dgripper [closed] to nsteps*dgripper [open])
    gripper.set_position(gripper_opening)   
    rospy.sleep(5.0)


    # Move the object up (intermediate pose)
    pose = ((P_tag_base[0], P_tag_base[1], zi+zpre_grasp), quat_i)
    result = get_ik(pose, result.jangles)
    if (result.valid):
        limb.move_to_joint_positions(result.jangles)

    # Move the object in the air
    pose = ((xf, yf, zf+zpre_grasp), quat_f)
    result = get_ik(pose, result.jangles)
    if (result.valid):
        limb.move_to_joint_positions(result.jangles)

    # Move to the final (release) pose
    pose = ((xf, yf, zf), quat_f)
    result = get_ik(pose, result.jangles)
    if (result.valid):
        limb.move_to_joint_positions(result.jangles)

    # Open the gripper
    gripper.open()
    rospy.sleep(5.0)

    # Move upwards without the object
    pose = ((xf, yf, zf+zpre_grasp), quat_f)
    result = get_ik(pose, result.jangles)
    if (result.valid):
        limb.move_to_joint_positions(result.jangles)

class tag_searcher:
    def __init__(self):
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tags)

    def tags(self, data):
        self.mytags = data

    def one_tag_search(self):
        while not self.mytags.detections:
            continue_detection = input('There is no tag read. Do you want to try again [y/n]')
            if continue_detection in ['n']:
                rospy.logerr("Could not detect tag, exiting the example.")
                return
            else:
                pass
        _number_tags =  len(self.mytags.detections)
        selection = np.random.randint(_number_tags)
        one_tag = self.mytags.detections[selection]
        return one_tag


def camera_setup(exposure = 10, gain = 60):
    _cameras = intera_interface.Cameras()
    _name_camera = 'right_hand_camera'
    if  _cameras.verify_camera_exists(_name_camera):
        _cameras.start_streaming(_name_camera)
    else:
        rospy.logerr("Could not detect the specified camera, exiting the example.")
        return

    _cameras.set_exposure(_name_camera, exposure)
    _cameras.set_gain(_name_camera, gain)




def main():
    # Initialize the node
    rospy.init_node('MoveCartesian')

    # Initialize interfaces
    limb = intera_interface.Limb('right')
    head_display = intera_interface.HeadDisplay()
    try:
        gripper = intera_interface.Gripper('right_gripper')
        gripper.calibrate()
        nsteps = 5.0 # Increase it for a finer motion
        dgripper = gripper.MAX_POSITION / nsteps
    except ValueError:
        rospy.logerr("Could not detect a gripper")
        return

    # Set the path for images
    folder = str(packages.get_pkg_dir('sawyer_utec')) + '/images/'
    # Diplay the UTEC logo in the robot head
    head_display.display_image(folder+'up1.jpg', False, 1.0)

    # Set camera and tag searcher
    camera_setup()
    tags = tag_searcher()

    
    # Move arm to the initial position
    limb.move_to_neutral()
    jangles_neutral = limb.joint_angles()
    if (False): print jangles_neutral
    # Open the gripper
    gripper.open()

    # ===================
    # Motion for object 1
    # ===================

    # PRIMER PISO
    # VASO 1 MOD
     # pose_init1 = ((0.6, 0.50, 0.03), quat_init)
     # pose_final1   = ((0.6, -0.5, 0.02), quat_final)

    # VASO 2 MOD
    # pose_initial2 = ((0.6, 0.40, 0.03), quat_init)
    # pose_final2   = ((0.6, -0.42, 0.02), quat_final)
 

    # VASO 3
    # pose_initial3 = ((0.6, 0.32, 0.03), quat_init)
    # pose_final3   = ((0.6, -0.34, 0.02), quat_final)
    
    # SEGUNDO PISO
    # VASO 4
    # pose_initial4 = ((0.7, 0.5, 0.03), quat_init)
    # pose_final4   = ((0.6, -0.46, 0.12), quat_final)

    # VASO 5
    # pose_initial5 = ((0.7, 0.4, 0.03), quat_init)
    # pose_final5   = ((0.6, -0.38, 0.12), quat_final) 

    # VASO 6
    # pose_initial = ((0.7, 0.4, 0.03), quat_init)
    # pose_final   = ((0.6, -0.38, 0.12), quat_final) 

    # Initial and final poses of the object

    quat_init = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    offset_table_z = - 0.05
    # vaso 1
    pose_initial1 = ((0.6, 0.4, 0.03 - offset_table_z), quat_init)
    quat_final = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_final1   = ((0.6, -0.5, 0.02 - offset_table_z), quat_final)
    # Gripper opening (0*dgripper [closed] to nsteps*dgripper [open])
    gripper_opening = 3.5*dgripper
    # Offset in z (from the desired position) for pre-grasping
    z_pre_grasp = 0.17
    pick_place_autonomous(limb, gripper, pose_initial1, pose_final1, gripper_opening, z_pre_grasp, jangles_neutral, tags)



    # vaso 2
    quat_init2 = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_initial2 = ((0.6, 0.40, 0.03 - offset_table_z), quat_init)
    quat_final2 = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_final2   = ((0.6, -0.42, 0.02 - offset_table_z), quat_final)
# Gripper opening (0*dgripper [closed] to nsteps*dgripper [open])
    gripper_opening = 3.5*dgripper
    # Offset in z (from the desired position) for pre-grasping
    z_pre_grasp = 0.17
    pick_place_autonomous(limb, gripper, pose_initial2, pose_final2, gripper_opening, z_pre_grasp, jangles_neutral, tags)
    

# vaso 3
    quat_init = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_initial3 = ((0.6, 0.40, 0.03 - offset_table_z), quat_init)
    quat_final = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_final3   = ((0.6, -0.34, 0.02 - offset_table_z), quat_final)
# Gripper opening (0*dgripper [closed] to nsteps*dgripper [open])
    gripper_opening = 3.5*dgripper
    # Offset in z (from the desired position) for pre-grasping
    z_pre_grasp = 0.17
    pick_place_autonomous(limb, gripper, pose_initial3, pose_final3, gripper_opening, z_pre_grasp, jangles_neutral, tags)
    
    # vaso 4
    quat_init = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_initial4 = ((0.6, 0.40, 0.03 - offset_table_z), quat_init)
    quat_final = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_final4   = ((0.6, -0.46, 0.12 - offset_table_z), quat_final)
# Gripper opening (0*dgripper [closed] to nsteps*dgripper [open])
    gripper_opening = 3.5*dgripper
    # Offset in z (from the desired position) for pre-grasping
    z_pre_grasp = 0.17
    pick_place_autonomous(limb, gripper, pose_initial4, pose_final4, gripper_opening, z_pre_grasp, jangles_neutral, tags)
    

    # vaso 5
    quat_init = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_initial5 = ((0.6, 0.40, 0.03 - offset_table_z), quat_init)
    quat_final = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_final5   = ((0.6, -0.38, 0.12 - offset_table_z), quat_final) 
# Gripper opening (0*dgripper [closed] to nsteps*dgripper [open])
    gripper_opening = 3.5*dgripper
    # Offset in z (from the desired position) for pre-grasping
    z_pre_grasp = 0.17
    pick_place_autonomous(limb, gripper, pose_initial5, pose_final5, gripper_opening, z_pre_grasp, jangles_neutral, tags)    

    #vaso 6
    quat_init = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_initial = ((0.6, 0.40, 0.03 - offset_table_z), quat_init)
    quat_final = quaternionFromAxisAngle(180.0, (0.0, 1.0, 0.0))
    pose_final   = ((0.6, -0.42, 0.22 - offset_table_z), quat_final)
    # Gripper opening (0*dgripper [closed] to nsteps*dgripper [open])
    gripper_opening = 3.5*dgripper
    # Offset in z (from the desired position) for pre-grasping
    z_pre_grasp = 0.17
    pick_place_autonomous(limb, gripper, pose_initial, pose_final, gripper_opening, z_pre_grasp, jangles_neutral, tags)
    
    limb.move_to_neutral()
    jangles_neutral = limb.joint_angles()
    if (False): print jangles_neutral

    # Display another face
    head_display.display_image(folder+'sleep1.png', False, 1.0)

if __name__ == '__main__':
    main()
