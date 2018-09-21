#
# Oscar E. Ramos
# Department of Mechatronics Engineering
# Universidad de Ingenieria y Tecnologia - UTEC
# Lima, Peru
#

import numpy as np
from copy import copy


def quaternionFromAxisAngle(angle, axis):
    """
    Increment in pose with respect to the global frame.

    Arguments:
        angle -- double [deg]
        axis -- (x,y,z) if not unitary, it will be internally normalized

    Output:
      quaternion in the format (ew, ex, ey, ez) where ew is the scalar part

    """
    angle2 = np.deg2rad(angle)/2.0
    ax = np.array([axis]).transpose()
    ax = ax/np.linalg.norm(ax)
    Q = np.matrix([[np.cos(angle2)],
                   [ax[0][0]*np.sin(angle2)],
                   [ax[1][0]*np.sin(angle2)],
                   [ax[2][0]*np.sin(angle2)]])
    return Q


def quaternionFromRotation(R):
    """
    Convert a rotation matrix to a unitary quaternion.
    
    Arguments:
      R -- rotation matrix as a 2d numpy array

    Output:
      quaternion in the format (ew, ex, ey, ez) where ew is the scalar part

    """
    dEpsilon = 1e-6
    quat = [0., 0., 0., 0.]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if (np.abs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*sgn(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if (np.abs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*sgn(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if (np.abs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*sgn(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return quat


def quaternionMult(q1, q2):
    """
    Multiply 2 quaternions. Each quaternion format is (ew, ex, ey, ez) where
    ex is the scalar part.

    Arguments:
      q1 -- First quaternion
      q2 -- Second quaternion

    Return:
      The quaternion multiplication (q1*q2)

    """
    res = [0., 0., 0., 0.]
    res[0] = -q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3] + q1[0]*q2[0]
    res[1] =  q1[0]*q2[1] - q1[3]*q2[2] + q1[2]*q2[3] + q1[1]*q2[0]
    res[2] =  q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0]
    res[3] = -q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3] + q1[3]*q2[0]
    return res


def rot2quat(R):
    """
    Turn a rotation matrix into a quaternion.

    Input:
      R -- Rotation Matrix
    Output
      Q -- Quaternion [w, ex, ey, ez]

    """
    dEpsilon = 1e-6;
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def rotx(ang):
    """
    Rotation about x. Angle in radians

    """
    M = np.array([[1.0, 0.0, 0.0],
                  [0.0, np.cos(ang), -np.sin(ang)],
                  [0.0, np.sin(ang), np.cos(ang)]])
    return M


def roty(ang):
    """
    Rotation about y. Angle in radians

    """
    M = np.array([[np.cos(ang), 0.0, np.sin(ang)],
                  [0.0, 1.0, 0.0],
                  [-np.sin(ang), 0.0, np.cos(ang)]])
    return M


def rotz(ang):
    """
    Rotation about z. Angle in radians

    """
    M = np.array([[np.cos(ang), -np.sin(ang), 0.0],
                  [np.sin(ang), np.cos(ang), 0.0],
                  [0.0, 0.0, 1.0]])
    return M


def sgn(x):
    """
    Sign function 

    """
    if (x >= 0):
        return 1.0
    else:
        return -1.0


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1];
    R[1,0] = w[2];  R[1,2] = -w[0];
    R[2,0] = -w[1]; R[2,1] = w[0];
    return R

    
def setJoints(q):
    """
    Create a message that contains the joint names and the provided joint values.

    Arguments:
      q -- Joint values with order [q0, q1, q2, q3, q4, q5, q6]. It can be a list, tuple
           or a 1D numpy array)

    Return:
      The message with the names: {'right_j0': q0, 'right_j1': q1, ...}

    """
    msg = {'right_j0': q[0], 'right_j1': q[1], 'right_j2': q[2], 'right_j3': q[3],
           'right_j4': q[4], 'right_j5': q[5], 'right_j6': q[6]}

    return msg




"""
--------------------------------------------------------

Functions for the Kinematics of Sawyer

--------------------------------------------------------
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
pi = np.pi


def dh(d, theta, a, alpha):
    """
    Compute the homogeneous transformation matrix associated with the
    Denavit-Hartenberg parameters

    """
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa  = np.sin(alpha)
    ca  = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0.0,      sa,      ca,     d],
                  [0.0,     0.0,     0.0,   1.0]])
    return T


class SawyerRobot(object):

    def __init__(self, mode="real"):
        """
        Arguments

          mode - "real" for the real robot (default), "sim" for the gazebo 
                  simulation

        """
        # Lengths for the links (in meters)
        self.l0 = 0.081;  self.h = 0.317; self.l1 = 0.1925; self.l2 = 0.4;
        self.l3 = 0.1685; self.l4 = 0.4;  self.l5 = 0.1363; self.l6 = 0.13375
        # Memory allocation
        self.Jp = np.zeros((3,7))
        self.J  = np.zeros((7,7))
        # Joint limits
        self.qmin = (-3.05, -3.81, -3.04, -3.04, -2.98, -2.98, -4.71)
        self.qmax = ( 3.05,  2.27,  3.04,  3.04,  2.98,  2.98,  4.71)
        # Topic that contains the sensed joint configuration
        rospy.Subscriber("/robot/joint_states", JointState, self._readJoints)
        # Sensed joint configuration
        self.jstate_ = 7*[0.,]
        # For the real robot
        if (mode=="real"):
            self.set_joints = self._publish_real
        # For the Gazebo simulation
        elif (mode=="sim"):
            ns = "/robot/right_joint_position_controller/joints"
            topic0 = ns + "/right_j0_controller/command"
            topic1 = ns + "/right_j1_controller/command"
            topic2 = ns + "/right_j2_controller/command"
            topic3 = ns + "/right_j3_controller/command"
            topic4 = ns + "/right_j4_controller/command"
            topic5 = ns + "/right_j5_controller/command"
            topic6 = ns + "/right_j6_controller/command"
            self.pub0 = rospy.Publisher(topic0, Float64, queue_size=10)
            self.pub1 = rospy.Publisher(topic1, Float64, queue_size=10)
            self.pub2 = rospy.Publisher(topic2, Float64, queue_size=10)
            self.pub3 = rospy.Publisher(topic3, Float64, queue_size=10)
            self.pub4 = rospy.Publisher(topic4, Float64, queue_size=10)
            self.pub5 = rospy.Publisher(topic5, Float64, queue_size=10)
            self.pub6 = rospy.Publisher(topic6, Float64, queue_size=10)
            self.set_joints = self._publish_sim
        else:
            rospy.logerr("Joint Interface: invalid option for mode")

    def _publish_sim(self, joints):
        """
        Publish the joints to the simulated robot in Gazebo

        """
        self.pub0.publish(joints[0])
        self.pub1.publish(joints[1])
        self.pub2.publish(joints[2])
        self.pub3.publish(joints[3])
        self.pub4.publish(joints[4])
        self.pub5.publish(joints[5])
        self.pub6.publish(joints[6])

    def _publish_real(self, joints):
        """
        Publish the joints to the real robot
        """
        pass

    def _readJoints(self, msg):
        """
        Callback to store the sensed joint positions from joint_states topic

        """
        # jstate_ must always be a valid joint configuration
        self.jstate_ = msg.position[3:]

    def get_joint_state(self):
        return np.array(self.jstate_)

    def _fkine(self, q, link=7):
        """
        Forward kinematics of Sawyer given any joint configuration
          q - joint angles
          link - end link for which the forward kinematics is computed (4 or 7)
        
        """
        # DH matrices
        T1 = dh(  self.h,    pi+q[0], -self.l0, pi/2)
        T2 = dh( self.l1, -pi/2+q[1],      0.0, pi/2)
        T3 = dh( self.l2,    pi+q[2],      0.0, pi/2)
        T4 = dh(-self.l3,    pi+q[3],      0.0, pi/2)
        T5 = dh( self.l4,    pi+q[4],      0.0, pi/2)
        T6 = dh( self.l5,    pi+q[5],      0.0, pi/2)
        T7 = dh( self.l6, -pi/2+q[6],      0.0,  0.0)
        if (link==7):
            # Homogeneous transformations for the end effector (1 to 7)
            T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)
        elif (link==4):
            # Homogeneous transformations for the elbow (1 to 4)
            T = T1.dot(T2).dot(T3).dot(T4)
        else:
            print("Error (_fkine_sawyer): invalid link number")
        return T

    def pose_wrist(self):
        """
        Position and orientation of the robot wrist at the current sensed
        joint configuration

        """
        return self._fkine(np.array(self.jstate_), link=7)

    def pose_elbow(self):
        """
        Position and orientation of the robot elbow at the current sensed
        joint configuration

        """
        return self._fkine(np.array(self.jstate_), link=4)

    def jacobian_wrist_position(self, delta=0.0001):
        """
        Analytic Jacobian for the wrist position. It returns a 3x7 matrix

        Input:
           delta -- increment for the numerical differentiation
        Output
           Jp -- Analytic Jacobian for position
        
        """
        # Current position and orientation (homogeneous transformation)
        T = self.pose_wrist()
        # Current joint configuration
        q = copy(np.array(self.jstate_))
        # Iteration for the derivative of each row
        for i in xrange(7):
            # Copy the initial joing configuration
            dq = copy(q);
            # Increase the i-th joint using a delta
            dq[i] = dq[i]+delta
            # Homogeneous transformation afther the increase (q+dq)
            dT = self._fkine(dq)
            # Approximation of the position Jacobian using finite differences
            self.Jp[:,i] = (dT[0:3,3]-T[0:3,3])/delta
        return self.Jp


    def jacobian_wrist(self, delta=0.0001):
        """
        Analytic Jacobian for both the wrist pose
        
        Input:
           delta -- increment for the numerical differentiation
        Output
           J -- Analytic Jacobian [Jp' Jor']'

        """
        T = self.pose_wrist()
        quat = rot2quat(T[0:3,0:3])
        # Current joint configuration
        q = copy(np.array(self.jstate_))
        for i in xrange(7):
            dq = copy(q);
            dq[i] = dq[i]+delta
            dT = self._fkine(dq)
            self.J[0:3,i] = (dT[0:3,3]-T[0:3,3])/delta
            dquat = rot2quat(dT[0:3,0:3])
            self.J[3:7,i] = (dquat-quat)/delta
        return J

    def ikine_wrist(self, xdes):
        """
        Compute the inverse kinematics of Sawyer numerically from the current
        joint configuration

        """
        epsilon  = 0.001
        max_iter = 1000
        delta    = 0.00001
        
        q  = copy(np.array(self.jstate_))
        for i in range(max_iter):
            J = self.jacobian_wrist_position()
            Jpinv = np.linalg.pinv(J)
            # TODO
            # Use damping pinv

            
            T = self.pose_wrist()
            error = xdes - T[0:3,3];
            q = q + np.dot(Jpinv, error);
            # End condition
            if (np.linalg.norm(error)<epsilon):
                break
        return q



class JoystickInterface(object):
    def __init__(self):
        rospy.Subscriber("/joy", Joy, self.readJoystick)
        self.joy_ = Joy()
        self.joy_.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
    def readJoystick(self, msg):
        # print msg
        self.joy_ = msg

    def get_values(self):
        return self.joy_
    
