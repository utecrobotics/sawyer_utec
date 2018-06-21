"""
Universidad de Ingenieria y Tecnologia - UTEC
Robotics and Mechatronics Lab

Some utility functions

"""

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

Functions for the forward Kinematics of Sawyer

--------------------------------------------------------
"""

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


def fkine(q):
    """
    Forward kinematics of Sawyer given the joint angles.

    """
    # Lengths (in meters)
    l0 = 0.081; h = 0.317; l1 = 0.1925; l2 = 0.4;
    l3 = 0.1685; l4 = 0.4; l5 = 0.1363; l6 = 0.13375
    # DH matrices
    T1 = dh(  h,    pi+q[0], -l0, pi/2)
    T2 = dh( l1, -pi/2+q[1],   0, pi/2)
    T3 = dh( l2,    pi+q[2],   0, pi/2)
    T4 = dh(-l3,    pi+q[3],   0, pi/2)
    T5 = dh( l4,    pi+q[4],   0, pi/2)
    T6 = dh( l5,    pi+q[5],   0, pi/2)
    T7 = dh( l6, -pi/2+q[6],   0,    0)
    # Homogeneous transformations for the end effector (0 to 7)
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)
    return T


def jacobian_position(q, delta=0.0001):
    """
    Analytic Jacobian for the position. It returns a 3x7 matrix

    """
    # Memory allocation
    J = np.zeros((3,7))
    # Initial homogeneous transformation (with q)
    T = fkine(q)
    # Iteration for the derivative of each row
    for i in xrange(7):
        # Copy the initial joing configuration
        dq = copy(q);
        # Increase the i-th joint using a delta
        dq[i] = dq[i]+delta
        # Homogeneous transformation afther the increase (q+dq)
        dT = fkine(dq)
        # Approximation of the position Jacobian using finite differences
        J[:,i] = (dT[0:3,3]-T[0:3,3])/delta
    return J


def jacobian_pose(q, delta=0.0001):
    """
    Analytic Jacobian for both the position and the orientation

    Input:
      q -- Joint configuration (numpy array)
      delta -- increment for the numerical differentiation
    Output
      J -- Analytic Jacobian [Jp' Jor']'

    """
    J = np.zeros((7,7))
    T = fkine(q)
    quat = rot2quat(T[0:3,0:3])
    for i in xrange(7):
        dq = copy(q);
        dq[i] = dq[i]+delta
        dT = fkine(dq)
        J[0:3,i] = (dT[0:3,3]-T[0:3,3])/delta
        dquat = rot2quat(dT[0:3,0:3])
        J[3:7,i] = (dquat-quat)/delta
    return J
