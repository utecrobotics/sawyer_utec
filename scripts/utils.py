#
# Some utility functions
#

import numpy as np


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


def sgn(x):
    """
    Sign function 

    """
    if (x >= 0):
        return 1.0
    else:
        return -1.0

