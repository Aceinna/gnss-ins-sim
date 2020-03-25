# -*- coding: utf-8 -*-
# Filename: attitude.py

"""
Attitude representations and their transformations
Created on 2017-09-12
@author: dongxiaoguang
"""

# import
import math
import numpy as np
#import scipy.linalg

# global
VERSION = '1.0'
TWO_PI = 2.0*math.pi
HALF_PI = 0.5*math.pi
D2R = math.pi/180.0
R2D = 180.0/math.pi

def get_cn2b_acc_mag_ned(acc, mag):
    '''
    Calculate NED to body transformation matrix from acc and mag.
    Args:
        acc: acc measurement, numpy array of 3x1.
        mag: mag measurement, numpy array of 3x1.
    Returns:
        cn2b: transformation matrix from NED to body
    '''
    z = -acc / math.sqrt(np.dot(acc, acc))
    acc_cross_mag = np.cross(z, mag)
    y = acc_cross_mag / math.sqrt(np.dot(acc_cross_mag, acc_cross_mag))
    x = np.cross(y, z)
    cn2b = np.zeros((3, 3))
    cn2b[0][0] = x[0]
    cn2b[1][0] = x[1]
    cn2b[2][0] = x[2]
    cn2b[0][1] = y[0]
    cn2b[1][1] = y[1]
    cn2b[2][1] = y[2]
    cn2b[0][2] = z[0]
    cn2b[1][2] = z[1]
    cn2b[2][2] = z[2]
    return cn2b

def quat_normalize(q):
    """
    Normalize a quaternion, scalar part is always non-negative
    Args:
        q: quaternion
    Returns:
        qn: normalized quaternion, scalar part is always non-negative
    """
    if q[0] < 0:
        q = -q
    q_norm = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    qn = q / q_norm
    return qn

def quat_conj(q):
    """
    Conjugate of quaternion
    Args:
        q: quaternion, scalar first
    Returns:
        qc: quaternion conjugate
    """
    qc = q.copy()
    qc[1] = -q[1]
    qc[2] = -q[2]
    qc[3] = -q[3]
    return qc

def quat_multiply(q1, q2):
    """
    Multiplication of two quaternions
    Args:
        q1: quaternion, scalar first
        q2: quaternion, scalar first
    Returns:
        q = q1 * q2
    """
    q = np.array([0.0, 0.0, 0.0, 0.0])
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    q[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    q[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    q[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return q

def quat2euler(q, rot_seq='zyx'):
    '''
    Convert quaternion to Euler angles
    Args:
        q: quaternion, [q0, q1, q2, q3], q0 is the scalar
        rot_seq: rotation sequence corresponding to the angles.
    Return:
        angles: 3x1 Euler angles, rad.
    '''
    rot_seq = rot_seq.lower()
    if rot_seq == 'zyx':
        [r1, r2, r3] = three_axis_rot(2.0*(q[1]*q[2] + q[0]*q[3]),
                                      q[0]**2.0 + q[1]**2.0 - q[2]**2.0 - q[3]**2.0,
                                      -2.0*(q[1]*q[3] - q[0]*q[2]),
                                      2.0*(q[2]*q[3] + q[0]*q[1]),
                                      q[0]**2.0 - q[1]**2.0 - q[2]**2.0 + q[3]**2.0)
        return np.array([r1, r2, r3])
    elif rot_seq == 'zyz':
        [r1, r2, r3] = two_axis_rot(2.0*(q[2]*q[3] - q[0]*q[1]),
                                    2.0*(q[1]*q[3] + q[0]*q[2]),
                                    q[0]**2.0 - q[1]**2.0 - q[2]**2.0 + q[3]**2.0,
                                    2.0*(q[2]*q[3] + q[0]*q[1]),
                                    -2.0*(q[1]*q[3] - q[0]*q[2]))
        return np.array([r1, r2, r3])
    elif rot_seq == 'zxy':
        [r1, r2, r3] = three_axis_rot(-2.0*(q[1]*q[2] - q[0]*q[3]),
                                      q[0]**2.0 - q[1]**2.0 + q[2]**2.0 - q[3]**2.0,
                                      2.0*(q[2]*q[3] + q[0]*q[1]),
                                      -2.0*(q[1]*q[3] - q[0]*q[2]),
                                      q[0]**2.0 - q[1]**2.0 - q[2]**2.0 + q[3]**2.0)
        return np.array([r1, r2, r3])
    elif rot_seq == 'zxz':
        [r1, r2, r3] = two_axis_rot(2.0*(q[1]*q[3] + q[0]*q[2]),
                                    -2.0*(q[2]*q[3] - q[0]*q[1]),
                                    q[0]**2.0 - q[1]**2.0 - q[2]**2.0 + q[3]**2.0,
                                    2.0*(q[1]*q[3] - q[0]*q[2]),
                                    2.0*(q[2]*q[3] + q[0]*q[1]))
        return np.array([r1, r2, r3])
    elif rot_seq == 'yxz':
        [r1, r2, r3] = three_axis_rot(2.0*(q[1]*q[3] + q[0]*q[2]),
                                      q[0]**2.0 - q[1]**2.0 - q[2]**2.0 + q[3]**2.0,
                                      -2.0*(q[2]*q[3] - q[0]*q[1]),
                                      2.0*(q[1]*q[2] + q[0]*q[3]),
                                      q[0]**2.0 - q[1]**2.0 + q[2]**2.0 - q[3]**2.0)
        return np.array([r1, r2, r3])
    elif rot_seq == 'yxy':
        [r1, r2, r3] = two_axis_rot(2.0*(q[1]*q[2] - q[0]*q[3]),
                                    2.0*(q[2]*q[3] + q[0]*q[1]),
                                    q[0]**2.0 - q[1]**2.0 + q[2]**2.0 - q[3]**2.0,
                                    2.0*(q[1]*q[2] + q[0]*q[3]),
                                    -2.0*(q[2]*q[3] - q[0]*q[1]))
        return np.array([r1, r2, r3])
    elif rot_seq == 'yzx':
        [r1, r2, r3] = three_axis_rot(-2.0*(q[1]*q[3] - q[0]*q[2]),
                                      q[0]**2.0 + q[1]**2.0 - q[2]**2.0 - q[3]**2.0,
                                      2.0*(q[1]*q[2] + q[0]*q[3]),
                                      -2.0*(q[2]*q[3] - q[0]*q[1]),
                                      q[0]**2.0 - q[1]**2.0 + q[2]**2.0 - q[3]**2.0)
        return np.array([r1, r2, r3])
    elif rot_seq == 'yzy':
        [r1, r2, r3] = two_axis_rot(2.0*(q[2]*q[3] + q[0]*q[1]),
                                    -2.0*(q[1]*q[2] - q[0]*q[3]),
                                    q[0]**2.0 - q[1]**2.0 + q[2]**2.0 - q[3]**2.0,
                                    2.0*(q[2]*q[3] - q[0]*q[1]),
                                    2.0*(q[1]*q[2] + q[0]*q[3]))
        return np.array([r1, r2, r3])
    elif rot_seq == 'xyz':
        [r1, r2, r3] = three_axis_rot(-2.0*(q[2]*q[3] - q[0]*q[1]),
                                      q[0]**2.0 - q[1]**2.0 - q[2]**2.0 + q[3]**2.0,
                                      2.0*(q[1]*q[3] + q[0]*q[2]),
                                      -2.0*(q[1]*q[2] - q[0]*q[3]),
                                      q[0]**2.0 + q[1]**2.0 - q[2]**2.0 - q[3]**2.0)
        return np.array([r1, r2, r3])
    elif rot_seq == 'xyx':
        [r1, r2, r3] = two_axis_rot(2.0*(q[1]*q[2] + q[0]*q[3]),
                                    -2.0*(q[1]*q[3] - q[0]*q[2]),
                                    q[0]**2.0 + q[1]**2.0 - q[2]**2.0 - q[3]**2.0,
                                    2.0*(q[1]*q[2] - q[0]*q[3]),
                                    2.0*(q[1]*q[3] + q[0]*q[2]))
        return np.array([r1, r2, r3])
    elif rot_seq == 'xzy':
        [r1, r2, r3] = three_axis_rot(2.0*(q[2]*q[3] + q[0]*q[1]),
                                      q[0]**2.0 - q[1]**2.0 + q[2]**2.0 - q[3]**2.0,
                                      -2.0*(q[1]*q[2] - q[0]*q[3]),
                                      2.0*(q[1]*q[3] + q[0]*q[2]),
                                      q[0]**2.0 + q[1]**2.0 - q[2]**2.0 - q[3]**2.0)
        return np.array([r1, r2, r3])
    elif rot_seq == 'xzx':
        [r1, r2, r3] = two_axis_rot(2.0*(q[1]*q[3] - q[0]*q[2]),
                                    2.0*(q[1]*q[2] + q[0]*q[3]),
                                    q[0]**2.0 + q[1]**2.0 - q[2]**2.0 - q[3]**2.0,
                                    2.0*(q[1]*q[3] + q[0]*q[2]),
                                    -2.0*(q[1]*q[2] - q[0]*q[3]))
        return np.array([r1, r2, r3])
    else:
        return False

def euler2quat(angles, rot_seq='zyx'):
    '''
    Convert Euler angles to quaternion.
    Args:
        angles: 3x1 Euler angles, rad.
        rot_seq: rotation sequence corresponding to the angles.
    Return:
        q: quaternion, [q0, q1, q2, q3], q0 is the scalar
    """
    '''
    cangle = np.cos(0.5*angles)
    sangle = np.sin(0.5*angles)
    rot_seq = rot_seq.lower()
    if rot_seq == 'zyx':
        return np.array([cangle[0]*cangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] - sangle[0]*sangle[1]*cangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*cangle[1]*sangle[2],
                         sangle[0]*cangle[1]*cangle[2] - cangle[0]*sangle[1]*sangle[2]])
    elif rot_seq == 'zyz':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2],
                         cangle[0]*sangle[1]*sangle[2] - sangle[0]*sangle[1]*cangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2],
                         sangle[0]*cangle[1]*cangle[2] + cangle[0]*cangle[1]*sangle[2]])
    elif rot_seq == 'zxy':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*sangle[1]*sangle[2],
                         cangle[0]*sangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] + sangle[0]*sangle[1]*cangle[2],
                         cangle[0]*sangle[1]*sangle[2] + sangle[0]*cangle[1]*cangle[2]])
    elif rot_seq == 'zxz':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2],
                         sangle[0]*sangle[1]*cangle[2] - cangle[0]*sangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] + sangle[0]*cangle[1]*cangle[2]])
    elif rot_seq == 'yxz':
        return np.array([cangle[0]*cangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*cangle[1]*sangle[2],
                         sangle[0]*cangle[1]*cangle[2] - cangle[0]*sangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] - sangle[0]*sangle[1]*cangle[2]])
    elif rot_seq == 'yxy':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2],
                         sangle[0]*cangle[1]*cangle[2] + cangle[0]*cangle[1]*sangle[2],
                         cangle[0]*sangle[1]*sangle[2] - sangle[0]*sangle[1]*cangle[2]])
    elif rot_seq == 'yzx':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*sangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] + sangle[0]*sangle[1]*cangle[2],
                         cangle[0]*sangle[1]*sangle[2] + sangle[0]*cangle[1]*cangle[2],
                         cangle[0]*sangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2]])
    elif rot_seq == 'yzy':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2],
                         sangle[0]*sangle[1]*cangle[2] - cangle[0]*sangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] + sangle[0]*cangle[1]*cangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2]])
    elif rot_seq == 'xyz':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*sangle[1]*sangle[2],
                         cangle[0]*sangle[1]*sangle[2] + sangle[0]*cangle[1]*cangle[2],
                         cangle[0]*sangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] + sangle[0]*sangle[1]*cangle[2]])
    elif rot_seq == 'xyx':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] + sangle[0]*cangle[1]*cangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2],
                         sangle[0]*sangle[1]*cangle[2] - cangle[0]*sangle[1]*sangle[2]])
    elif rot_seq == 'xzy':
        return np.array([cangle[0]*cangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2],
                         sangle[0]*cangle[1]*cangle[2] - cangle[0]*sangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] - sangle[0]*sangle[1]*cangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*cangle[1]*sangle[2]])
    elif rot_seq == 'xzx':
        return np.array([cangle[0]*cangle[1]*cangle[2] - sangle[0]*cangle[1]*sangle[2],
                         cangle[0]*cangle[1]*sangle[2] + sangle[0]*cangle[1]*cangle[2],
                         cangle[0]*sangle[1]*sangle[2] - sangle[0]*sangle[1]*cangle[2],
                         cangle[0]*sangle[1]*cangle[2] + sangle[0]*sangle[1]*sangle[2]])
    else:
        return False

def quat2dcm(q):
    """
    Convert quaternion to direction cosine matrix
    Args:
        q: quaternion, [q0, q1, q2, q3], q0 is the scalar
    Return:
        dcm: direction cosine matrix
    """
    q0q0 = q[0] * q[0]
    q0q1 = q[0] * q[1]
    q0q2 = q[0] * q[2]
    q0q3 = q[0] * q[3]
    q1q1 = q[1] * q[1]
    q1q2 = q[1] * q[2]
    q1q3 = q[1] * q[3]
    q2q2 = q[2] * q[2]
    q2q3 = q[2] * q[3]
    q3q3 = q[3] * q[3]
    dcm = np.zeros((3, 3))
    dcm[0, 0] = q0q0 + q1q1 - q2q2 - q3q3
    dcm[0, 1] = 2.0*(q1q2 + q0q3)
    dcm[0, 2] = 2.0*(q1q3 - q0q2)
    dcm[1, 0] = 2.0*(q1q2 - q0q3)
    dcm[1, 1] = q0q0 - q1q1 + q2q2 - q3q3
    dcm[1, 2] = 2.0*(q2q3 + q0q1)
    dcm[2, 0] = 2.0*(q1q3 + q0q2)
    dcm[2, 1] = 2.0*(q2q3 - q0q1)
    dcm[2, 2] = q0q0 - q1q1 - q2q2 + q3q3
    return dcm

def dcm2quat(c):
    """
    Convert direction cosine matrix to quaternion
    Args:
        c: direciton cosine matrix
    Returns:
        q: quaternion, scalar first
    """
    tr = c[0, 0] + c[1, 1] + c[2, 2]
    tmp = np.array([0.0, 0.0, 0.0, 0.0])
    q = np.array([0.0, 0.0, 0.0, 0.0])
    if tr > 0.0:
        tmp[0] = 0.5 * math.sqrt(1.0 + tr)
        tmp[1] = 0.25 / tmp[0] * (c[1, 2] - c[2, 1])
        tmp[2] = 0.25 / tmp[0] * (c[2, 0] - c[0, 2])
        tmp[3] = 0.25 / tmp[0] * (c[0, 1] - c[1, 0])
    else:
        if (c[1, 1] > c[0, 0]) and (c[1, 1] > c[2, 2]):
            sqdip1 = math.sqrt(c[1, 1] - c[0, 0] - c[2, 2] + 1.0)
            tmp[2] = 0.5*sqdip1
            if sqdip1 != 0.0:    #// if it equals 0, something is wrong
                sqdip1 = 0.5/sqdip1
            tmp[0] = (c[2, 0] - c[0, 2]) * sqdip1
            tmp[1] = (c[0, 1] + c[1, 0]) * sqdip1
            tmp[3] = (c[1, 2] + c[2, 1]) * sqdip1
        elif c[2, 2] > c[0, 0]:
            sqdip1 = math.sqrt(c[2, 2] - c[0, 0] - c[1, 1] + 1.0)
            tmp[3] = 0.5*sqdip1
            if sqdip1 != 0.0: #// if it equals 0, something is wrong
                sqdip1 = 0.5/sqdip1
            tmp[0] = (c[0, 1] - c[1, 0]) * sqdip1
            tmp[1] = (c[2, 0] + c[0, 2]) * sqdip1
            tmp[2] = (c[1, 2] + c[2, 1]) * sqdip1
        else:
            sqdip1 = math.sqrt(c[0, 0] - c[1, 1] - c[2, 2] + 1.0)
            tmp[1] = 0.5*sqdip1
            if sqdip1 != 0.0:    #// if it equals 0, something is wrong
                sqdip1 = 0.5/sqdip1
            tmp[0] = (c[1, 2] - c[2, 1]) * sqdip1
            tmp[2] = (c[0, 1] + c[1, 0]) * sqdip1
            tmp[3] = (c[2, 0] + c[0, 2]) * sqdip1
    # quaternion normalization, *** no need if dcm is really a dcm
    #quatNormalize(tmp,q)
	# ensure q[0] is non-negative
    if tmp[0] < 0:
        q = -1.0 * tmp
    else:
        q = tmp
    return q

def euler2dcm(angles, rot_seq='zyx'):
    """
    Convert Euler angles to direction cosine matrix.
    The Euler angles rotate the frame n to the frame b according to specified
    rotation sequency. The DCM is a 3x3 coordinate transformation matrix from n
    to b. That is v_b  = DCM * v_n. '_b' or '_n' mean the vector 'v' is expressed
    in the frame b or n.
    Args:
        angles: 3x1 Euler angles, rad.
        rot_seq: rotation sequence corresponding to the angles.
    Returns:
        dcm: 3x3 coordinate transformation matrix from n to b
    """
    dcm = np.zeros((3, 3))
    cangle = np.cos(angles)
    sangle = np.sin(angles)
    rot_seq = rot_seq.lower()
    if rot_seq == 'zyx':
        dcm[0, 0] = cangle[1]*cangle[0]
        dcm[0, 1] = cangle[1]*sangle[0]
        dcm[0, 2] = -sangle[1]
        dcm[1, 0] = sangle[2]*sangle[1]*cangle[0] - cangle[2]*sangle[0]
        dcm[1, 1] = sangle[2]*sangle[1]*sangle[0] + cangle[2]*cangle[0]
        dcm[1, 2] = cangle[1]*sangle[2]
        dcm[2, 0] = sangle[1]*cangle[2]*cangle[0] + sangle[0]*sangle[2]
        dcm[2, 1] = sangle[1]*cangle[2]*sangle[0] - cangle[0]*sangle[2]
        dcm[2, 2] = cangle[1]*cangle[2]
        return dcm
    elif rot_seq == 'zyz':
        dcm[0, 0] = cangle[0]*cangle[2]*cangle[1] - sangle[0]*sangle[2]
        dcm[0, 1] = sangle[0]*cangle[2]*cangle[1] + cangle[0]*sangle[2]
        dcm[0, 2] = -sangle[1]*cangle[2]
        dcm[1, 0] = -cangle[0]*cangle[1]*sangle[2] - sangle[0]*cangle[2]
        dcm[1, 1] = -sangle[0]*cangle[1]*sangle[2] + cangle[0]*cangle[2]
        dcm[1, 2] = sangle[1]*sangle[2]
        dcm[2, 0] = cangle[0]*sangle[1]
        dcm[2, 1] = sangle[0]*sangle[1]
        dcm[2, 2] = cangle[1]
        return dcm
    elif rot_seq == 'zxy':
        dcm[0, 0] = cangle[2]*cangle[0] - sangle[1]*sangle[2]*sangle[0]
        dcm[0, 1] = cangle[2]*sangle[0] + sangle[1]*sangle[2]*cangle[0]
        dcm[0, 2] = -sangle[2]*cangle[1]
        dcm[1, 0] = -cangle[1]*sangle[0]
        dcm[1, 1] = cangle[1]*cangle[0]
        dcm[1, 2] = sangle[1]
        dcm[2, 0] = sangle[2]*cangle[0] + sangle[1]*cangle[2]*sangle[0]
        dcm[2, 1] = sangle[2]*sangle[0] - sangle[1]*cangle[2]*cangle[0]
        dcm[2, 2] = cangle[1]*cangle[2]
        return dcm
    elif rot_seq == 'zxz':
        dcm[0, 0] = -sangle[0]*cangle[1]*sangle[2] + cangle[0]*cangle[2]
        dcm[0, 1] = cangle[0]*cangle[1]*sangle[2] + sangle[0]*cangle[2]
        dcm[0, 2] = sangle[1]*sangle[2]
        dcm[1, 0] = -sangle[0]*cangle[2]*cangle[1] - cangle[0]*sangle[2]
        dcm[1, 1] = cangle[0]*cangle[2]*cangle[1] - sangle[0]*sangle[2]
        dcm[1, 2] = sangle[1]*cangle[2]
        dcm[2, 0] = sangle[0]*sangle[1]
        dcm[2, 1] = -cangle[0]*sangle[1]
        dcm[2, 2] = cangle[1]
        return dcm
    elif rot_seq == 'yxz':
        dcm[0, 0] = cangle[0]*cangle[2] + sangle[1]*sangle[0]*sangle[2]
        dcm[0, 1] = cangle[1]*sangle[2]
        dcm[0, 2] = -sangle[0]*cangle[2] + sangle[1]*cangle[0]*sangle[2]
        dcm[1, 0] = -cangle[0]*sangle[2] + sangle[1]*sangle[0]*cangle[2]
        dcm[1, 1] = cangle[1]*cangle[2]
        dcm[1, 2] = sangle[0]*sangle[2] + sangle[1]*cangle[0]*cangle[2]
        dcm[2, 0] = sangle[0]*cangle[1]
        dcm[2, 1] = -sangle[1]
        dcm[2, 2] = cangle[1]*cangle[0]
        return dcm
    elif rot_seq == 'yxy':
        dcm[0, 0] = -sangle[0]*cangle[1]*sangle[2] + cangle[0]*cangle[2]
        dcm[0, 1] = sangle[1]*sangle[2]
        dcm[0, 2] = -cangle[0]*cangle[1]*sangle[2] - sangle[0]*cangle[2]
        dcm[1, 0] = sangle[0]*sangle[1]
        dcm[1, 1] = cangle[1]
        dcm[1, 2] = cangle[0]*sangle[1]
        dcm[2, 0] = sangle[0]*cangle[2]*cangle[1] + cangle[0]*sangle[2]
        dcm[2, 1] = -sangle[1]*cangle[2]
        dcm[2, 2] = cangle[0]*cangle[2]*cangle[1] - sangle[0]*sangle[2]
        return dcm
    elif rot_seq == 'yxz':
        dcm[0, 0] = cangle[0]*cangle[1]
        dcm[0, 1] = sangle[1]
        dcm[0, 2] = -sangle[0]*cangle[1]
        dcm[1, 0] = -cangle[2]*cangle[0]*sangle[1] + sangle[2]*sangle[0]
        dcm[1, 1] = cangle[1]*cangle[2]
        dcm[1, 2] = cangle[2]*sangle[0]*sangle[1] + sangle[2]*cangle[0]
        dcm[2, 0] = sangle[2]*cangle[0]*sangle[1] + cangle[2]*sangle[0]
        dcm[2, 1] = -sangle[2]*cangle[1]
        dcm[2, 2] = -sangle[2]*sangle[0]*sangle[1] + cangle[2]*cangle[0]
        return dcm
    elif rot_seq == 'yzy':
        dcm[0, 0] = cangle[0]*cangle[2]*cangle[1] - sangle[0]*sangle[2]
        dcm[0, 1] = sangle[1]*cangle[2]
        dcm[0, 2] = -sangle[0]*cangle[2]*cangle[1] - cangle[0]*sangle[2]
        dcm[1, 0] = -cangle[0]*sangle[1]
        dcm[1, 1] = cangle[1]
        dcm[1, 2] = sangle[0]*sangle[1]
        dcm[2, 0] = cangle[0]*cangle[1]*sangle[2] + sangle[0]*cangle[2]
        dcm[2, 1] = sangle[1]*sangle[2]
        dcm[2, 2] = -sangle[0]*cangle[1]*sangle[2] + cangle[0]*cangle[2]
        return dcm
    elif rot_seq == 'xyz':
        dcm[0, 0] = cangle[1]*cangle[2]
        dcm[0, 1] = sangle[0]*sangle[1]*cangle[2] + cangle[0]*sangle[2]
        dcm[0, 2] = -cangle[0]*sangle[1]*cangle[2] + sangle[0]*sangle[2]
        dcm[1, 0] = -cangle[1]*sangle[2]
        dcm[1, 1] = -sangle[0]*sangle[1]*sangle[2] + cangle[0]*cangle[2]
        dcm[1, 2] = cangle[0]*sangle[1]*sangle[2] + sangle[0]*cangle[2]
        dcm[2, 0] = sangle[1]
        dcm[2, 1] = -sangle[0]*cangle[1]
        dcm[2, 2] = cangle[0]*cangle[1]
        return dcm
    elif rot_seq == 'xyx':
        dcm[0, 0] = cangle[1]
        dcm[0, 1] = sangle[0]*sangle[1]
        dcm[0, 2] = -cangle[0]*sangle[1]
        dcm[1, 0] = sangle[1]*sangle[2]
        dcm[1, 1] = -sangle[0]*cangle[1]*sangle[2] + cangle[0]*cangle[2]
        dcm[1, 2] = cangle[0]*cangle[1]*sangle[2] + sangle[0]*cangle[2]
        dcm[2, 0] = sangle[1]*cangle[2]
        dcm[2, 1] = -sangle[0]*cangle[2]*cangle[1] - cangle[0]*sangle[2]
        dcm[2, 2] = cangle[0]*cangle[2]*cangle[1] - sangle[0]*sangle[2]
        return dcm
    elif rot_seq == 'xzy':
        dcm[0, 0] = cangle[2]*cangle[1]
        dcm[0, 1] = cangle[0]*cangle[2]*sangle[1] + sangle[0]*sangle[2]
        dcm[0, 2] = sangle[0]*cangle[2]*sangle[1] - cangle[0]*sangle[2]
        dcm[1, 0] = -sangle[1]
        dcm[1, 1] = cangle[0]*cangle[1]
        dcm[1, 2] = sangle[0]*cangle[1]
        dcm[2, 0] = sangle[2]*cangle[1]
        dcm[2, 1] = cangle[0]*sangle[1]*sangle[2] - sangle[0]*cangle[2]
        dcm[2, 2] = sangle[0]*sangle[1]*sangle[2] + cangle[0]*cangle[2]
        return dcm
    elif rot_seq == 'xzx':
        dcm[0, 0] = cangle[1]
        dcm[0, 1] = cangle[0]*sangle[1]
        dcm[0, 2] = sangle[0]*sangle[1]
        dcm[1, 0] = -sangle[1]*cangle[2]
        dcm[1, 1] = cangle[0]*cangle[2]*cangle[1] - sangle[0]*sangle[2]
        dcm[1, 2] = sangle[0]*cangle[2]*cangle[1] + cangle[0]*sangle[2]
        dcm[2, 0] = sangle[1]*sangle[2]
        dcm[2, 1] = -cangle[0]*cangle[1]*sangle[2] - sangle[0]*cangle[2]
        dcm[2, 2] = -sangle[0]*cangle[1]*sangle[2] + cangle[0]*cangle[2]
        return dcm
    else:
        return False

def dcm2euler(dcm, rot_seq='zyx'):
    """
    Convert direction cosine matrix to Euler angles.
    The Euler angles rotate the frame n to the frame b according to specified
    rotation sequency. The DCM is a 3x3 coordinate transformation matrix from n
    to b. That is v_b  = DCM * v_n. '_b' or '_n' mean the vector 'v' is expressed
    in the frame b or n.
    Args:
        dcm: 3x3 coordinate transformation matrix from n to b
        rot_seq: rotation sequence corresponding to the angles.
    Returns:
        angles: 3x1 Euler angles, rad.
    """
    if rot_seq == 'zyx':
        #     [          cy*cz,          cy*sz,            -sy]
        #     [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
        #     [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
        [r1, r2, r3] = three_axis_rot(dcm[0][1], dcm[0][0], -dcm[0][2],
                                      dcm[1][2], dcm[2][2])
        return np.array([r1, r2, r3])
    elif rot_seq == 'zyz':
        #     [  cz2*cy*cz-sz2*sz,  cz2*cy*sz+sz2*cz,           -cz2*sy]
        #     [ -sz2*cy*cz-cz2*sz, -sz2*cy*sz+cz2*cz,            sz2*sy]
        #     [             sy*cz,             sy*sz,                cy]
        [r1, r2, r3] = two_axis_rot(dcm[2][1], dcm[2][0], dcm[2][2],
                                    dcm[1][2], -dcm[0][2])
        return np.array([r1, r2, r3])
    elif rot_seq == 'zxy':
        #     [ cy*cz-sy*sx*sz, cy*sz+sy*sx*cz,         -sy*cx]
        #     [         -sz*cx,          cz*cx,             sx]
        #     [ sy*cz+cy*sx*sz, sy*sz-cy*sx*cz,          cy*cx]
        [r1, r2, r3] = three_axis_rot(-dcm[1][0], dcm[1][1], dcm[1][2],
                                      -dcm[0][2], dcm[2][2])
        return np.array([r1, r2, r3])
    elif rot_seq == 'zxz':
        #     [  cz2*cz-sz2*cx*sz,  cz2*sz+sz2*cx*cz,            sz2*sx]
        #     [ -sz2*cz-cz2*cx*sz, -sz2*sz+cz2*cx*cz,            cz2*sx]
        #     [             sz*sx,            -cz*sx,                cx]
        [r1, r2, r3] = two_axis_rot(dcm[2][0], -dcm[2][1], dcm[2][2],
                                    dcm[0][2], dcm[1][2])
        return np.array([r1, r2, r3])
    elif rot_seq == 'yxz':
        #     [  cy*cz+sy*sx*sz,           sz*cx, -sy*cz+cy*sx*sz]
        #     [ -cy*sz+sy*sx*cz,           cz*cx,  sy*sz+cy*sx*cz]
        #     [           sy*cx,             -sx,           cy*cx]
        [r1, r2, r3] = three_axis_rot(dcm[2][0], dcm[2][2], -dcm[2][1],
                                      dcm[0][1], dcm[1][1])
        return np.array([r1, r2, r3])
    elif rot_seq == 'yxy':
        #     [  cy2*cy-sy2*cx*sy,            sy2*sx, -cy2*sy-sy2*cx*cy]
        #     [             sy*sx,                cx,             cy*sx]
        #     [  sy2*cy+cy2*cx*sy,           -cy2*sx, -sy2*sy+cy2*cx*cy]
        [r1, r2, r3] = two_axis_rot(dcm[1][0], dcm[1][2], dcm[1][1],
                                    dcm[0][1], -dcm[2][1])
        return np.array([r1, r2, r3])
    elif rot_seq == 'yzx':
        #     [           cy*cz,              sz,          -sy*cz]
        #     [ -sz*cx*cy+sy*sx,           cz*cx,  sy*cx*sz+cy*sx]
        #     [  cy*sx*sz+sy*cx,          -cz*sx, -sy*sx*sz+cy*cx]
        [r1, r2, r3] = three_axis_rot(-dcm[0][2], dcm[0][0], dcm[0][1],
                                      -dcm[2][1], dcm[1][1])
        return np.array([r1, r2, r3])
    elif rot_seq == 'yzy':
        #     [  cy2*cz*cy-sy2*sy,            cy2*sz, -cy2*cz*sy-sy2*cy]
        #     [            -cy*sz,                cz,             sy*sz]
        #     [  sy2*cz*cy+cy2*sy,            sy2*sz, -sy2*cz*sy+cy2*cy]
        [r1, r2, r3] = two_axis_rot(dcm[1][2], -dcm[1][0], dcm[1][1],
                                    dcm[2][1], dcm[0][1])
        return np.array([r1, r2, r3])
    elif rot_seq == 'xyz':
        #     [          cy*cz, sz*cx+sy*sx*cz, sz*sx-sy*cx*cz]
        #     [         -cy*sz, cz*cx-sy*sx*sz, cz*sx+sy*cx*sz]
        #     [             sy,         -cy*sx,          cy*cx]
        [r1, r2, r3] = three_axis_rot(-dcm[2][1], dcm[2][2], dcm[2][0],
                                      -dcm[1][0], dcm[0][0])
        return np.array([r1, r2, r3])
    elif rot_seq == 'xyx':
        #     [                cy,             sy*sx,            -sy*cx]
        #     [            sx2*sy,  cx2*cx-sx2*cy*sx,  cx2*sx+sx2*cy*cx]
        #     [            cx2*sy, -sx2*cx-cx2*cy*sx, -sx2*sx+cx2*cy*cx]
        [r1, r2, r3] = two_axis_rot(dcm[0][1], -dcm[0][2], dcm[0][0],
                                    dcm[1][0], dcm[2][0])
        return np.array([r1, r2, r3])
    elif rot_seq == 'xzy':
        #     [          cy*cz, sz*cx*cy+sy*sx, cy*sx*sz-sy*cx]
        #     [            -sz,          cz*cx,          cz*sx]
        #     [          sy*cz, sy*cx*sz-cy*sx, sy*sx*sz+cy*cx]
        [r1, r2, r3] = three_axis_rot(dcm[1][2], dcm[1][1], -dcm[1][0],
                                      dcm[2][0], dcm[0][0])
        return np.array([r1, r2, r3])
    elif rot_seq == 'xzx':
        #     [                cz,             sz*cx,             sz*sx]
        #     [           -cx2*sz,  cx2*cz*cx-sx2*sx,  cx2*cz*sx+sx2*cx]
        #     [            sx2*sz, -sx2*cz*cx-cx2*sx, -sx2*cz*sx+cx2*cx]
        [r1, r2, r3] = two_axis_rot(dcm[0][2], dcm[0][1], dcm[0][0],
                                    dcm[2][0], -dcm[1][0])
        return np.array([r1, r2, r3])
    else:
        return False

def ecef_to_ned(lat, lon):
    '''
    transformation matrix from the ECEF frame to the NED frame defined by lat and lon.
    Args:
        lat: latitude, rad
        lon: longitude, rad
    '''
    return rot_y(-math.pi/2.0 - lat).dot(rot_z(lon))

def three_axis_rot(r11, r12, r21, r31, r32):
    r1 = math.atan2(r11, r12)
    r2 = math.asin(r21)
    r3 = math.atan2(r31, r32)
    return r1, r2, r3

def two_axis_rot(r11, r12, r21, r31, r32):
    r1 = math.atan2(r11, r12)
    r2 = math.acos(r21)
    r3 = math.atan2(r31, r32)
    return np.array([r1, r2, r3])

def rot_x(angle):
    """
    Coordinate transformation matrix from the original frame to the frame after
    rotation when rotating about x axis
    Args:
        angle: rotation angle, rad
    Returns:
        rx: 3x3 orthogonal matrix
    """
    sangle = math.sin(angle)
    cangle = math.cos(angle)
    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cangle, sangle],
                   [0.0, -sangle, cangle]])
    return rx

def rot_y(angle):
    """
    Coordinate transformation matrix from the original frame to the frame after
    rotation when rotating about y axis
    Args:
        angle: rotation angle, rad
    Returns:
        ry: 3x3 orthogonal matrix
    """
    sangle = math.sin(angle)
    cangle = math.cos(angle)
    ry = np.array([[cangle, 0.0, -sangle],
                   [0.0, 1.0, 0.0],
                   [sangle, 0.0, cangle]])
    return ry

def rot_z(angle):
    """
    Coordinate transformation matrix from the original frame to the frame after
    rotation when rotating about z axis
    Args:
        angle: rotation angle, rad
    Returns:
        rz: 3x3 orthogonal matrix
    """
    sangle = math.sin(angle)
    cangle = math.cos(angle)
    rz = np.array([[cangle, sangle, 0.0],
                   [-sangle, cangle, 0.0],
                   [0.0, 0.0, 1.0]])
    return rz

def quat_update(q, w, dt):
    '''
    Args:
        q: quaternion, scalar first.
        w: angular velocity, rad/s.
        dt: sample period, sec.
    Returns:
        q: updated quaternion.
    '''
    rot_quat = rotation_quat(w, dt)
    q = quat_multiply(q, rot_quat)
    q = quat_normalize(q)
    return q

def euler_update_zyx(x, w, dt):
    '''
    Update Euler angles.
    Rotation sequence is zyx [phi theta psi], rad.
    Args:
        x: input Euler angles, rad
        w: angular velocity, rad/s
        dt: time interval, sec
    Returns:
        y: propagate x according to angular velocity w for dt seconds
    '''
    # Euler angle rate
    c_psi = math.cos(x[2])
    s_psi = math.sin(x[2])
    phi_dot = (w[2]*c_psi + w[1]*s_psi) / math.cos(x[1])
    theta_dot = w[1]*c_psi - w[2]*s_psi
    psi_dot = w[0] + (w[2]*c_psi + w[1]*s_psi) * math.tan(x[1])

    y = np.copy(x)          # y = x will cause x modified when modifying y
    y[0] += phi_dot * dt
    y[1] += theta_dot * dt
    y[2] += psi_dot * dt
    # limit value
    # pitch is more complicated
    if y[1] > HALF_PI:
        y[1] = math.pi - y[1]
        y[0] = y[0] + math.pi
        y[2] = y[2] + math.pi
    elif y[1] < -HALF_PI:
        y[1] = -math.pi - y[1]
        y[0] = y[0] + math.pi
        y[2] = y[2] + math.pi
    # yaw is within [-pi, pi]
    if y[0] > math.pi:
        y[0] = y[0] - TWO_PI
    elif y[0] < -math.pi:
        y[0] = y[0] + TWO_PI
    # roll is within [-pi, pi]
    if y[2] > math.pi:
        y[2] = y[2] - TWO_PI
    elif y[2] < -math.pi:
        y[2] = y[2] + TWO_PI
    return y

def rotation_quat(w, dt):
    '''
    Args:
        w: angular velocity, rad/s.
        dt: sample period, sec.
    Returns:
        rot_quat: rotation quaternion corresponds to w and dt
    '''
    rot_vec = w * dt                    # rotation vector
    theta = math.sqrt(np.dot(rot_vec, rot_vec))    # rotation angle
    half_theta = 0.5 * theta            # half rotation angle
    s = math.sin(half_theta)
    c = math.cos(half_theta)
    if theta == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    elif c >= 0:
        tmp = s / theta
        return np.array([c, tmp*rot_vec[0], tmp*rot_vec[1], tmp*rot_vec[2]])
    else:
        tmp = -s / theta
        return np.array([-c, tmp*rot_vec[0], tmp*rot_vec[1], tmp*rot_vec[2]])

def get_cross_mtx(a):
    '''
    x = cross(a, b) = a_cross * b. This function generate a_cross from a.
    Args:
        a: (3,) array.
    Returns:
        a_cross: (3,3) matrix
    '''
    a_cross = np.array([[0.0, -a[2], a[1]],
                        [a[2], 0.0, -a[0]],
                        [-a[1], a[0], 0.0]])
    return a_cross

def cross3(a, b):
    '''
    cross product of array of size 3.
    Args:
        a: array of size 3.
        b: array of size 3.
    Returns:
        c: c = cross(a,b), of size 3.
    '''
    c = np.array([a[1]*b[2] - a[2]*b[1],
                  a[2]*b[0] - a[0]*b[2],
                  a[0]*b[1] - a[1]*b[0]])
    return c

def euler_angle_range_three_axis(angles):
    '''
    Limit Euler angle range.
    For three-axis rotation, the angle ranges are [-pi, pi], [-pi/2, pi/2] and [-pi, pi]
    For two-axis rotation, the angle ranges are [-pi, pi], [0, pi] and [-pi, pi]
    Args:
        angles: numpy array of (3,) or (3,1)
        rot_seq: only 'ZYX' is supported now.
    '''
    half_pi = 0.5 * math.pi
    # convert the second angle in range [-pi, pi]
    a1 = angles[0]
    a2 = angle_range_pi(angles[1])
    a3 = angles[2]
    # the second angle is not within [-pi/2, pi/2]?
    if a2 > half_pi:
        a2 = math.pi - a2
        a1 = a1 + math.pi
        a3 = a3 + math.pi
    elif a2 < -half_pi:
        a2 = -math.pi - a2
        a1 = a1 + math.pi
        a3 = a3 + math.pi
    a1 = angle_range_pi(a1)
    a3 = angle_range_pi(a3)
    return np.array([a1, a2, a3])

def angle_range_pi(x):
    '''
    Limit angle range within [-pi, pi]
    Args：
        x: rad
    Return:
        equivalent angle of x, [-pi, pi], rad
    '''
    # [0, 2pi]
    x = x % (TWO_PI)
    # [-pi, pi]
    if x > math.pi:
        x = x - TWO_PI
    return x


# def angle_range_180(x):
#     '''
#     Limit angle range within [-180, 180]
#     Args：
#         x: deg
#     Return:
#         equivalent angle of x, [-180, 180], deg
#     '''
#     # [0, 360]
#     x = x % 360.0
#     # [-180, 180]
#     if x > 180.0:
#         x = x - 360.0
#     return x
