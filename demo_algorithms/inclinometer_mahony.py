# -*- coding: utf-8 -*-
# Fielname = inclinometer_mahony.py

"""
IMU fusion.
Created on 2017-09-27
@author: dongxiaoguang
"""

# import
import math
import numpy as np
from gnss_ins_sim.attitude import attitude

# globals
VERSION = '1.0'

class MahonyFilter(object):
    '''
    Mahony filter.
    '''
    def __init__(self):
        '''
        vars
        '''
        # algorithm description
        self.input = ['fs', 'gyro', 'accel']#, 'mag']
        self.output = ['att_quat', 'wb', 'ab']
        self.batch = True
        self.results = None
        self.quat = None
        self.wb = None
        self.ab = None
        # algorithm vars
        # config
        self.innovationLimit = 0.1
        self.kp_acc_high = 1
        self.kp_acc_low = 0.01
        self.ki_acc_high = 0.5
        self.ki_acc_low = 0.001
        # state
        self.ini = 0                                # indicate if attitude is initialized
        self.dt = 1.0                               # sample period, sec
        self.q = np.array([1.0, 0.0, 0.0, 0.0])     # quaternion
        self.err_int = np.array([0.0, 0.0, 0.0])    # integral of error
        self.kp_acc = 1
        self.ki_acc = 0.001
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.tmp = np.array([0.0, 0.0, 0.0])

    def run(self, set_of_input):
        '''
        main procedure of the algorithm
        Args:
            set_of_input is a tuple or list consistent with self.input
        '''
        # get input
        self.dt = 1.0 / set_of_input[0]
        gyro = set_of_input[1]
        accel = set_of_input[2]
        # mag = set_of_input[3]
        n = accel.shape[0]
        # calculate
        self.quat = np.zeros((n, 4))
        self.wb = np.zeros((n, 3))
        self.ab = np.zeros((n, 3))
        for i in range(n):
            self.update(gyro[i, :], accel[i, :])
            # generate qaternion, must be a tuple or list consistent with self.output
            self.quat[i, :] = self.q
            self.wb[i, :] = self.gyro_bias
            self.ab[i, :] = self.tmp

    def update(self, gyro, acc, mag=np.array([0.0, 0.0, 0.0])):
        '''
        Mahony filter for gyro, acc and mag.
        Args:
        Returns:
        '''
        mag_valid = (mag[0] != 0.0) or (mag[1] != 0.0) or (mag[2] != 0.0)
        acc_valid = (acc[0] != 0.0) or (acc[1] != 0.0) or (acc[2] != 0.0)
        # dynamic mode to be added here
        if math.fabs(math.sqrt(np.dot(acc, acc)) - 9.8) > 0.2 or\
           math.sqrt(np.dot(gyro, gyro)) > 0.2:
            self.kp_acc = self.kp_acc_low
            self.ki_acc = self.ki_acc_low
            # print("low")
        else:
            self.kp_acc = self.kp_acc_high
            self.ki_acc = self.ki_acc_high
            # print("high")
        # normalize acc
        if acc_valid:
            acc = acc / math.sqrt(np.dot(acc, acc))
        # initialize attitude
        if (not self.ini) and acc_valid:
            self.ini = 1
            self.err_int[0] = 0.0
            self.err_int[1] = 0.0
            self.err_int[2] = 0.0
            pseudo_mag = np.array([0.0, 0.0, 0.0])
            if acc[0] >= 1.0:
                pseudo_mag[0] = 0.0
                pseudo_mag[1] = 0.0
                pseudo_mag[2] = 1.0
            elif acc[1] <= -1.0:
                pseudo_mag[0] = 0.0
                pseudo_mag[1] = 0.0
                pseudo_mag[2] = -1.0
            else:
                pseudo_mag[0] = math.sqrt(1.0 - acc[0]*acc[0])
                pseudo_mag[1] = -acc[1] * acc[0] / pseudo_mag[0]
                pseudo_mag[2] = -acc[0] * acc[2] / pseudo_mag[0]
            cn2b = attitude.get_cn2b_acc_mag_ned(acc, pseudo_mag)
            self.q = attitude.dcm2quat(cn2b)
        # mag is not valid, acc+gyro fusion
        if not mag_valid:
            self.update_imu(gyro, acc)

    def update_imu(self, gyro, acc):
        '''
        Mahony filter for gyro and acc.
        Args:
            gyro: 3, rad/s
            acc: 3, normalizaed acc
        Returns:
        '''
        # unit gravity vector from gyro propagation
        q1 = self.q
        v = np.array([0.0, 0.0, 0.0])
        v[0] = -2.0 * (q1[1]*q1[3] - q1[0]*q1[2])
        v[1] = -2.0 * (q1[0]*q1[1] + q1[2]*q1[3])
        v[2] = -q1[0]*q1[0] + q1[1]*q1[1] + q1[2]*q1[2] - q1[3]*q1[3]
        acc_err = np.cross(acc, v)
        acc_err_norm = math.sqrt(np.dot(acc_err, acc_err))
        if acc_err_norm > self.innovationLimit:
            acc_err = acc_err / acc_err_norm * self.innovationLimit
        # for i in range(0, 3):                   # limite error
        #     if math.fabs(acc_err[i]) > self.innovationLimit:
        #         acc_err[i] = np.sign(acc_err[i]) * self.innovationLimit
        #print(acc_err)
        # integral of the error
        self.err_int = self.err_int + self.ki_acc * acc_err * self.dt
        # gyro correction
        k = 0.9
        this_gyro_bias = self.kp_acc * acc_err + self.err_int
        self.gyro_bias = k*self.gyro_bias + (1-k)*this_gyro_bias
        self.tmp = acc_err
        gyro = gyro + self.gyro_bias
        # quaternion update
        self.q = attitude.quat_update(self.q, gyro, self.dt)

    def get_results(self):
        '''
        return algorithm results as specified in self.output
        '''
        return [self.quat, self.wb, self.ab]

    def reset(self):
        '''
        Reset the fusion process to uninitialized state.
        '''
        self.ini = 0
