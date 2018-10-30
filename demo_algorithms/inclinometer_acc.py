# -*- coding: utf-8 -*-
# Fielname = inclinometer_acc.py

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

class TiltAcc(object):
    '''
    Tilt sensor using only accelerometer.
    '''
    def __init__(self):
        '''
        vars
        '''
        # algorithm description
        self.name = 'StaticTilt'
        self.input = ['accel']#, 'mag']
        self.output = ['att_quat']
        self.batch = True
        self.results = None
        # algorithm vars
        self.ini = 0                                # indicate if attitude is initialized
        self.q = np.array([1.0, 0.0, 0.0, 0.0])     # quaternion
        self.err_int = np.array([0.0, 0.0, 0.0])    # integral of error
        self.kp_acc = 0.1
        self.ki_acc = 0.001

    def run(self, set_of_input):
        '''
        main procedure of the algorithm
        Args:
            set_of_input is a tuple or list consistent with self.input
        '''
        # get input
        accel = set_of_input[0]
        # mag = set_of_input[3]
        n = accel.shape[0]
        # calculate
        self.results = np.zeros((n, 4))
        mag = np.array([1.0, 0.0, 0.0])
        for i in range(n):
            # generate results, must be a tuple or list consistent with self.output
            dcm = attitude.get_cn2b_acc_mag_ned(accel[i, :], mag)
            self.q = attitude.dcm2quat(dcm)
            self.results[i, :] = self.q

    def get_results(self):
        '''
        return algorithm results as specified in self.output
        '''
        return [self.results]

    def reset(self):
        '''
        Reset the fusion process to uninitialized state.
        '''
        self.ini = 0
