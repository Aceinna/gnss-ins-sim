# -*- coding: utf-8 -*-
# Filename: allan_analysis.py

"""
Allan variance analysis.
This is a demo algorithm for Sim.
Reference: O.J. Woodman. An introduction to inertial navigation.
Created on 2017-12-20
@author: dongxiaoguang
"""

# import
import math
import numpy as np
from allan import allan
D2R = math.pi/180

class Allan(object):
    '''
    Allan var. A demo algorithm for Sim
    '''
    def __init__(self):
        '''
        algorithm description
        '''
        self.input = ['fs', 'accel', 'gyro']
        self.output = ['av_t', 'av_accel', 'av_gyro']
        self.batch = True   # Put all data from t0 to tf if True (default)
                            # Sequentially put data from t0 to tf if False
        self.results = None

    def run(self, set_of_input):
        '''
        main procedure of the algorithm
        Args:
            set_of_input is a tuple or list consistent with self.input
        '''
        # get input
        fs = set_of_input[0]
        accel = set_of_input[1]
        gyro = set_of_input[2]
        # calculate
        avar_ax, tau = allan.allan_var(accel[:, 0]/9.8, fs)
        avar_ay = allan.allan_var(accel[:, 1]/9.8, fs)[0]
        avar_az = allan.allan_var(accel[:, 2]/9.8, fs)[0]
        avar_wx = allan.allan_var(gyro[:, 0]/D2R, fs)[0]
        avar_wy = allan.allan_var(gyro[:, 1]/D2R, fs)[0]
        avar_wz = allan.allan_var(gyro[:, 2]/D2R, fs)[0]
        # generate results, must be a tuple or list consistent with self.output
        self.results = tau,\
                       np.array([avar_ax, avar_ay, avar_az]).T,\
                       np.array([avar_wx, avar_wy, avar_wz]).T

    def get_results(self):
        '''
        return algorithm results as specified in self.output
        '''
        return self.results
