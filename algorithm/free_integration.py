# -*- coding: utf-8 -*-
# Filename: free_integration.py

"""
IMU free integration.
This is a demo algorithm for Sim.
Created on 2017-12-20
@author: dongxiaoguang
"""

import numpy as np
from attitude import attitude

class FreeIntegration(object):
    '''
    Integrate gyro to get attitude, double integrate linear acceleration to get position.
    '''
    def __init__(self):
        '''
        vars
        '''
        # algorithm description
        self.input = ['fs', 'gyro', 'accel']
        self.output = ['att_quat', 'pos', 'vel']
        self.batch = True
        self.results = None
        # algorithm vars
        self.ini = 0
        self.dt = 1.0
        self.q = None
        self.pos = None
        self.vel = None

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
        n = accel.shape[0]
        # calculate
        self.q = np.zeros((n, 4))
        self.pos = np.zeros((n, 3))
        self.vel = np.zeros((n, 3))
        for i in range(n):
            pass
        # results
        self.results = [self.q, self.pos, self.vel]

    def get_results(self):
        '''
        return algorithm results as specified in self.output
        '''
        return self.results

    def reset(self):
        '''
        Reset the fusion process to uninitialized state.
        '''
        self.ini = False
