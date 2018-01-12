# -*- coding: utf-8 -*-
# Filename: free_integration.py

"""
IMU free integration in a virtual inertial frame.
This is a demo algorithm for Sim.
Created on 2017-12-20
@author: dongxiaoguang
"""

import numpy as np
from attitude import attitude
from geoparams import geoparams

class FreeIntegration(object):
    '''
    Integrate gyro to get attitude, double integrate linear acceleration to get position.
    '''
    def __init__(self, ini_pos_vel_att):
        '''
        Args:
            ini_pos_vel_att: 9x1 initial position, velocity and attitude.
                3x1 position in the form of LLA, units: [rad, rad, m];
                3x1 velocity in the body frame, units: m/s;
                3x1 Euler anels [yaw, pitch, roll], rotation sequency is ZYX, rad.
        '''
        # algorithm description
        self.input = ['fs', 'gyro', 'accel']
        self.output = ['att_euler', 'pos', 'vel']
        self.batch = True
        self.results = None
        # algorithm vars
        self.dt = 1.0
        self.att = None
        self.pos = None
        self.vel = None
        self.vel_b = None
        # ini state
        self.r0 = geoparams.lla2xyz(ini_pos_vel_att[0:3])
        self.v0 = ini_pos_vel_att[3:6]
        self.att0 = ini_pos_vel_att[6:9]
        # Earth gravity
        earth_param = geoparams.geo_param(ini_pos_vel_att[0:3])    # geo parameters
        self.g_n = np.array([0, 0, earth_param[2]])

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
        # Free IMU integration
        self.att = np.zeros((n, 3))
        self.pos = np.zeros((n, 3))
        self.vel = np.zeros((n, 3))
        self.vel_b = np.zeros((n, 3))
        c_bn = np.eye((3))
        for i in range(n):
            #### initialize
            if i == 0:
                self.att[i, :] = self.att0
                self.pos[i, :] = self.r0
                self.vel_b[i, :] = self.v0
                c_bn = attitude.euler2dcm(self.att[i, :])
                self.vel[i, :] = c_bn.T.dot(self.v0)
                continue
            #### propagate Euler angles
            self.att[i, :] = attitude.euler_update_zyx(self.att[i-1, :], gyro[i-1, :], self.dt)
            #### propagate velocity in the navigation frame
            # accel_n = c_nb.dot(accel[i-1, :])
            # self.vel[i, :] = self.vel[i-1, :] + (accel_n + self.g_n) * self.dt
            #### propagate velocity in the body frame
            # c_bn here is from last loop (i-1), and used to project gravity
            self.vel_b[i, :] = self.vel_b[i-1, :] +\
                             (accel[i-1, :] + c_bn.dot(self.g_n)) * self.dt -\
                             attitude.cross3(gyro[i-1, :], self.vel_b[i-1, :]) * self.dt
            # c_bn (i)
            c_bn = attitude.euler2dcm(self.att[i, :])
            self.vel[i, :] = c_bn.T.dot(self.vel_b[i, :])   # velocity in navigation frame
            self.pos[i, :] = self.pos[i-1, :] + self.vel[i-1, :] * self.dt

        # results
        self.results = [self.att, self.pos, self.vel_b]

    def get_results(self):
        '''
        return algorithm results as specified in self.output
        '''
        return self.results

    def reset(self):
        '''
        Reset the fusion process to uninitialized state.
        '''
        pass
