# -*- coding: utf-8 -*-
# Filename: free_integration.py

"""
IMU free integration in a virtual inertial frame.
This is a demo algorithm for Sim.
Created on 2017-12-20
@author: dongxiaoguang
"""

import numpy as np
from gnss_ins_sim.attitude import attitude
from gnss_ins_sim.geoparams import geoparams

class FreeIntegration(object):
    '''
    Integrate gyro to get attitude, double integrate linear acceleration to get position.
    '''
    def __init__(self, ini_pos_vel_att, earth_rot=True):
        '''
        Args:
            ini_pos_vel_att: 9x1 numpy array containing initial position, velocity and attitude.
                3x1 position in the form of LLA, units: [rad, rad, m];
                3x1 velocity in the body frame, units: m/s;
                3x1 Euler anels [yaw, pitch, roll], rotation sequency is ZYX, rad.
            earth_rot: Consider the Earth rotation or not. Only used when ref_frame=0.
        '''
        # algorithm description
        self.input = ['ref_frame', 'fs', 'gyro', 'accel']
        self.output = ['att_euler', 'pos', 'vel']
        self.earth_rot = earth_rot
        self.batch = True
        self.results = None
        # algorithm vars
        self.ref_frame = 1
        self.dt = 1.0
        self.att = None
        self.pos = None
        self.vel = None
        self.vel_b = None
        # ini state
        self.set_of_inis = 1        # number of sets of inis
        self.run_times = int(0)     # algo run times. If run_times <= set_of_inis, the i-th run
                                    # uses the i-th set of initial states. Otherwise, the first
                                    # set of initial states will be used
        # only one set of inis is provided, multiple runs have the same inis.
        if ini_pos_vel_att.ndim == 1:
            self.set_of_inis = 1
            ini_pos_vel_att = ini_pos_vel_att.reshape((ini_pos_vel_att.shape[0], 1))
        # multiple set of inis is provided, multiple runs can have different inis.
        elif ini_pos_vel_att.ndim == 2:
            self.set_of_inis = ini_pos_vel_att.shape[1]
        else:
            raise ValueError('Initial states should be a 1D or 2D numpy array, \
                              but the dimension is %s.'% ini_pos_vel_att.ndim)
        self.r0 = ini_pos_vel_att[0:3]
        self.v0 = ini_pos_vel_att[3:6]
        self.att0 = ini_pos_vel_att[6:9]
        self.gravity = None
        if len(ini_pos_vel_att) > 9:
            self.gravity = ini_pos_vel_att[9]

    def run(self, set_of_input):
        '''
        main procedure of the algorithm
        Args:
            set_of_input is a tuple or list consistent with self.input
        '''
        self.run_times += 1
        # get input
        if set_of_input[0] == 0:
            self.ref_frame = 0
        self.dt = 1.0 / set_of_input[1]
        gyro = set_of_input[2]
        accel = set_of_input[3]
        n = accel.shape[0]
        # Free IMU integration
        self.att = np.zeros((n, 3))
        self.pos = np.zeros((n, 3))
        self.vel = np.zeros((n, 3))     # NED vel
        self.vel_b = np.zeros((n, 3))   # body vel
        c_bn = np.eye((3))
        if self.ref_frame == 1:
            # which set of initial states to use
            idx = self.run_times - 1
            if self.run_times > self.set_of_inis:
                idx = 0
            # Earth gravity
            if self.gravity is None:
                earth_param = geoparams.geo_param(self.r0)    # geo parameters
                g_n = np.array([0, 0, earth_param[2]])
            else:
                g_n = np.array([0, 0, self.gravity[idx]])
            for i in range(n):
                #### initialize
                if i == 0:
                    self.att[i, :] = self.att0[:, idx]
                    self.pos[i, :] = geoparams.lla2ecef(self.r0[:, idx])
                    self.vel_b[i, :] = self.v0[:, idx]
                    c_bn = attitude.euler2dcm(self.att[i, :])
                    self.vel[i, :] = c_bn.T.dot(self.vel_b[i, :])
                    continue
                #### propagate Euler angles
                self.att[i, :] = attitude.euler_update_zyx(self.att[i-1, :], gyro[i-1, :], self.dt)
                #### propagate velocity in the navigation frame
                # accel_n = c_nb.dot(accel[i-1, :])
                # self.vel[i, :] = self.vel[i-1, :] + (accel_n + g_n) * self.dt
                #### propagate velocity in the body frame
                # c_bn here is from last loop (i-1), and used to project gravity
                self.vel_b[i, :] = self.vel_b[i-1, :] +\
                                (accel[i-1, :] + c_bn.dot(g_n)) * self.dt -\
                                attitude.cross3(gyro[i-1, :], self.vel_b[i-1, :]) * self.dt
                # c_bn (i)
                c_bn = attitude.euler2dcm(self.att[i, :])
                self.vel[i, :] = c_bn.T.dot(self.vel_b[i, :])   # velocity in navigation frame
                self.pos[i, :] = self.pos[i-1, :] + self.vel[i-1, :] * self.dt
        else:
            # which set of initial states to use
            idx = self.run_times - 1
            if self.run_times > self.set_of_inis:
                idx = 0
            w_en_n = np.zeros(3)
            w_ie_n = np.zeros(3)
            for i in range(n):
                #### initialize
                if i == 0:
                    self.att[i, :] = self.att0[:, idx]
                    self.pos[i, :] = self.r0[:, idx]
                    self.vel_b[i, :] = self.v0[:, idx]
                    c_bn = attitude.euler2dcm(self.att[i, :])
                    self.vel[i, :] = c_bn.T.dot(self.vel_b[i, :])
                    continue
                #### geo parameters
                earth_param = geoparams.geo_param(self.pos[i-1, :])
                rm = earth_param[0]
                rn = earth_param[1]
                g = earth_param[2]
                sl = earth_param[3]
                cl = earth_param[4]
                w_ie = earth_param[5]
                rm_effective = rm + self.pos[i-1, 2]
                rn_effective = rn + self.pos[i-1, 2]
                if self.gravity is None:
                    g_n = np.array([0, 0, g])
                else:
                    g_n = np.array([0, 0, self.gravity[idx]])
                w_en_n[0] = self.vel[i-1, 1] / rn_effective              # wN
                w_en_n[1] = -self.vel[i-1, 0] / rm_effective             # wE
                w_en_n[2] = -self.vel[i-1, 1] * sl /cl / rn_effective    # wD
                if self.earth_rot:
                    w_ie_n[0] = w_ie * cl
                    w_ie_n[2] = -w_ie * sl
                #### propagate euler angles
                w_nb_b = gyro[i-1, :] - c_bn.dot(w_en_n + w_ie_n)
                self.att[i, :] = attitude.euler_update_zyx(self.att[i-1, :], w_nb_b, self.dt)
                #### propagate velocity
                # vel_dot_b = accel[i-1, :] + c_bn.T.dot(g_n) -\
                #             attitude.cross3(c_bn.dot(w_ie_n)+gyro[i-1,:], self.vel_b[i-1,:])
                # self.vel_b[i,:] = self.vel_b[i-1,:] + vel_dot_b*self.dt
                vel_dot_n = c_bn.T.dot(accel[i-1, :]) + g_n -\
                            attitude.cross3(2*w_ie_n + w_en_n, self.vel[i-1, :])
                self.vel[i, :] = self.vel[i-1, :] + vel_dot_n * self.dt
                #### propagate position
                lat_dot = self.vel[i-1, 0] / rm_effective
                lon_dot = self.vel[i-1, 1] / rn_effective / cl
                alt_dot = -self.vel[i-1, 2]
                self.pos[i, 0] = self.pos[i-1, 0] + lat_dot * self.dt
                self.pos[i, 1] = self.pos[i-1, 1] + lon_dot * self.dt
                self.pos[i, 2] = self.pos[i-1, 2] + alt_dot * self.dt
                #### output
                c_bn = attitude.euler2dcm(self.att[i, :])
                self.vel_b[i, :] = c_bn.dot(self.vel[i, :])
        # results
        self.results = [self.att, self.pos, self.vel]

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
