# -*- coding: utf-8 -*-
# Fielname = ins_loose.py

"""
Loosely coupled INS algorithm.
Created on 2018-05-24
@author: dongxiaoguang
"""

# import
import math
import numpy as np
from gnss_ins_sim.attitude import attitude
from gnss_ins_sim.geoparams import geoparams

UNINI = 0           # uninitialized
ATTITUDE_INI = 1    # attitude initialized
POS_INI = 2         # position and velocity initialized

class InsLoose(object):
    '''
    Loosely coupled INS algorithm.
    '''
    def __init__(self):
        '''
        vars
        '''
        # algorithm description
        self.input = ['fs', 'gyro', 'accel', 'time', 'gps_time', 'gps']#, 'mag']
        self.output = ['pos', 'vel', 'att_euler', 'wb', 'ab']
        self.batch = True
        self.results = None
        # algorithm vars
        self.ini = 0                                # indicate if attitude is initialized
        self.dt = 1.0                               # sample period, sec
        self.q = np.array([1.0, 0.0, 0.0, 0.0])     # quaternion

    def run(self, set_of_input):
        '''
        main procedure of the algorithm
        Args:
            set_of_input is a tuple or list consistent with self.input
        '''
        # get input
        fs = set_of_input[0]
        gyro = set_of_input[1]
        accel = set_of_input[2]
        time = set_of_input[3]
        gps_time = set_of_input[4]
        gps = set_of_input[5]
        # run the algorithm
        self.ins_loose(fs, time, gps_time, gyro, accel, gps)

    def ins_loose(self, fs, time, gps_time, gyro, accel, gps):
        '''
        main procedure of the loosely couple INS algorithm.
        Initialization strategy:
            1. Collect first n accel samples to initialize roll and pitch.
            2. If GPS is available during collecting the n samples, position and velocity
                are initialized.
            3. If GPS is not availabe during collecting the n samples, position and velocity
                will be initialzied when GPS becomes available. Before position and velocity
                are initialized, system runs in free integration mode.
        '''
        # Kalman filter state and matrices
        x = np.zeros((15, 1))
        F = np.eye(15)
        Q = np.zeros((15, 15))
        H = np.eye(15)
        R = np.zeros((6, 1))
        # KF
        dt = 1.0 /fs
        n = time.shape[0]
        samples_for_attitude_ini = 10
        average_accel = np.zeros((3,))
        i_gps_time = 0  # index of current GPS time, GPS data for this time are not used yet.
        for i in range(n):
            # None of attitude, position and velocity is initialized.
            if self.ini == UNINI:
                # average accel for initial pitch and roll
                average_accel += accel[i, :]
                if i == samples_for_attitude_ini - 1:
                    average_accel /= 10.0
                    accel_norm = math.sqrt(average_accel[0]*average_accel[0] +\
                                           average_accel[1]*average_accel[1] +\
                                           average_accel[2]*average_accel[2])
                    average_accel /= accel_norm
                    euler_zyx = np.zeros((3,))  # Euler angles, ZYX
                    euler_zyx[0] = 10.0 * attitude.D2R
                    euler_zyx[1] = math.asin(average_accel[0])
                    euler_zyx[2] = math.atan2(-average_accel[1], -average_accel[2])
                    # find if there is any valid GPS measurement during attitude initialization.
                    # if there is, using the latest one to initialize position and velocity.
                    while time[i] > gps_time[i_gps_time+1]:
                        i_gps_time += 1
                    if gps_time[i_gps_time] > time[i]:
                        self.ini = ATTITUDE_INI
                        pos = np.zeros((3,))
                        vel = np.zeros((3,))
                    else:
                        self.ini = POS_INI
                        pos = gps[i_gps_time, 0:3]
                        vel = gps[i_gps_time, 3:6]
                        i_gps_time += 1 # this GPS measurement is used, index moves to next

            # Attitude is initialized, to initialize position and velocity
            elif self.ini == ATTITUDE_INI:
                # try to iniialize position and velocity
                if time[i] > gps_time[i_gps_time]:
                    # propagate to gps_time
                    self.prediction(gyro[i-1, :], accel[i-1, :], gps_time[i_gps_time]-time[i-1])
                    # initialize position and velocity
                    pos = gps[i_gps_time, 0:3]
                    vel = gps[i_gps_time, 3:6]
                    # propagate to current time
                    self.prediction(gyro[i-1, :], accel[i-1, :], time[i]-gps_time[i_gps_time])
                    self.ini = POS_INI
                    i_gps_time += 1
                else:
                    self.prediction(gyro[i-1, :], accel[i-1, :], dt)

            # attitude, position and velocity are all initialized
            elif self.ini == POS_INI:
                pass
            else:
                self.ini = UNINI

    def prediction(self, gyro, acc, dt):
        '''
        Kalman prediction
        '''
        pass

    def correction(self, gps):
        '''
        Kalman correction
        '''
        pass

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
