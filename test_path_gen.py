# -*- coding: utf-8 -*-
# Filename: test_path_gen.py

"""
Test pathgen.path_gen
Created on 2017-09-14
@author: dongxiaoguang
"""

import os
import math
import numpy as np
from sim import imu_model
from sim import imu_sim

# globals
VERSION = '1.0'
D2R = math.pi/180

data_path = os.path.abspath('.//sim_def_files//')
fs = 100.0          # IMU sample frequency
fs_gps = 10.0       # GPS sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now
# ref_frame = 0       # 0: NED frame, 1: virtual inertial frame

def test_sim():
    '''
    test Sim
    '''
    #### IMU model, typical for IMU381
    imu_err = {'gyro_b': np.array([0.0, 0.0, 0.0]),
               'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0,
               'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0,
               'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
               'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
               'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0,
               'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0,
               'accel_b_corr': np.array([200.0, 200.0, 200.0]),
               'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0
              }
    # imu_err = 'high-accuracy'
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    #### Algorithm
    # Allan analysis
    # from algorithm import allan_analysis
    # algo = allan_analysis.Allan()

    # ECF based inclinometer
    # from algorithm import inclinometer_mahony
    # algo = inclinometer_mahony.MahonyFilter()

    # Free integration in a virtual inertial frame
    from algorithm import free_integration
    ini_pos_vel_att = np.genfromtxt(data_path+"//motion_def-90deg_turn.csv",\
                                    delimiter=',', skip_header=1, max_rows=1)
    ini_pos_vel_att[0] = ini_pos_vel_att[0] * D2R
    ini_pos_vel_att[1] = ini_pos_vel_att[1] * D2R
    ini_pos_vel_att[6:9] = ini_pos_vel_att[6:9] * D2R
    ini_vel_err = np.array([0.0, 0.0, 0.0]) # initial velocity error in the body frame, m/s
    ini_att_err = np.array([0.0, 0.0, 0.0]) # initial Euler angles error, deg
    ini_pos_vel_att[3:6] += ini_vel_err
    ini_pos_vel_att[6:9] += ini_att_err * D2R
    algo = free_integration.FreeIntegration(ini_pos_vel_att)

    #### start simulation
    sim = imu_sim.Sim([fs, fs_gps, fs_mag], imu, data_path+"//motion_def-90deg_turn.csv",
                      ref_frame=1,
                      mode=np.array([1.0, 0.5, 2.0]),
                      env=None,#np.genfromtxt(data_path + '//vib_psd.csv', delimiter=','),
                      #env=np.genfromtxt(data_path+'//vib_psd.csv', delimiter=',', skip_header=1),
                      algorithm=algo)
    sim.run(2000)
    # generate simulation results, summary, and save data to files
    # sim.results('./data/')  # save data files
    sim.results()  # do not save data
    # plot data
    # sim.plot(['ref_pos', 'gyro'], opt={'ref_pos': '3d'})
    print('test sim OK.')

if __name__ == '__main__':
    test_sim()
