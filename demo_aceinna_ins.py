# -*- coding: utf-8 -*-
# Filename: demo_aceinna-ins.py

"""
An inclinometer demo.
Created on 2018-10-31
@author: dongxiaoguang
"""

import os
import math
import numpy as np
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# globals
D2R = math.pi/180.0

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 200.0          # IMU sample frequency

def test_dmu380_sim():
    '''
    test Sim with DMU380 algorithm.
    '''
    #### choose a built-in IMU model, typical for IMU380
    imu_err = {'gyro_b': np.array([1.0, -1.0, 0.5]) * 1800.0,
               'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0,
               'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0,
               'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
               'accel_b': np.array([5.0e-3, 5.0e-3, 5.0e-3]) * 1.0,
               'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0,
               'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0,
               'accel_b_corr': np.array([200.0, 200.0, 200.0]),
               'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0
              }
    gps_err = {'stdp': np.array([5.0, 5.0, 7.0]) * 1.0,
               'stdv': np.array([0.05, 0.05, 0.05]) * 1.0}
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True, gps_opt=gps_err)

    #### Algorithm
    # DMU380 algorithm
    from demo_algorithms import aceinna_ins
    cfg_file = os.path.abspath('.//demo_algorithms//dmu380_sim_lib//ekfSim_ins.cfg')
    algo = aceinna_ins.DMU380Sim(cfg_file)

    #### start simulation
    sim = ins_sim.Sim([fs, fs, fs],
                      motion_def_path+"//motion_def-long_drive.csv",
                    #   ".//demo_saved_data//car_test_20180929//",
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,#'[0.1 0.01 0.11]g-random',
                      algorithm=algo)
    sim.run(1)
    # generate simulation results, summary, and save data to files
    sim.results('', gen_kml=True)  # do not save data
    # plot data
    sim.plot(['pos', 'vel', 'att_euler'], opt={'pos':'error', 'att_euler':'error'})

if __name__ == '__main__':
    test_dmu380_sim()
