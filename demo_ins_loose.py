# -*- coding: utf-8 -*-
# Filename: demo_free_integration.py

"""
A demo of a loosely coupled INS algorithm.
Created on 2018-05-24
@author: dongxiaoguang
"""

import os
import math
import numpy as np
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0          # IMU sample frequency
fs_gps = 10         # GPS sample frequency

def test_ins_loose():
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
    # do not generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=True)

    #### Algorithm
    # loosely couple INS algorithm
    from demo_algorithms import ins_loose
    algo = ins_loose.InsLoose()

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, 0.0],
                      motion_def_path+"//motion_def-90deg_turn.csv",
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=algo)
    # run the simulation for 1000 times
    sim.run()
    # generate simulation results, summary
    # do not save data since the simulation runs for 1000 times and generates too many results
    sim.results(end_point=True)
    sim.plot(['ref_pos', 'pos'], opt={'ref_pos': '3d'})

if __name__ == '__main__':
    print('Still under development. Please try demo_aceinna_ins.py.')
    # test_ins_loose()
