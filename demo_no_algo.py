# -*- coding: utf-8 -*-
# Filename: demo_no_algo.py

"""
The simplest demo of Sim.
Only generate reference trajectory (pos, vel, sensor output). No algorithm.
Created on 2018-01-23
@author: dongxiaoguang
"""

import os
import math
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0          # IMU sample frequency
fs_gps = 10.0       # GPS sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now

def test_path_gen():
    '''
    test only path generation in Sim.
    '''
    #### choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"//motion_def-3d.csv",
                      ref_frame=1,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run(1)
    # save simulation data to files
    sim.results('')
    # plot data, 3d plot of reference positoin, 2d plots of gyro and accel
    sim.plot(['ref_pos', 'gyro', 'gps_visibility'], opt={'ref_pos': '3d'})

if __name__ == '__main__':
    test_path_gen()
