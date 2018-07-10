# -*- coding: utf-8 -*-
# Filename: demo_mag_cal.py

"""
The simplest demo of soft iron and hard iron calibration.
Created on 2018-07-09
@author: dongxiaoguang
"""

import os
import math
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_saved_data//')
fs = 100.0          # IMU sample frequency
fs_gps = 10.0       # GPS sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now

def test_mag_cal():
    '''
    test only path generation in Sim.
    '''
    print("This demo only runs on Ubuntu x64.")
    #### choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=False)

    #### Algorithm
    from demo_algorithms import mag_calibrate
    algo = mag_calibrate.MagCal()

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"//test_mag_cal//",
                      ref_frame=1,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=algo)
    sim.run(1)
    # save simulation data to files
    sim.results()
    # plot data
    sim.plot(['mag', 'mag_cal'], opt={'mag': '3d', 'mag_cal': '3d'})

if __name__ == '__main__':
    test_mag_cal()
