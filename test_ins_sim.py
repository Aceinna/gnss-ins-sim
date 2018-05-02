# -*- coding: utf-8 -*-
# Filename: test_ins_sim.py

"""
Test ins_sim.
Created on 2018-04-23
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

    #### Algorithm
    # ECF based inclinometer
    from demo_algorithms import inclinometer_mahony
    algo = inclinometer_mahony.MahonyFilter()

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"//motion_def.csv",
                    #   'e://Projects//gnss-ins-sim//demo_saved_data//2018-04-24-14-55-53//',
                      ref_frame=1,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=algo)
    sim.run(3)
    # save simulation data to files
    sim.results('.//demo_saved_data//')
    # plot data, 3d plot of reference positoin, 2d plots of gyro and accel
    sim.plot(['att_euler'], opt={'att_euler': 'error'})

if __name__ == '__main__':
    test_path_gen()
