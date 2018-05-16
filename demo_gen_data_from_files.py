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

def test_gen_data_from_files():
    '''
    test only path generation in Sim.
    '''
    #### choose a built-in IMU model, typical for IMU381
    imu = None

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      'e://Projects//gnss-ins-sim//demo_saved_data//2018-05-16-14-23-34//',
                      ref_frame=1,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run()
    # save simulation data to files
    sim.results()
    # plot data, 3d plot of reference positoin, 2d plots of gyro and accel
    sim.plot(['ref_att_euler', 'att_euler'], opt={'att_euler': 'error'})

if __name__ == '__main__':
    test_gen_data_from_files()
