# -*- coding: utf-8 -*-
# Filename: demo_dmu380_sim.py

"""
An inclinometer demo.
Created on 2018-03-16
@author: dongxiaoguang
"""

import os
import math
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import imu_sim

# globals
D2R = math.pi/180.0

motion_def_path = os.path.abspath('.//demo_motion_def_files//motion_def//')
fs = 200.0          # IMU sample frequency

def test_dmu380_sim():
    '''
    test Sim with DMU380 algorithm.
    '''
    print("This demo only runs on Ubuntu x64.")
    #### choose a built-in IMU model, typical for IMU380
    imu_err = 'mid-accuracy'
    # do not generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=False)

    #### Algorithm
    # DMU380 algorithm
    from demo_algorithms import dmu380_sim
    cfg_file = os.path.abspath('.//demo_algorithms//dmu380_sim_lib//ekfSim_tilt.cfg')
    algo = dmu380_sim.DMU380Sim(cfg_file)

    #### start simulation
    sim = imu_sim.Sim([fs, 0.0, fs], imu,
                      motion_def_path+"//motion_def-Komatsu_loaded_-5_0.csv",
                      ref_frame=1,
                      mode=None,
                      env=None,
                      algorithm=algo)
    sim.run(10)
    # generate simulation results, summary, and save data to files
    sim.results()  # do not save data
    # plot data
    sim.plot(['att_euler'])

if __name__ == '__main__':
    test_dmu380_sim()
