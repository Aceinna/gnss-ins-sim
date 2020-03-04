# -*- coding: utf-8 -*-
# Filename: test_ins_sim.py

"""
Test ins_sim.
Created on 2018-04-23
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
fs_gps = 10.0       # GPS sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now

def gen_data_first(data_dir):
    '''
    Generate data that will be used by test_gen_data_from_files()
    '''
    # imu model
    imu = imu_model.IMU(accuracy='mid-accuracy', axis=6, gps=False)

    # start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"//motion_def-90deg_turn.csv",
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run(10)
    # save simulation data to files
    sim.results(data_dir)

def test_gen_data_from_files(data_dir):
    '''
    test data generation from files.
    '''
    #### start simulation
    #### Algorithm
    # Free integration in a virtual inertial frame
    from demo_algorithms import free_integration
    '''
    Free integration requires initial states (position, velocity and attitude). You should provide
    theses values when you create the algorithm object.
    '''
    ini_pos_vel_att = np.genfromtxt(motion_def_path+"//motion_def-90deg_turn.csv",\
                                    delimiter=',', skip_header=1, max_rows=1)
    ini_pos_vel_att[0] = ini_pos_vel_att[0] * D2R
    ini_pos_vel_att[1] = ini_pos_vel_att[1] * D2R
    ini_pos_vel_att[6:9] = ini_pos_vel_att[6:9] * D2R
    # add initial states error if needed
    ini_vel_err = np.array([0.0, 0.0, 0.0]) # initial velocity error in the body frame, m/s
    ini_att_err = np.array([0.0, 0.0, 0.0]) # initial Euler angles error, deg
    ini_pos_vel_att[3:6] += ini_vel_err
    ini_pos_vel_att[6:9] += ini_att_err * D2R
    # create the algorith object
    algo = free_integration.FreeIntegration(ini_pos_vel_att)

    #### start simulation
    sim = ins_sim.Sim([fs, 0.0, 0.0],
                      data_dir,
                      ref_frame=0,
                      imu=None,
                      mode=None,
                      env=None,
                      algorithm=algo)
    # run the simulation for 1000 times
    sim.run(1)
    # generate simulation results, summary
    sim.results('', err_stats_start=-1, gen_kml=True)
    sim.plot(['att_euler'])

if __name__ == '__main__':
    dir_of_logged_files = os.path.abspath('.//demo_saved_data//tmp//')
    gen_data_first(dir_of_logged_files)
    test_gen_data_from_files(dir_of_logged_files)
