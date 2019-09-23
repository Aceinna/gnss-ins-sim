# -*- coding: utf-8 -*-
# Filename: demo_free_integration.py

"""
Demonstrate free integration performance using logged data.
Created on 2018-12-03
@author: dongxiaoguang
"""

import os
import math
import numpy as np
from gnss_ins_sim.attitude import attitude
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# log_dir = "./demo_data_files/nxp/"
log_dir = "./demo_data_files/bosch/"
fs = 100.0          # IMU sample frequency
using_external_g = True

def test_free_integration():
    '''
    test Sim
    '''
    #### do not need an IMU model
    imu = None

    #### Algorithm
    from demo_algorithms import free_integration
    ini_pos_vel_att = np.genfromtxt(log_dir+"ini.txt", delimiter=',')
    ini_pos_vel_att[0:2] *= attitude.D2R  # For Lat and Lon, deg to rad
    ini_pos_vel_att[6:9] *= attitude.D2R  # Attitude from deg to rad
    if not using_external_g:
        ini_pos_vel_att = ini_pos_vel_att[0:9]
    # create the algorithm object
    algo = free_integration.FreeIntegration(ini_pos_vel_att, earth_rot=False)

    from demo_algorithms import inclinometer_acc
    algo2 = inclinometer_acc.TiltAcc()

    #### start simulation
    sim = ins_sim.Sim([fs, 0.0, 0.0],
                      log_dir,
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=[algo, algo2])
    # run the simulation
    sim.run(1)
    # generate simulation results, summary
    sim.results('', err_stats_start=-1, extra_opt='ned')
    # plot
    sim.plot(['pos', 'vel', 'att_euler', 'accel', 'gyro'],
             opt={'pos':'error', 'vel':'error', 'att_euler':'error'})

if __name__ == '__main__':
    test_free_integration()
