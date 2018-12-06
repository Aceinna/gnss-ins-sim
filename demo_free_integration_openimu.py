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

log_dir = "E:/Projects/python-imu380-mult/free_integration_data/"
fs = 100.0          # IMU sample frequency
using_external_g = True

def test_free_integration():
    '''
    test Sim
    '''
    #### do not need an IMU model
    imu = None

    #### Algorithm
    # Free integration in a virtual inertial frame
    from demo_algorithms import free_integration
    ini_pos_vel_att = np.genfromtxt(log_dir+"ini.txt")
    # add initial states error if needed
    ini_vel_err = np.array([0.0, 0.0, 0.0]) # initial velocity error in the body frame, m/s
    ini_att_err = np.array([0.0, 0.0, 0.0]) # initial Euler angles error, deg
    ini_pos_vel_att[0:3] = ini_pos_vel_att[0:3] * attitude.D2R
    ini_pos_vel_att[3:6] += ini_vel_err
    ini_pos_vel_att[6:9] = (ini_pos_vel_att[6:9] + ini_att_err) * attitude.D2R
    if not using_external_g:
        ini_pos_vel_att = ini_pos_vel_att[0:9]
    # create the algorithm object
    algo = free_integration.FreeIntegration(ini_pos_vel_att)

    #### start simulation
    sim = ins_sim.Sim([fs, 0.0, 0.0],
                      log_dir,
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=algo)
    # run the simulation
    sim.run(1)
    # generate simulation results, summary
    sim.results('', end_point=False)
    # plot
    sim.plot(['pos', 'vel', 'att_euler', 'accel'],\
             opt={'pos':'error', 'vel':'error', 'att_euler': 'error'},\
             extra_opt={'pos':'ned', 'mpl_opt':''})

if __name__ == '__main__':
    test_free_integration()
