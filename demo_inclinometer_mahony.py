# -*- coding: utf-8 -*-
# Filename: demo_inclinometer_mahony.py

"""
An inclinometer demo.
Created on 2018-01-23
@author: dongxiaoguang
"""

import os
import math
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import imu_sim

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0          # IMU sample frequency

def test_inclinometer_mahony():
    '''
    test Sim with a inclinometer algorithm based on Mahony's theory.
    '''
    #### choose a built-in IMU model, typical for IMU380
    imu_err = 'low-accuracy'
    # do not generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=False)

    #### Algorithm
    # ECF based inclinometer
    from demo_algorithms import inclinometer_mahony
    algo = inclinometer_mahony.MahonyFilter()

    #### start simulation
    sim = imu_sim.Sim([fs, 0.0, 0.0], imu, motion_def_path+"//motion_def.csv",
                      ref_frame=1,
                      mode=None,
                      env=None,
                      algorithm=algo)
    sim.run()
    # generate simulation results, summary, and save data to files
    sim.results()  # do not save data
    # plot data
    sim.plot(['att_euler', 'ref_att_euler'], opt={'att_euler': 'error'})

if __name__ == '__main__':
    test_inclinometer_mahony()
