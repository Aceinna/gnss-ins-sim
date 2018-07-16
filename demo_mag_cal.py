# -*- coding: utf-8 -*-
# Filename: demo_mag_cal.py

"""
The simplest demo of soft iron and hard iron calibration.
Created on 2018-07-09
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

def test_mag_cal():
    '''
    test soft iron and hard iron calibration.
    '''
    print("This demo only runs on Ubuntu x64.")
    #### IMU model, typical for IMU381
    imu_err = 'mid-accuracy'
    # do not generate GPS data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=False)
    mag_error = {'si': np.eye(3) + np.random.randn(3, 3)*0.1,
                 'hi': np.array([10.0, 10.0, 10.0])*1.0
                }
    imu.set_mag_error(mag_error)
    #### Algorithm
    from demo_algorithms import mag_calibrate
    algo = mag_calibrate.MagCal()

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"//motion_def_mag_cal.csv",
                    #   motion_def_path+"//test_mag_cal//",
                      ref_frame=1,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=algo)
    sim.run(1)
    # save simulation data to files
    sim.results()
    # plot data
    sim.plot(['mag', 'mag_cal'], opt={'mag': 'projection', 'mag_cal': 'projection'}, extra_opt='.')
    # show calibration params:
    print('true soft iron is:')
    print(np.linalg.inv(mag_error['si']))
    print('estimated soft iron is:')
    print(sim.dmgr.soft_iron.data)
    print('true hard iron is:')
    print(mag_error['hi'])
    print('estimated hard iron is:')
    print(sim.dmgr.hard_iron.data)

if __name__ == '__main__':
    test_mag_cal()
