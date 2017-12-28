# -*- coding: utf-8 -*-
# Filename: test_path_gen.py

"""
Test pathgen.path_gen
Created on 2017-09-14
@author: dongxiaoguang
"""

import os
import math
import numpy as np
from sim import imu_model
from sim import imu_sim
from algorithm import allan_analysis

# globals
VERSION = '1.0'
D2R = math.pi/180

data_path = os.path.abspath('.//data//')
fs = 100.0           # IMU sample frequency
fs_gps = 10.0        # GPS sample frequency
ref_frame = 0       # 0: NED frame, 1: virtual inertial frame

def test_sim():
    '''
    test sim
    '''
    imu_err = {'gyro_b': np.array([0.0, 0.0, 0.0]),
               'gyro_arw': np.array([0.52142, 0.64938, 0.73193]),
               'gyro_b_stability': np.array([5.34, 9.40, 6.57]),
               'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
               'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
               'accel_vrw': np.array([0.03119, 0.03009, 0.04779]),
               'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]),
               'accel_b_corr': np.array([200.0, 200.0, 200.0]),
               'mag_std': np.array([0.2, 0.2, 0.2])
              }
    # imu_err = 'mid-accuracy'

    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)
    algo = allan_analysis.Allan()
    sim = imu_sim.Sim([fs, fs_gps, fs], imu, data_path+"//motion_def-static.csv",
                      mode=np.array([1.0, 0.5, 2.0]),
                      algorithm=algo)
    sim.run(10)
    sim.results(data_dir='.//data//')
    sim.plot(['time', 'av_gyro', 'av_accel'])
    print('test sim OK.')

if __name__ == '__main__':
    test_sim()
