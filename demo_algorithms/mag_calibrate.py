# -*- coding: utf-8 -*-
# Fielname = dmu380_offline_sim.py

"""
IMU fusion.
Created on 2018-03-15
@author: dongxiaoguang
"""

# import
import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from ctypes import *

# globals
VERSION = '1.0'

class MagCal(object):
    '''
    A wrapper for soft iron and hard iron calibration.
    To calibrate mag, one should rotate the mag sensor in a uniformly-distributed magnetic field
    about its x, y and z axis, respectively. During rotation, log the mag sensor data. The logged
    data from the rotations will be used to calculate the soft iron and hard iron calibration
    parameters: si and hi. The calibration parameters should be used in the following way:
    mag_calibrated = si * mag_raw - hi
    '''
    def __init__(self):
        '''
        '''
        # algorithm description
        self.input = ['mag']
        self.output = ['soft_iron', 'hard_iron', 'mag_cal']
        self.batch = True
        self.results = None
        # algorithm vars
        this_dir = os.path.dirname(__file__)
        self.algo_lib = os.path.join(this_dir, 'mag_calibrate_lib/libmagcal.so')
        if not os.path.exists(self.algo_lib):
            if not self.build_lib():
                raise OSError('Shared libs not found.')
        self.sim_engine = cdll.LoadLibrary(self.algo_lib)
        # initialize algorithm


    def run(self, set_of_input):
        '''
        main procedure of the algorithm
        Args:
            set_of_input is a tuple or list consistent with self.input
                mag: numpy array of size (n,3). It contains mag data collected during rotating
                    the sensor about its x, y and z axis, respectively.
        '''
        # get input
        mag = set_of_input[0].copy()    # mag will changed, "copy" to avoid changing input data
        # plot mag data and get x/y/z rotating start/end index
        plt.plot(mag)
        plt.grid(True)

        plt.show(block=False)
        x0 = input('Please input start index of rotation about x axis:')
        xf = input('Please input end index of rotation about x axis:')
        y0 = input('Please input start index of rotation about y axis:')
        yf = input('Please input end index of rotation about y axis:')
        z0 = input('Please input start index of rotation about z axis:')
        zf = input('Please input end index of rotation about z axis:')
        x0 = int(x0)
        xf = int(xf)
        y0 = int(y0)
        yf = int(yf)
        z0 = int(z0)
        zf = int(zf)
        # run
        # calculate soft iron and hard iron
        si = np.zeros((3, 3))
        si_ptr = si.ctypes.data_as(POINTER(c_double))
        hi = np.zeros((1, 4))
        hi_ptr = hi.ctypes.data_as(POINTER(c_double))
        mx = mag[x0:xf, :].ctypes.data_as(POINTER(c_double))
        my = mag[y0:yf, :].ctypes.data_as(POINTER(c_double))
        mz = mag[z0:zf, :].ctypes.data_as(POINTER(c_double))
        iRowNum = np.array((xf-x0, yf-y0, zf-z0), dtype='int32')
        iRowNum_ptr = iRowNum.ctypes.data_as(POINTER(c_double))
        self.sim_engine.MagCalibrate(si_ptr, hi_ptr, mx, my, mz, iRowNum_ptr)
        # results
        mag = np.vstack([mag[x0:xf], mag[y0:yf], mag[z0:zf]])
        self.results = [si, hi, mag]

    def update(self, gyro, acc, mag=np.array([0.0, 0.0, 0.0])):
        '''
        Mahony filter for gyro, acc and mag.
        Args:
        Returns:
        '''
        pass

    def get_results(self):
        '''
        Returns:
            algorithm results as specified in self.output
                algorithm time step, sec
                Euler angles [yaw pitch roll], rad
        '''
        return self.results

    def reset(self):
        '''
        Reset the fusion process to uninitialized state.
        '''
        self.sim_engine = cdll.LoadLibrary(self.algo_lib)

    def build_lib(self, dst_dir=None, src_dir=None):
        '''
        Build shared lib
        Args:
            dst_dir: dir to put the built libs in.
            src_dir: dir containing the source code.
        Returns:
            True if success, False if error.
        '''
        this_dir = os.path.dirname(__file__)
        # get dir containing the source code
        if src_dir is None:
            src_dir = os.path.join(this_dir, '//home//dong//c_projects//mag_cal//')
        if not os.path.exists(src_dir):
            print('Source code directory ' + src_dir + ' does not exist.')
            return False
        # get dir to put the libs in
        if dst_dir is None:
            dst_dir = os.path.join(this_dir, './/mag_calibrate_lib//')
        if not os.path.exists(dst_dir):
            os.mkdir(dst_dir)

        algo_lib = 'libmagcal.so'
        # get current workding dir
        cwd = os.getcwd()
        # create the cmake dir
        cmake_dir = src_dir + '//cmake//'
        if not os.path.exists(cmake_dir):
            os.mkdir(cmake_dir)
        else:
            os.system("rm -rf " + cmake_dir + "*")
        # call cmake and make to build the libs
        os.chdir(cmake_dir)
        ret = os.system("cmake ..")
        ret = os.system("make")
        algo_lib = cmake_dir + 'lib//' + algo_lib
        if os.path.exists(algo_lib):
            os.system("mv " + algo_lib + " " + dst_dir)

        # restore working dir
        os.chdir(cwd)
        return True
