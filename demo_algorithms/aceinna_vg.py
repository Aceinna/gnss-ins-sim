# -*- coding: utf-8 -*-
# Fielname = dmu380_offline_sim.py

"""
IMU fusion.
Created on 2018-03-15
@author: dongxiaoguang
"""

# import
import os
import platform
import math
import numpy as np
from ctypes import *

# globals
VERSION = '1.0'

R2D = 180.0 / math.pi

class SIM_COMFIG(Structure):
    '''
    read config params from file, and put params in this structure
    '''
    _fields_ = [("_pktType", c_bool),
                ("pktType", c_uint8),
                ("_inputDataRate", c_bool),
                ("inputDataRate", c_uint8),
                ("_outputDataRate", c_bool),
                ("outputDataRate", c_uint8),
                ("_rsType", c_bool),
                ("rsTypeStr", c_char * 64),
                ("_dmuVersion", c_bool),
                ("dmuVersionStr", c_char * 64),
                ("_hasMags", c_bool),
                ("hasMags", c_bool),
                ("_useMags", c_bool),
                ("useMags", c_bool),
                ("_hasGps", c_bool),
                ("hasGps", c_bool),
                ("_useGps", c_bool),
                ("useGps", c_bool),
                ("_algorithm", c_bool),
                ("algorithm", c_bool),
                ("_freeIntegrate", c_bool),
                ("freeIntegrate", c_bool),
                ("_dynamicMotion", c_bool),
                ("dynamicMotion", c_bool),
                ("_stationaryLockYaw", c_bool),
                ("stationaryLockYaw", c_bool),
                ("_turnSwitchThreshold", c_bool),
                ("turnSwitchThreshold", c_ushort),
                ("_hardIron_X", c_bool),
                ("hardIron_X", c_float),
                ("_hardIron_Y", c_bool),
                ("hardIron_Y", c_float),
                ("_softIronScaleRatio", c_bool),
                ("softIronScaleRatio", c_float),
                ("_softIronAngle", c_bool),
                ("softIronAngle", c_float),
                ("_headingTrackOffset", c_bool),
                ("headingTrackOffset", c_ushort),
                ("_accelLPFType", c_bool),
                ("accelLPFTypeStr", c_char * 64),
                ("_accelSwitch", c_bool),
                ("accelSwitch", c_float),
                ("_origLlinAcelSwitch", c_bool),
                ("origLlinAcelSwitch", c_bool),
                ("_linAccelSwitchDelay", c_bool),
                ("linAccelSwitchDelay", c_float),
                ("_Free_Integration_Cntr", c_bool),
                ("Free_Integration_Cntr", c_float),
                ("_Stabilize_System", c_bool),
                ("Stabilize_System", c_float),
                ("_Initialize_Attitude", c_bool),
                ("Initialize_Attitude", c_float),
                ("_High_Gain_AHRS", c_bool),
                ("High_Gain_AHRS", c_float),
                ("_Low_Gain_AHRS", c_bool),
                ("Low_Gain_AHRS", c_float),
                ("_Max_GPS_Drop_Time", c_bool),
                ("Max_GPS_Drop_Time", c_int32),
                ("_suppressDisgnosticMsgs", c_bool),
                ("suppressDisgnosticMsgs", c_bool),
                ("_procCovarMult_rateBias", c_bool),
                ("procCovarMult_rateBias", c_float),
                ("_procCovarMult_attitude", c_bool),
                ("procCovarMult_attitude", c_float),
                ("_measCovarMult_roll", c_bool),
                ("measCovarMult_roll", c_float),
                ("_measCovarMult_pitch", c_bool),
                ("measCovarMult_pitch", c_float)]

class EKF_STATE(Structure):
    '''
    Return EFK state in this structure
    '''
    _fields_ = [("timeStep", c_uint32),
                ("kfPosN", c_float*3),
                ("kfVelN", c_float*3),
                ("kfQuat", c_float*4),
                ("kfRateBias", c_float*3),
                ("kfAccelBias", c_float*3),
                ("kfCorrectedRateB", c_float*3),
                ("kfCorrectedAccelB", c_float*3),
                ("algoFilteredYawRate", c_float),
                ("kfEulerAngles", c_float*3),
                ("algoState", c_int),
                ("algoTurnSwitch", c_ushort),
                ("algoLinAccelSwitch", c_uint8),
                ("algoAMag", c_float),
                ("algoAFiltN", c_float * 3),]

class DMU380Sim(object):
    '''
    A wrapper form DMU380 algorithm offline simulation.
    '''
    def __init__(self, config_file):
        '''
        Args:
            config_file: a configuration file
        '''
        self.ext = '.so'
        # platform
        if platform.system() == 'Windows':
            self.ext = '.dll'
            if '64' in platform.architecture()[0]:
                self.ext = '-x64' + self.ext
            else:
                self.ext = '-x86' + self.ext
        # algorithm description
        self.input = ['fs', 'gyro', 'accel']
        self.output = ['algo_time', 'att_euler', 'wb']
        self.batch = True
        self.results = None
        # algorithm vars
        this_dir = os.path.dirname(__file__)
        self.config_lib = os.path.join(this_dir, 'dmu380_sim_lib/libsim_utilities' + self.ext)
        self.sim_lib = os.path.join(this_dir, 'dmu380_sim_lib/aceinna_vg' + self.ext)
        if not (os.path.exists(self.config_lib) and os.path.exists(self.sim_lib)):
            if not self.build_lib():
                raise OSError('Shared libs not found.')
        self.parse_config = cdll.LoadLibrary(self.config_lib)
        self.sim_engine = cdll.LoadLibrary(self.sim_lib)
        # initialize algorithm
        self.sim_config = SIM_COMFIG()
        self.parse_config.parseConfigFile(c_char_p(config_file.encode('utf-8')),\
                                          pointer(self.sim_config))
        self.sim_engine.SimInitialize(pointer(self.sim_config))
        # if mag required?
        if self.sim_config.hasMags and self.sim_config.useMags:
            self.input.append('mag')

    def run(self, set_of_input):
        '''
        main procedure of the algorithm
        Args:
            set_of_input is a tuple or list consistent with self.input
                fs: sample frequency, Hz
                gyro: numpy array of size (n,3), rad/s
                accel: numpy array of size (n,3), m/s/s
                mag: numpy array of size (n,3), Gauss
        '''
        # get input
        fs = set_of_input[0]
        gyro = set_of_input[1]
        accel = set_of_input[2]
        if 'mag' in self.input:
            mag = set_of_input[3]
        n = accel.shape[0]
        # algo output
        time_step = np.zeros((n,))
        euler_angles = np.zeros((n, 3))
        rate_bias = np.zeros((n, 3))
        # run
        ekf_state = EKF_STATE()
        output_len = 0
        for i in range(0, n):
            sensor_data = np.zeros((15,))
            sensor_data[0:3] = gyro[i, :]*R2D
            sensor_data[3:6] = accel[i, :]/9.80665
            if 'mag' in self.input:
                sensor_data[6:9] = mag[i, :]/100.0
            sensorReadings = sensor_data.ctypes.data_as(POINTER(c_double))
            new_results = self.sim_engine.SimRun(sensorReadings)
            # get output
            if new_results == 1:
                self.sim_engine.GetEKF_STATES(pointer(ekf_state))
                # time_step[output_len] = ekf_state.timeStep / fs
                time_step[output_len] = i / fs
                # Euler angles order is [roll pitch yaw] in the algo
                # We use yaw [pitch roll yaw] order in the simulation
                euler_angles[output_len, 0] = ekf_state.kfEulerAngles[2]
                euler_angles[output_len, 1] = ekf_state.kfEulerAngles[1]
                euler_angles[output_len, 2] = ekf_state.kfEulerAngles[0]
                rate_bias[output_len, 0] = ekf_state.kfRateBias[0]
                rate_bias[output_len, 1] = ekf_state.kfRateBias[1]
                rate_bias[output_len, 2] = ekf_state.kfRateBias[2]
                output_len += 1
        # results
        self.results = [time_step[0:output_len],\
                        euler_angles[0:output_len, :],\
                        rate_bias[0:output_len, :]]

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
        self.sim_engine = cdll.LoadLibrary(self.sim_lib)
        self.sim_engine.SimInitialize(pointer(self.sim_config))

    def build_lib(self, dst_dir=None, src_dir=None):
        '''
        Build shared lib
        Args:
            dst_dir: dir to put the built libs in.
            src_dir: dir containing the source code.
        Returns:
            True if success, False if error.
        '''
        if self.ext == '.dll':
            print("Automatic generation of .dll is not supported.")
            return False
        this_dir = os.path.dirname(__file__)
        # get dir containing the source code
        if src_dir is None:
            src_dir = os.path.join(this_dir, '//home//dong//c_projects//dmu380_sim_src//')
        if not os.path.exists(src_dir):
            print('Source code directory ' + src_dir + ' does not exist.')
            return False
        # get dir to put the libs in
        if dst_dir is None:
            dst_dir = os.path.join(this_dir, './/dmu380_sim_lib//')
        if not os.path.exists(dst_dir):
            os.mkdir(dst_dir)

        algo_lib = 'libdmu380_algo_sim.so'
        sim_utilities_lib = 'libsim_utilities.so'
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
        algo_lib = cmake_dir + 'algo//' + algo_lib
        sim_utilities_lib = cmake_dir + 'SimUtilities//' + sim_utilities_lib
        if os.path.exists(algo_lib) and os.path.exists(sim_utilities_lib):
            os.system("mv " + algo_lib + " " + dst_dir)
            os.system("mv " + sim_utilities_lib + " " + dst_dir)

        # restore working dir
        os.chdir(cwd)
        return True
