# -*- coding: utf-8 -*-
# Fielname = dmu380_offline_sim.py

"""
IMU fusion.
Created on 2018-03-15
@author: dongxiaoguang
"""

# import
import os
import struct
import platform
import math
import numpy as np
from ctypes import *
from ctypes import wintypes

# globals
VERSION = '1.0'

R2D = 180.0 / math.pi

class SIM_COMFIG(Structure):
    '''
    read config params from file, and put params in this structure
    '''
    _fields_ = [("_inputDataRate", c_bool),
                ("inputDataRate", c_uint8),
                ("_outputDataRate", c_bool),
                ("outputDataRate", c_uint8),
                ("_hasMags", c_bool),
                ("hasMags", c_bool),
                ("_useMags", c_bool),
                ("useMags", c_bool),
                ("_hasGps", c_bool),
                ("hasGps", c_bool),
                ("_useGps", c_bool),
                ("useGps", c_bool),
                ("_freeIntegrate", c_bool),
                ("freeIntegrate", c_bool),
                ("_dynamicMotion", c_bool),
                ("dynamicMotion", c_bool),
                ("_stationaryLockYaw", c_bool),
                ("stationaryLockYaw", c_bool),
                ("_turnSwitchThreshold", c_bool),
                ("turnSwitchThreshold", c_float),
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
                ("_maxGpsDropTime", c_bool),
                ("maxGpsDropTime", c_int32),
                ("_maxReliableDRTime", c_bool),
                ("maxReliableDRTime", c_int32),
                ("_procCovarMult_rateBias", c_bool),
                ("procCovarMult_rateBias", c_float),
                ("_procCovarMult_attitude", c_bool),
                ("procCovarMult_attitude", c_float),
                ("_measCovarMult_roll", c_bool),
                ("measCovarMult_roll", c_float),
                ("_measCovarMult_pitch", c_bool),
                ("measCovarMult_pitch", c_float)
                ]

class GPS_DATA(Structure):
    '''
    Input GPS data structure
    '''
    _fields_ = [("gpsFixType", c_uint8),
                ("gpsUpdate", c_uint8),
                ("numSatellites", c_uint8),
                ("itow", c_uint),
                ("latitude", c_double),
                ("longitude", c_double),
                ("vNed", c_double*3),
                ("trueCourse", c_double),
                ("rawGroundSpeed", c_double),
                ("altitude", c_double),
                ("GPSSecondFraction", c_double),
                ("GPSmonth", c_uint8),
                ("GPSday", c_uint8),
                ("GPSyear", c_uint8),
                ("GPSHour", c_char),
                ("GPSMinute", c_char),
                ("GPSSecond", c_char),
                ("GPSHorizAcc", c_float),
                ("GPSVertAcc", c_float),
                ("HDOP", c_float),
                ("geoidAboveEllipsoid", c_float),
                ]

class ODO_DATA(Structure):
    '''
    Input odometer data structure
    '''
    _fields_ = [("odoUpdate", c_uint8),
                ("v", c_float),
                ]

class EKF_STATE(Structure):
    '''
    Return EFK state in this structure
    '''
    _fields_ = [("timeStep", c_uint32),
                ("kfPosN", c_double*3),
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
                ("algoLinAccelSwitch", c_uint8),]

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
            if struct.calcsize("P") == 8:
                self.ext = '-x64' + self.ext
            else:
                self.ext = '-x86' + self.ext
        else:
            raise OSError('Only support windows.')
        # algorithm description
        self.input = ['fs', 'gyro', 'accel', 'gps', 'gps_visibility', 'time', 'gps_time', 'odo']
        self.output = ['algo_time', 'pos', 'vel', 'att_euler', 'wb', 'ab']
        self.batch = True
        self.results = None
        # algorithm vars
        this_dir = os.path.dirname(__file__)
        self.config_lib = os.path.join(this_dir, 'dmu380_sim_lib/libsim_utilities' + self.ext)
        self.sim_lib = os.path.join(this_dir, 'dmu380_sim_lib/aceinna_ins' + self.ext)
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
        idx = 0
        fs = set_of_input[idx]
        idx += 1
        gyro = set_of_input[idx]
        idx += 1
        accel = set_of_input[idx]
        idx += 1
        gps = set_of_input[idx]
        idx += 1
        gps_visibility = set_of_input[idx]
        idx += 1
        time = set_of_input[idx]
        idx += 1
        gps_time = set_of_input[idx]
        idx += 1
        odometer = set_of_input[idx]
        idx += 1
        if 'mag' in self.input:
            mag = set_of_input[idx]
        n = accel.shape[0]
        # algo output
        time_step = np.zeros((n,))
        pos = np.zeros((n, 3))
        vel = np.zeros((n, 3))
        euler_angles = np.zeros((n, 3))
        rate_bias = np.zeros((n, 3))
        accel_bias = np.zeros((n, 3))
        # run
        g = GPS_DATA()
        odo = ODO_DATA()
        ekf_state = EKF_STATE()
        output_len = 0
        idx_gps = 0
        for i in range(0, n):
            a = np.zeros((3,))
            w = np.zeros((3,))
            m = np.zeros((3,))
            a = accel[i, :] / 9.80665
            w = gyro[i, :]
            if 'mag' in self.input:
                m = mag[i, :] / 100.0
            aptr = a.ctypes.data_as(POINTER(c_double))
            wptr = w.ctypes.data_as(POINTER(c_double))
            mptr = m.ctypes.data_as(POINTER(c_double))
            # fill in GPS data
            g.gpsUpdate = 0
            if gps_time[idx_gps] == time[i]:
                # there is a GPS update
                g.gpsUpdate = 1
                g.gpsFixType = int(gps_visibility[idx_gps])
                g.latitude = gps[idx_gps, 0] * R2D
                g.longitude = gps[idx_gps, 1] * R2D
                g.altitude = gps[idx_gps, 2]
                g.vNed[0] = gps[idx_gps, 3]
                g.vNed[1] = gps[idx_gps, 4]
                g.vNed[2] = gps[idx_gps, 5]
                g.rawGroundSpeed = math.sqrt(g.vNed[0]*g.vNed[0] + g.vNed[1]*g.vNed[1])
                g.trueCourse = math.atan2(g.vNed[1], g.vNed[0]) * R2D
                # print(g.trueCourse, g.vNed[1], g.vNed[0])
                g.HDOP = 1.0
                g.GPSHorizAcc = g.HDOP * 3.0
                g.GPSVertAcc = g.GPSHorizAcc * 1.5
                g.itow = int(gps_time[idx_gps] * 1000)
                idx_gps += 1
                if idx_gps == gps_time.shape[0]:
                    idx_gps = gps_time.shape[0] - 1
            # fill in odo data
            odo.odoUpdate = 1
            odo.v = odometer[i]
            self.sim_engine.SimRun(aptr, wptr, mptr, pointer(g), pointer(odo))
            # get output
            self.sim_engine.GetEKF_STATES(pointer(ekf_state))
            # time_step[output_len] = ekf_state.timeStep / fs
            time_step[output_len] = i / fs
            # Euler angles order is [roll pitch yaw] in the algo
            # We use yaw [pitch roll yaw] order in the simulation
            pos[output_len, 0] = ekf_state.kfPosN[0] / R2D
            pos[output_len, 1] = ekf_state.kfPosN[1] / R2D
            pos[output_len, 2] = ekf_state.kfPosN[2]
            vel[output_len, 0] = ekf_state.kfVelN[0]
            vel[output_len, 1] = ekf_state.kfVelN[1]
            vel[output_len, 2] = ekf_state.kfVelN[2]
            euler_angles[output_len, 0] = ekf_state.kfEulerAngles[2]
            euler_angles[output_len, 1] = ekf_state.kfEulerAngles[1]
            euler_angles[output_len, 2] = ekf_state.kfEulerAngles[0]
            rate_bias[output_len, 0] = ekf_state.kfRateBias[0]
            rate_bias[output_len, 1] = ekf_state.kfRateBias[1]
            rate_bias[output_len, 2] = ekf_state.kfRateBias[2]
            accel_bias[output_len, 0] = ekf_state.kfAccelBias[0]
            accel_bias[output_len, 1] = ekf_state.kfAccelBias[1]
            accel_bias[output_len, 2] = ekf_state.kfAccelBias[2]
            output_len += 1
        # results
        self.results = [time_step[0:output_len],\
                        pos[0:output_len, :],\
                        vel[0:output_len, :],\
                        euler_angles[0:output_len, :],\
                        rate_bias[0:output_len, :],\
                        accel_bias[0:output_len, :]]

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
        windll.kernel32.FreeLibrary.argtypes = [wintypes.HMODULE]
        windll.kernel32.FreeLibrary(self.sim_engine._handle)
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
