# -*- coding: utf-8 -*-
# Fielname = imu_model.py

"""
IMU class.
Created on 2017-12-19
@author: dongxiaoguang
"""

import math
import numpy as np

D2R = math.pi/180

## default IMU, magnetometer and GPS error profiles.
# low accuracy, from AHRS380
#http://www.memsic.cn/userfiles/files/Datasheets/Inertial-System-Datasheets/AHRS380SA_Datasheet.pdf
gyro_low_accuracy = {'b': np.array([0.0, 0.0, 0.0]) * D2R,
                     'b_drift': np.array([10.0, 10.0, 10.0]) * D2R/3600.0,
                     'b_corr':np.array([100.0, 100.0, 100.0]),
                     'arw': np.array([0.75, 0.75, 0.75]) * D2R/60.0}
accel_low_accuracy = {'b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
                      'b_drift': np.array([2.0e-4, 2.0e-4, 2.0e-4]),
                      'b_corr': np.array([100.0, 100.0, 100.0]),
                      'vrw': np.array([0.05, 0.05, 0.05]) / 60.0}
mag_low_accuracy = {'si': np.eye(3) + np.random.randn(3, 3)*0.0,
                    'hi': np.array([10.0, 10.0, 10.0])*0.0,
                    'std': np.array([0.1, 0.1, 0.1])}
# mid accuracy, partly from IMU381
gyro_mid_accuracy = {'b': np.array([0.0, 0.0, 0.0]) * D2R,
                     'b_drift': np.array([3.5, 3.5, 3.5]) * D2R/3600.0,
                     'b_corr':np.array([100.0, 100.0, 100.0]),
                     'arw': np.array([0.25, 0.25, 0.25]) * D2R/60}
accel_mid_accuracy = {'b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
                      'b_drift': np.array([5.0e-5, 5.0e-5, 5.0e-5]),
                      'b_corr': np.array([100.0, 100.0, 100.0]),
                      'vrw': np.array([0.03, 0.03, 0.03]) / 60}
mag_mid_accuracy = {'si': np.eye(3) + np.random.randn(3, 3)*0.0,
                    'hi': np.array([10.0, 10.0, 10.0])*0.0,
                    'std': np.array([0.01, 0.01, 0.01])}
# high accuracy, partly from HG9900, partly from
# http://www.dtic.mil/get-tr-doc/pdf?AD=ADA581016
gyro_high_accuracy = {'b': np.array([0.0, 0.0, 0.0]) * D2R,
                      'b_drift': np.array([0.1, 0.1, 0.1]) * D2R/3600.0,
                      'b_corr':np.array([100.0, 100.0, 100.0]),
                      'arw': np.array([2.0e-3, 2.0e-3, 2.0e-3]) * D2R/60}
accel_high_accuracy = {'b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
                       'b_drift': np.array([3.6e-6, 3.6e-6, 3.6e-6]),
                       'b_corr': np.array([100.0, 100.0, 100.0]),
                       'vrw': np.array([2.5e-5, 2.5e-5, 2.5e-5]) / 60}
mag_high_accuracy = {'si': np.eye(3) + np.random.randn(3, 3)*0.0,
                     'hi': np.array([10.0, 10.0, 10.0])*0.0,
                     'std': np.array([0.001, 0.001, 0.001])}

## built-in GPS error profiles
gps_low_accuracy = {'stdp': np.array([5.0, 5.0, 7.0]),
                    'stdv': np.array([0.05, 0.05, 0.05])}

## built-in odometer error profiles
odo_low_accuracy = {'scale': 0.99,
                    'stdv': 0.1}
class IMU(object):
    '''
    IMU class
    '''

    def __init__(self, accuracy='low-accuracy', axis=6,\
                 gps=True, gps_opt=None,\
                 odo=False, odo_opt=None):
        '''
        Args:
            accuracy: IMU grade.
                This can be a string to use the built-in IMU models:
                    'low-accuracy':
                    'mid-accuracy':
                    'high-accuracy':
                or a dictionary to custom the IMU model:
                    'gyro_b': gyro bias, deg/hr
                    'gyro_arw': gyro angle random walk, deg/rt-hr
                    'gyro_b_stability': gyro bias instability, deg/hr
                    'gyro_b_corr': gyro bias isntability correlation time, sec
                    'accel_b': accel bias, m/s2
                    'accel_vrw' : accel velocity random walk, m/s/rt-hr
                    'accel_b_stability': accel bias instability, m/s2
                    'accel_b_corr': accel bias isntability correlation time, sec
                    'mag_si': soft iron, default is a 3x3 identity matrix.
                    'mag_hi': hard iron, default is 3x1 zero vector.
                    'mag_std': mag noise std.
                If magnetometer is enabled and you want to specify the magnetic params,
                soft iron and hard iron are optional and noise std is needed. If soft
                iron or hard iron is not provided, defautls will be used.
            axis: 6 for IMU, 9 for IMU+magnetometer
            gps: True if GPS exists, False if not.
            gps_opt: a dictionary to specify the GPS error model.
                'stdp': position RMS error, meters
                'stdv': vertical RMS error, meters/second
            odo: True if odometer exists, False if not.
            odo_opt: a dictionary to specify the odometer error model.
                'scale': scale factor
                'stdv': velocity measurement noise, meters/second.
        '''
        # check axis
        self.magnetometer = False
        if axis == 9:
            self.magnetometer = True
        elif axis != 6:
            raise ValueError('axis should be either 6 or 9.')

        # built-in imu error model
        self.gyro_err = gyro_low_accuracy                   #   default is low accuracy
        self.accel_err = accel_low_accuracy
        self.mag_err = mag_low_accuracy

        # accuracy is a string, use built-in models
        if isinstance(accuracy, str):
            if accuracy == 'low-accuracy':                  # 'low-accuracy'
                pass
            elif accuracy == 'mid-accuracy':                # 'mid-accuracy'
                self.gyro_err = gyro_mid_accuracy
                self.accel_err = accel_mid_accuracy
                self.mag_err = mag_mid_accuracy
            elif accuracy == 'high-accuracy':               # 'high-accuracy'
                self.gyro_err = gyro_high_accuracy
                self.accel_err = accel_high_accuracy
                self.mag_err = mag_high_accuracy
            else:                                           # not a valid string
                raise ValueError('accuracy is not a valid string.')
        # accuracy is a dict, user defined models
        elif isinstance(accuracy, dict):
            # set IMU and mag error params
            if 'gyro_b' in accuracy and\
               'gyro_b_stability' in accuracy and\
               'gyro_arw' in accuracy and\
               'accel_b' in accuracy and\
               'accel_b_stability' in accuracy and\
               'accel_vrw' in accuracy:                 # check keys in accuracy
                # required parameters
                self.gyro_err['b'] = accuracy['gyro_b'] * D2R / 3600.0
                self.gyro_err['b_drift'] = accuracy['gyro_b_stability'] * D2R / 3600.0
                self.gyro_err['arw'] = accuracy['gyro_arw'] * D2R / 60.0
                self.accel_err['b'] = accuracy['accel_b']
                self.accel_err['b_drift'] = accuracy['accel_b_stability']
                self.accel_err['vrw'] = accuracy['accel_vrw'] /60.0
                if self.magnetometer:   # at least noise std should be specified
                    if 'mag_std' in accuracy:
                        self.mag_err['std'] = accuracy['mag_std']
                    else:
                        raise ValueError('Magnetometer is enabled, ' +\
                                         'but its noise std is not specified.')
                # optional parameters
                if 'gyro_b_corr' in accuracy:
                    self.gyro_err['b_corr'] = accuracy['gyro_b_corr']
                else:
                    self.gyro_err['b_corr'] = np.array([float("inf"), float("inf"), float("inf")])
                if 'accel_b_corr' in accuracy:
                    self.accel_err['b_corr'] = accuracy['accel_b_corr']
                else:
                    self.accel_err['b_corr'] = np.array([float("inf"), float("inf"), float("inf")])
                if 'mag_si' in accuracy:
                    self.mag_err['si'] = accuracy['mag_si']
                else:
                    self.mag_err['si'] = np.eye(3)
                if 'mag_hi' in accuracy:
                    self.mag_err['hi'] = accuracy['mag_hi']
                else:
                    self.mag_err['hi'] = np.array([0.0, 0.0, 0.0])
            # raise error when required keys are missing
            else:
                raise ValueError('accuracy should at least have keys: \n' +\
                                 'gyro_b, gyro_b_stability, gyro_arw, ' +\
                                 'accel_b, accel_b_stability and accel_vrw')
        else:
            raise TypeError('accuracy is not valid.')

        # build GPS model
        if gps:
            self.gps = True
            if gps_opt is None:
                self.gps_err = gps_low_accuracy
            elif isinstance(gps_opt, dict):
                if 'stdp' in gps_opt and 'stdv' in gps_opt:
                    self.gps_err = gps_opt
                else:
                    raise ValueError('gps_opt should have key: stdp and stdv')
            else:
                raise TypeError('gps_opt should be None or a dict')
        else:
            self.gps = False
            self.gps_err = None

        # build odometer model
        if odo:
            self.odo = True
            if odo_opt is None:
                self.odo_err = odo_low_accuracy
            elif isinstance(odo_opt, dict):
                if 'scale' in odo_opt and 'stdv' in odo_opt:
                    self.odo_err = odo_opt
                else:
                    raise ValueError('odo_opt should have key: scale and stdv')
            else:
                raise TypeError('odo_opt should be None or a dict')
        else:
            self.odo = False
            self.odo_err = None

    def set_gyro_error(self, gyro_error='low-accuracy'):
        '''
        set gyro error model
        Args:
            gyro_error: gyro error model.
                This can be a string to use the built-in gyro models:
                    'low-accuracy':
                    'mid-accuracy':
                    'high-accuracy':
                or a dictionary to custom the IMU model:
                    'b': gyro bias, deg/hr
                    'arw': gyro angle random walk, deg/rt-hr
                    'b_drift': gyro bias instability, deg/hr
                    'b_corr': gyro bias isntability correlation time, sec
        '''
        if isinstance(gyro_error, str):
            if gyro_error == 'low-accuracy':                  # 'low-accuracy'
                self.gyro_err = gyro_low_accuracy
            elif gyro_error == 'mid-accuracy':                # 'mid-accuracy'
                self.gyro_err = gyro_mid_accuracy
            elif gyro_error == 'high-accuracy':               # 'high-accuracy'
                self.gyro_err = gyro_high_accuracy
            else:                                           # not a valid string
                raise ValueError('gyro_error is not a valid string.')
        # accuracy is a dict, user defined models
        elif isinstance(gyro_error, dict):
            for i in gyro_error:
                if i in self.gyro_err:
                    self.gyro_err[i] = gyro_error[i]
                else:
                    raise ValueError('unsupported key: %s in gyro_error'% i)
        else:
            raise TypeError('gyro_error is not valid.')

    def set_accel_error(self, accel_error='low-accuracy'):
        '''
        set accel error model
        Args:
            accel_error: accel error model.
                This can be a string to use the built-in accel models:
                    'low-accuracy':
                    'mid-accuracy':
                    'high-accuracy':
                or a dictionary to custom the IMU model:
                    'b': accel bias, m/s2
                    'vrw' : accel velocity random walk, m/s/rt-hr
                    'b_drift': accel bias instability, m/s2
                    'b_corr': accel bias isntability correlation time, sec
        '''
        if isinstance(accel_error, str):
            if accel_error == 'low-accuracy':                  # 'low-accuracy'
                self.accel_err = accel_low_accuracy
            elif accel_error == 'mid-accuracy':                # 'mid-accuracy'
                self.accel_err = accel_mid_accuracy
            elif accel_error == 'high-accuracy':               # 'high-accuracy'
                self.accel_err = accel_high_accuracy
            else:                                           # not a valid string
                raise ValueError('accel_error is not a valid string.')
        # accuracy is a dict, user defined models
        elif isinstance(accel_error, dict):
            for i in accel_error:
                if i in self.accel_err:
                    self.accel_err[i] = accel_error[i]
                else:
                    raise ValueError('unsupported key: %s in accel_error'% i)
        else:
            raise TypeError('accel_error is not valid.')

    def set_gps(self, gps_error=None):
        '''
        set GPS error model
        Args:
            gps_error: GPS error model
                This can be either none or a dictionary to specify the GPS error model:
                    'stdp': position RMS error, meters
                    'stdv': velocity RMS error, meters/second
        '''
        if not self.gps:
            return
        if gps_error is None:
            self.gps_err = gps_low_accuracy
        elif isinstance(gps_error, dict):
            if 'stdp' in gps_error and\
                'stdv' in gps_error:
                self.gps_err = gps_error
            else:
                raise ValueError('gps_error should have key: stdp and stdv')
        else:
            raise TypeError('gps_error should be None or a dict')

    def set_odo(self, odo_error=None):
        '''
        set odometer error model
        Args:
            odo_error: odometer error model
                This can be either none or a dictionary to specify the odometer error model:
                    'scale': scale factor
                    'stdv': velocity RMS error, meters/second
        '''
        if not self.gps:
            return
        if odo_error is None:
            self.odo_err = odo_low_accuracy
        elif isinstance(odo_error, dict):
            if 'stdp' in odo_error and\
                'stdv' in odo_error:
                self.gps_err = odo_error
            else:
                raise ValueError('odo_error should have key: stdp and stdv')
        else:
            raise TypeError('odo_error should be None or a dict')

    def set_mag_error(self, mag_error='low-accuracy'):
        '''
        set magnetometer error model
        Args:
            mag_error: magnetometer error model.
                This can be a string to use the built-in mag models:
                    'low-accuracy':
                    'mid-accuracy':
                    'high-accuracy':
                or a dictionary to custom the IMU model:
                    'si': soft iron, default is a 3x3 identity matrix.
                    'hi': hard iron, default is 3x1 zero vector.
                    'std': mag noise std.
        '''
        if not self.magnetometer:
            return
        if isinstance(mag_error, str):
            if mag_error == 'low-accuracy':                  # 'low-accuracy'
                self.mag_err = mag_low_accuracy
            elif mag_error == 'mid-accuracy':                # 'mid-accuracy'
                self.mag_err = mag_mid_accuracy
            elif mag_error == 'high-accuracy':               # 'high-accuracy'
                self.mag_err = mag_high_accuracy
            else:                                           # not a valid string
                raise ValueError('mag_error is not a valid string.')
        # accuracy is a dict, user defined models
        elif isinstance(mag_error, dict):
            for i in mag_error:
                if i in self.mag_err:
                    self.mag_err[i] = mag_error[i]
                else:
                    raise ValueError('unsupported key: %s in mag_error'% i)
        else:
            raise TypeError('mag_error is not valid.')
