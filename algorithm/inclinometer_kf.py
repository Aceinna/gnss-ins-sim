# -*- coding: utf-8 -*-
# Fielname = inclinometer_kf.py

"""
KF based dyanmic inclinometer algorithm.
System equations:
    b_dot = nb
    l_dot = (-1/tao)*l + nl
    g_dot = -(w_m-b-nw)xg
Measurement euqations:
    a = l - g + na
where
    b is gyro bias;
    g is Earth gravity in body frame;
    l is linear acc in body frame;
    a is accelerometer measurement;
    w_m is gyro measurement;
    nw is gyro Gaussian noise;
    nb is gyro bias Gaussian noise;
    nl is linear acceleration Gaussian noise;
    na is accelerometer Gaussian noise.
Created on 2017-10-10
@author: dongxiaoguang
"""

# import
import math
import numpy as np
from attitude import attitude

# globals
VERSION = '1.0'

class InclinometerKF_la(object):
    '''
    EKF-based dynamic inclinometer with linear accelerationi estimation.
    '''
    def __init__(self, fs, sigma_nb, sigma_nw, sigma_na, sigma_nl, tao):
        '''
        Algorithm definitions
        Args:
            fs: sample frequency, Hz.
            sigma_nb: gyro bias instability. b_dot = nb, b stands for gyro bias.
            sigma_nw: gyro noise std. w_measurement = w_true + b + nw.
            sigma_na: acc measurement noise std.
            sigma_nl: linear acceleration system noise, la_dot = -(1/tao)*la + nl.
            tao: linear acceleration correlation time. sec.
        '''
        self.ini = 0
        self.fs = fs
        self.dt = 1.0/fs
        self.nb = sigma_nb
        self.nl = sigma_nl
        self.nw = sigma_nw
        self.na = sigma_na
        self.tao = tao
        self.X = np.zeros((9,))
        self.P = np.zeros((9, 9))
        self.K = np.zeros((9, 3))
        self.Q = np.diag(np.hstack([self.nb*self.nb,
                                    self.nl*self.nl,
                                    self.nw*self.nw]), 0) * self.dt * self.dt
        self.R = np.diag(self.na*self.na, 0)
        self.phi = np.zeros((9, 9))
        self.phi[0, 0] = 1.0
        self.phi[1, 1] = 1.0
        self.phi[2, 2] = 1.0
        self.phi[3, 3] = 1.0 - self.dt/self.tao
        self.phi[4, 4] = 1.0 - self.dt/self.tao
        self.phi[5, 5] = 1.0 - self.dt/self.tao
        self.H = np.hstack([np.zeros((3, 3)), np.eye(3), -np.eye(3)])
        self.last_w = np.zeros((3,))
        self.angles = np.zeros((3,))
        return

    def update(self, w, a):
        '''
        Input sensor measurement and fusion
        Args:
            w: gyro measurement, rad/s.
            a: accelerometer measurement, m/s2.
        '''
        a = a / 9.8
        # initialization
        if self.ini == 0:
            self.ini = 1
            self.X = 0.0 * self.X
            acc_norm = math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
            if acc_norm > 0:
                self.X[6] = -a[0] / acc_norm
                self.X[7] = -a[1] / acc_norm
                self.X[8] = -a[2] / acc_norm
            else:
                self.X[6] = 0.0
                self.X[7] = 0.0
                self.X[8] = 1.0
            self.P = np.diag([1.0e-1, 1.0e-1, 1.0e-1,
                              1.0e-2, 1.0e-2, 1.0e-2,
                              1.0e-2, 1.0e-2, 1.0e-2])
            self.last_w[0] = w[0]
            self.last_w[1] = w[1]
            self.last_w[2] = w[2]
            self.angles[0] = 0.0
            self.angles[1] = math.asin(-self.X[6])
            self.angles[2] = math.atan2(self.X[7], self.X[8])
            return
        # prediction
        b_mi = self.X[0:3]
        l_mi = (1.0-self.dt/self.tao) * np.eye(3).dot(self.X[3:6])
        w_corr = self.last_w - self.X[0:3]
        g_mi = self.X[6:9] + self.dt*np.cross(-w_corr, self.X[6:9])
        z = l_mi - g_mi
        # innovation
        inno = a - z
        inno_norm = math.sqrt(inno[0]*inno[0] + inno[1]*inno[1] + inno[2]*inno[2])
        if inno_norm > 0.1:
            self.Q[3, 3] = 100.0
            self.Q[4, 4] = 100.0
            self.Q[5, 5] = 100.0
        else:
            self.Q[3, 3] = 1.0 - self.dt/self.tao
            self.Q[4, 4] = 1.0 - self.dt/self.tao
            self.Q[5, 5] = 1.0 - self.dt/self.tao
        self.phi[6:9, 0:3] = -self.dt * attitude.get_cross_mtx(self.X[6:9])
        self.phi[6:9, 6:9] = np.eye(3) + self.dt * attitude.get_cross_mtx(-w_corr)
        P_mi = self.phi.dot(self.P).dot(self.phi.T) + self.Q
        # update
        self.K = (P_mi.dot(self.H.T)).dot(np.linalg.inv((self.H.dot(P_mi)).dot(self.H.T)+self.R))
        self.X = np.hstack([b_mi, l_mi, g_mi]) + self.K.dot(inno)
        g_norm = math.sqrt(self.X[6]*self.X[6] + self.X[7]*self.X[7] + self.X[8]*self.X[8])
        self.X[6] = self.X[6] / g_norm
        self.X[7] = self.X[7] / g_norm
        self.X[8] = self.X[8] / g_norm
        self.P = (np.eye(9) - self.K.dot(self.H)).dot(P_mi)
        # save w
        self.last_w[0] = w[0]
        self.last_w[1] = w[1]
        self.last_w[2] = w[2]
        # Euler angles, ZYX<==>YPR, rad
        self.angles[0] = 0.0
        self.angles[1] = math.asin(-self.X[6])
        self.angles[2] = math.atan2(self.X[7], self.X[8])
        return

class InclinometerKF(object):
    '''
    EKF-based dynamic inclinometer without linear accelerationi estimation.
    '''
    def __init__(self, fs, sigma_nb, sigma_nw, sigma_na):
        '''
        Initialize sample period
        Args:
            fs: sample frequency, Hz.
            sigma_nb: gyro bias instability. b_dot = nb, b stands for gyro bias.
            sigma_nw: gyro noise std. w_measurement = w_true + b + nw.
            sigma_na: acc measurement noise std.
        '''
        self.ini = 0
        self.fs = fs
        self.dt = 1.0/fs
        self.nb = sigma_nb
        self.nw = sigma_nw
        self.na = sigma_na
        self.X = np.zeros((6,))
        self.P = np.zeros((6, 6))
        self.K = np.zeros((6, 3))
        self.Q = np.diag(np.hstack([self.nb*self.nb,
                                    self.nw*self.nw]), 0) * self.dt * self.dt
        self.R = np.diag(self.na*self.na, 0)
        self.phi = np.zeros((6, 6))
        self.phi[0, 0] = 1.0
        self.phi[1, 1] = 1.0
        self.phi[2, 2] = 1.0
        self.H = np.hstack([np.zeros((3, 3)), -np.eye(3)])
        self.last_w = np.zeros((3,))
        self.angles = np.zeros((3,))
        return

    def update(self, w, a):
        '''
        Input sensor measurement and fusion
        Args:
            w: gyro measurement, rad/s.
            a: accelerometer measurement, m/s2.
        '''
        a = a / 9.8
        # initialization
        if self.ini == 0:
            self.ini = 1
            self.X = 0.0 * self.X
            acc_norm = math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
            if acc_norm > 0.0:
                self.X[3] = -a[0] / acc_norm
                self.X[4] = -a[1] / acc_norm
                self.X[5] = -a[2] / acc_norm
            else:
                self.X[3] = 0.0
                self.X[4] = 0.0
                self.X[5] = 1.0
            self.P = np.diag([1.0e-1, 1.0e-1, 1.0e-1,
                              1.0e-2, 1.0e-2, 1.0e-2])
            self.last_w[0] = w[0]
            self.last_w[1] = w[1]
            self.last_w[2] = w[2]
            self.angles[0] = 0.0
            self.angles[1] = math.asin(-self.X[3])
            self.angles[2] = math.atan2(self.X[4], self.X[5])
            return
        # prediction
        b_mi = self.X[0:3]
        w_corr = self.last_w - self.X[0:3]
        g_mi = self.X[3:6] + self.dt*np.cross(-w_corr, self.X[3:6])
        z = -g_mi
        # innovation
        inno = a - z
        inno_norm = math.sqrt(inno[0]*inno[0] + inno[1]*inno[1] + inno[2]*inno[2])
        if inno_norm > 0.1:
            self.R = np.eye(3)*1.0          # dynamic, increase measurement noise
            inno = inno / inno_norm * 0.1   # limit innovation
        else:
            self.R = np.diag(self.na*self.na*10000)
        self.phi[3:6, 0:3] = -self.dt * attitude.get_cross_mtx(self.X[3:6])
        self.phi[3:6, 3:6] = np.eye(3) + self.dt * attitude.get_cross_mtx(-w_corr)
        P_mi = self.phi.dot(self.P).dot(self.phi.T) + self.Q
        # update
        self.K = (P_mi.dot(self.H.T)).dot(np.linalg.inv((self.H.dot(P_mi)).dot(self.H.T)+self.R))
        self.X = np.hstack([b_mi, g_mi]) + self.K.dot(inno)
        g_norm = math.sqrt(self.X[3]*self.X[3] + self.X[4]*self.X[4] + self.X[5]*self.X[5])
        self.X[3] = self.X[3] / g_norm
        self.X[4] = self.X[4] / g_norm
        self.X[5] = self.X[5] / g_norm
        self.P = (np.eye(6) - self.K.dot(self.H)).dot(P_mi)
        # save w
        self.last_w[0] = w[0]
        self.last_w[1] = w[1]
        self.last_w[2] = w[2]
        # Euler angles, ZYX<==>YPR, rad
        self.angles[0] = 0.0
        self.angles[1] = math.asin(-self.X[3])
        self.angles[2] = math.atan2(self.X[4], self.X[5])
        return
