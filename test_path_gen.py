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
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from mpl_toolkits.mplot3d import Axes3D
from pathgen import pathgen
from attitude import attitude
from buffer import buffer

# globals
VERSION = '1.0'
D2R = math.pi/180

data_path = os.path.abspath('.//data//')
fs = 100.0           # IMU sample frequency
fs_gps = 10.0        # GPS sample frequency
ref_frame = 0       # 0: NED frame, 1: virtual inertial frame

def test_path_gen():
    """
    test path_gen, save data to azure.
    """
    # define trajectory params
    ini_pos_n = np.array([32*D2R, 120*D2R, 0]) # Lat, Lon, Alt
    in_ivel_b = np.array([0, 0, 0])              # velocity in body frame
    ini_att = np.array([0, 0, 0])                # yaw, pitch, roll
    motion_def = np.array(                       # motion commands
        [[1, 0, 0, 0, 0, 10],
         [5, 0, 45*D2R, 0, 10, 250],
         [1, 0, 0, 0, 0, 10],
         [3, 90*D2R, -45*D2R, 0, 0, 25],
         [1, 0, 0, 0, 0, 50],
         [3, 180*D2R, 0, 0, 0, 25],
         [1, 0, 0, 0, 0, 50],
         [3, -180*D2R, 0, 0, 0, 25],
         [1, 0, 0, 0, 0, 50],
         [3, 180*D2R, 0, 0, 0, 25],
         [1, 0, 0, 0, 0, 50],
         [3, -180*D2R, 0, 0, 0, 25],
         [1, 0, 0, 0, 0, 50],
         [3, 180*D2R, 0, 0, 0, 25],
         [1, 0, 0, 0, 0, 50],
         [3, -180*D2R, 0, 0, 0, 25],
         [1, 0, 0, 0, 0, 50],
         [5, 0, 0, 0, 0, 10]])
    '''motion_def = np.array(                       # motion commands
        [[1, 0, 0, 0, 0, 10],
         [5, 0, 15*D2R, 0, 1, 25],
         [1, 0, 0, 0, 0, 50],
         [3, 0, -15*D2R, 0, 1, 25],
         [1, 0, 0, 0, 0, 50]])'''
    '''motion_def = np.array(                       # motion commands
        [[3, 90*D2R, 0, 0, 0, 10]])'''
    output_def = np.array([[1.0, fs], [1.0, fs_gps]])       # output definitions
    mobility = np.array([1.0, 0.5, 2.0])                    # maneuver capability
    # generate trajectory
    rtn = pathgen.path_gen(np.hstack((ini_pos_n, in_ivel_b, ini_att)),
                           motion_def, output_def, mobility, ref_frame, 1)
    np.savetxt(data_path+'//imu.txt', rtn['imu'])
    np.savetxt(data_path+'//nav.txt', rtn['nav'])
    if len(rtn['mag']) > 0:
        np.savetxt(data_path+'//mag.txt', rtn['mag'])
    if len(rtn['gps']) > 0:
        np.savetxt(data_path+'//gps.txt', rtn['gps'])
    if len(rtn['odo']) > 0:
        np.savetxt(data_path+'//odo.txt', rtn['odo'])
    print('test_path_gen OK.')

def test_sensor_gen():
    '''
    test acc_gen, gyro_gen, gps_gen, mag_gen
    '''
    # plot position trajectory
    nav_data = np.genfromtxt(data_path + '//nav.txt', delimiter=' ')
    fig_pos = plt.figure('pos_nav')
    ax = fig_pos.add_subplot(111, projection='3d', aspect='equal')
    ax.plot(nav_data[:, 1], nav_data[:, 2], nav_data[:, 3])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_aspect('equal')
    #plt.show()
    # plot velocity
    fig_vel = plt.figure('vel_nav')
    ax = fig_vel.add_subplot(111)
    line_obj = ax.plot(nav_data[:, 0], nav_data[:, 4:7])
    plt.legend(iter(line_obj), ('x', 'y', 'z'))
    #plt.show()
    # add error to acc data
    imu_data = np.genfromtxt(data_path + '//imu.txt', delimiter=' ')
    acc_true = imu_data[:, 1:4]
    acc_err = {'b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
               'b_drift': np.array([0.02e-2, 0.02e-2, 0.02e-2]),
               'b_corr': np.array([60.0, 60.0, 60.0]),
               'vrw': np.array([0.05/60, 0.05/60, 0.05/60])}
    psd_amp = np.genfromtxt(data_path + '//sxx.csv', delimiter=',')
    psd_freq = np.genfromtxt(data_path + '//freq.csv', delimiter=',')
    vib_def = {'freq': psd_freq,
               'x': psd_amp*0.0,
               'y': psd_amp*0.0,
               'z': psd_amp*0.0}
    acc_mea = pathgen.acc_gen(acc_true, acc_err, vib_def, fs)
    fig_acc = plt.figure('acc')
    line_obj = plt.plot(imu_data[:, 0], acc_mea[:, 0:3])
    plt.legend(iter(line_obj), ('x', 'y', 'z'))
    #plt.show()
    # add error to gyro data
    gyro_true = imu_data[:, 4:7]
    gyro_err = {'b': np.array([0.0*D2R, 0.0*D2R, 0.0*D2R]),
                'b_drift': np.array([10.0*D2R, 10.0*D2R, 10.0*D2R])/3600.0,
                'b_corr':np.array([float("inf"), float("inf"), float("inf")]),
                'arw': np.array([0.75*D2R/60, 0.75*D2R/60, 0.75*D2R/60])}
    gyro_mea = pathgen.gyro_gen(gyro_true, gyro_err, fs)
    fig_gyro = plt.figure('gyro')
    line_obj = plt.plot(imu_data[:, 0], gyro_mea[:, 0:3])
    plt.legend(iter(line_obj), ('x', 'y', 'z'))
    # add error to GPS data
    gps_data = np.genfromtxt(data_path + '//gps.txt', delimiter=' ')
    gps_err = {'stdm': np.array([1.0, 1.0, 1.0]),
               'stdv': np.array([0.05, 0.05, 0.05])}
    gps_mea = pathgen.gps_gen(gps_data, gps_err, fs_gps, ref_frame)
    #np.savetxt('gps.txt', gps_mea)
    fig_gps = plt.figure('pos_gps')
    ax = fig_gps.add_subplot(111, projection='3d')
    ax.plot(gps_mea[:, 1], gps_mea[:, 2], gps_mea[:, 3])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    # add error to magnetic data
    mag_data = np.genfromtxt(data_path + '//mag.txt', delimiter=' ')
    mag_err = {'si': np.eye(3) + np.random.randn(3, 3)*0.0,
               'hi': np.array([10.0, 10.0, 10.0])*0.0,
               'std': np.array([0.5, 0.5, 0.5])}
    mag_mea = pathgen.mag_gen(mag_data[:, 1:4], mag_err)
    fig_mag = plt.figure('mag')
    line_obj = plt.plot(mag_data[:, 0], mag_mea[:, 0:3])
    plt.legend(iter(line_obj), ('x', 'y', 'z'))
    plt.show()
    ### save sensor data
    #np.savetxt('acc.txt', acc_mea)
    #np.savetxt('gyro.txt', gyro_mea)
    #np.savetxt('mag.txt', mag_mea)
    print('test_sensor gen OK.')

def test_allan():
    '''
    test allan var.
    '''
    from allan import allan
    gyro_true = np.zeros((10000, 3))
    gyro_err = {'b': np.array([3*D2R, 3*D2R, 3*D2R]),
                'b_drift': np.array([0.007*D2R, 0.007*D2R, 0.007*D2R]),
                'b_corr':np.array([100.0, 100.0, 100.0]),
                'arw': np.array([2*D2R/60, 2*D2R/60, 2*D2R/60])}
    gyro_mea = pathgen.gyro_gen(gyro_true, gyro_err, fs)
    np.savetxt(data_path+'//test.txt', gyro_mea)
    [avar, tau] = allan.allan_var(gyro_mea[:, 0], fs)
    plt.loglog(tau, np.sqrt(avar))
    plt.show()
    print('test_allan OK.')

def test_allan_for_comparison():
    '''
    test allan var compare sim with true.
    '''
    from allan import allan
    ### true imu data
    # load true data
    imu_data = np.genfromtxt(data_path + '//allantest.csv', delimiter=',')
    # 3-axis acc allan var
    fig_acc = plt.figure('allan_acc')
    axis_acc = fig_acc.add_subplot(111)
    [avar_ax, tau] = allan.allan_var(imu_data[:, 0], fs)
    ax, = axis_acc.loglog(tau, np.sqrt(avar_ax))
    [avar_ay, tau] = allan.allan_var(imu_data[:, 1], fs)
    ay, = axis_acc.loglog(tau, np.sqrt(avar_ay))
    [avar_az, tau] = allan.allan_var(imu_data[:, 2], fs)
    az, = axis_acc.loglog(tau, np.sqrt(avar_az))
    plt.legend([ax, ay, az], ['ax', 'ay', 'az'])
    plt.grid()
    # 3-axis gyro allan var
    fig_gyro = plt.figure('allan_gyro')
    axis_gyro = fig_gyro.add_subplot(111)
    [avar_wx, tau] = allan.allan_var(imu_data[:, 3], fs)
    wx, = axis_gyro.loglog(tau, np.sqrt(avar_wx))
    [avar_wy, tau] = allan.allan_var(imu_data[:, 4], fs)
    wy, = axis_gyro.loglog(tau, np.sqrt(avar_wy))
    [avar_wz, tau] = allan.allan_var(imu_data[:, 5], fs)
    wz, = axis_gyro.loglog(tau, np.sqrt(avar_wz))
    plt.legend([wx, wy, wz], ['wx', 'wy', 'wz'])
    plt.grid()
    ### sim imu data
    # generate imu data according to allan var
    imu_data = np.zeros(imu_data.shape)
    # add error to acc data
    acc_true = imu_data[:, 0:3]
    acc_err = {'b': np.array([0.0e-3, 0.0e-3, -0.0e-3]),
               'b_drift': np.array([4.29e-5, 5.72e-5, 8.02e-5]),
               'b_corr': np.array([200.0, 200.0, 200.0]),
               'vrw': np.array([0.03119/60.0, 0.03009/60.0, 0.04779/60.0])}
    acc_mea = pathgen.acc_gen(acc_true, acc_err, [], fs)
    # add error to gyro data
    gyro_true = imu_data[:, 3:6]
    gyro_err = {'b': np.array([0.0*D2R, 0.0*D2R, 0.0*D2R]),
                'b_drift': np.array([5.34*D2R/3600, 9.40*D2R/3600, 6.57*D2R/3600]),
                'b_corr': np.array([100.0, 100.0, 100.0]),
                'arw': np.array([0.52142*D2R/60, 0.64938*D2R/60, 0.73193*D2R/60])}
    gyro_mea = pathgen.gyro_gen(gyro_true, gyro_err, fs)
    # 3-axis sim acc allan var
    fig_acc_sim = plt.figure('allan_acc_sim')
    axis_acc_sim = fig_acc_sim.add_subplot(111)
    [avar_ax_sim, tau] = allan.allan_var(acc_mea[:, 0]/9.8, fs)
    ax_sim, = axis_acc_sim.loglog(tau, np.sqrt(avar_ax_sim))
    [avar_ay_sim, tau] = allan.allan_var(acc_mea[:, 1]/9.8, fs)
    ay_sim, = axis_acc_sim.loglog(tau, np.sqrt(avar_ay_sim))
    [avar_az_sim, tau] = allan.allan_var(acc_mea[:, 2]/9.8, fs)
    az_sim, = axis_acc_sim.loglog(tau, np.sqrt(avar_az_sim))
    plt.legend([ax_sim, ay_sim, az_sim], ['ax', 'ay', 'az'])
    plt.grid()
    # 3-axis sim gyro allan var
    fig_gyro_sim = plt.figure('allan_gyro_sim')
    axis_gyro_sim = fig_gyro_sim.add_subplot(111)
    [avar_wx_sim, tau] = allan.allan_var(gyro_mea[:, 0]/D2R, fs)
    wx_sim, = axis_gyro_sim.loglog(tau, np.sqrt(avar_wx_sim))
    [avar_wy_sim, tau] = allan.allan_var(gyro_mea[:, 1]/D2R, fs)
    wy_sim, = axis_gyro_sim.loglog(tau, np.sqrt(avar_wy_sim))
    [avar_wz_sim, tau] = allan.allan_var(gyro_mea[:, 2]/D2R, fs)
    wz_sim, = axis_gyro_sim.loglog(tau, np.sqrt(avar_wz_sim))
    plt.legend([wx_sim, wy_sim, wz_sim], ['wx', 'wy', 'wz'])
    plt.grid()
    # acc x comparison
    fig_ax = plt.figure('allan_ax')
    axis_ax = fig_ax.add_subplot(111)
    ax_comp, = axis_ax.loglog(tau, np.sqrt(avar_ax))
    ax_sim_comp, = axis_ax.loglog(tau, np.sqrt(avar_ax_sim))
    plt.legend([ax_comp, ax_sim_comp], ['ax-true', 'ax-sim'])
    plt.grid()
    # acc y comparison
    fig_ay = plt.figure('allan_ay')
    axis_ay = fig_ay.add_subplot(111)
    ay_comp, = axis_ay.loglog(tau, np.sqrt(avar_ay))
    ay_sim_comp, = axis_ay.loglog(tau, np.sqrt(avar_ay_sim))
    plt.legend([ay_comp, ay_sim_comp], ['ay-true', 'ay-sim'])
    plt.grid()
    # acc z comparison
    fig_az = plt.figure('allan_az')
    axis_az = fig_az.add_subplot(111)
    az_comp, = axis_az.loglog(tau, np.sqrt(avar_az))
    az_sim_comp, = axis_az.loglog(tau, np.sqrt(avar_az_sim))
    plt.legend([az_comp, az_sim_comp], ['az-true', 'az-sim'])
    plt.grid()
    # gyro x comparison
    fig_wx = plt.figure('allan_wx')
    axis_wx = fig_wx.add_subplot(111)
    wx_comp, = axis_wx.loglog(tau, np.sqrt(avar_wx))
    wx_sim_comp, = axis_wx.loglog(tau, np.sqrt(avar_wx_sim))
    plt.legend([wx_comp, wx_sim_comp], ['wx-true', 'wx-sim'])
    plt.grid()
    # gyro y comparison
    fig_wy = plt.figure('allan_wy')
    axis_wy = fig_wy.add_subplot(111)
    wy_comp, = axis_wy.loglog(tau, np.sqrt(avar_wy))
    wy_sim_comp, = axis_wy.loglog(tau, np.sqrt(avar_wy_sim))
    plt.legend([wy_comp, wy_sim_comp], ['wy-true', 'wy-sim'])
    plt.grid()
    # gyro z comparsion
    fig_wz = plt.figure('allan_wz')
    axis_wz = fig_wz.add_subplot(111)
    wz_comp, = axis_wz.loglog(tau, np.sqrt(avar_wz))
    wz_sim_comp, = axis_wz.loglog(tau, np.sqrt(avar_wz_sim))
    plt.legend([wz_comp, wz_sim_comp], ['wz-true', 'wz-sim'])
    plt.grid()
    # show figures
    plt.show()
    print('test_allan_for_comparison OK.')

def test_filter_design():
    '''
    test Butterworth lpf design
    '''
    from filter import butterworth
    [rtn, num, den] = butterworth.butter_lpf(5, 15, 2)
    print(rtn)
    print(num)
    print(den)

def test_filter():
    '''
    test filter
    '''
    from filter import butterworth
    from filter import filter
    filter_order = 2
    bf_raw = buffer.Buffer(3, filter_order+1)
    bf_filtered = buffer.Buffer(3, filter_order+1)
    # filter design
    fc = np.array([1, 1, 1])
    num = np.zeros((3, filter_order+1))
    den = np.zeros((3, filter_order+1))
    for i in range(0, 3):
        [rtn, num[i], den[i]] = butterworth.butter_lpf(fc[i], fs, filter_order)
    # read imu data
    imu_data = np.genfromtxt(data_path + '//imu.txt', delimiter=' ')
    n = imu_data.shape[0]
    # add error to acc data
    acc_true = imu_data[:, 1:4]
    acc_err = {'b': np.array([10.0e-3, 15.0e-3, -10.0e-3]),
               'b_drift': np.array([1.0e-3, 1.0e-3, 1.0e-3]),
               'b_corr':np.array([60.0, 60.0, 60.0]),
               'vrw': np.array([0.3/60, 0.3/60, 0.3/60])}
    acc_mea = pathgen.acc_gen(acc_true, acc_err, [], fs)
    # filter the imu data
    lpf = filter.Filter(num, den)
    acc_filtered = np.zeros((n, 3))
    for i in range(0, n):
        #print(i)
        bf_raw.put(acc_mea[i, :])
        lpf.flt(bf_raw, bf_filtered)
        (rtn, acc_filtered[i, :]) = bf_filtered.get(0)
    np.savetxt(data_path + '//acc_mea.txt', acc_mea)
    np.savetxt(data_path + '//acc_filtered.txt', acc_filtered)
    print('filter OK')

def test_buffer():
    '''
    test buffer
    '''
    # test put
    bf = buffer.Buffer(3, 10)
    for i in range(0, 5):
        bf.put(np.array([i, i, i]))
        print(bf.i)
        print(bf.num)
        print(bf.full)
        print(bf.d)
    # test get
    for i in range(0, 15):
        print(i)
        print(bf.get(i))

def test_psd():
    '''
    test psd
    '''
    # read true imu data
    imu_data = np.genfromtxt(data_path + '//imu.txt', delimiter=' ')
    acc_true = imu_data[:, 1:4]
    # imu error profile
    acc_err = {'b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
               'b_drift': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
               'b_corr': np.array([60.0, 60.0, 60.0]),
               'vrw': np.array([0.0/60, 0.0/60, 0.0/60])}
    # vibration profile
    psd_amp = np.genfromtxt(data_path + '//sxx.csv', delimiter=',')
    psd_freq = np.genfromtxt(data_path + '//freq.csv', delimiter=',')
    vib_def = {'freq': psd_freq,
               'x': psd_amp*0.1,
               'y': psd_amp*0.1,
               'z': psd_amp}
    # generate acc measurements
    acc_mea = pathgen.acc_gen(acc_true, acc_err, vib_def, fs)
    # save data for analysis
    np.savetxt('acc.txt', acc_mea)
    x = acc_mea[0:16384, 2]
    x = x - np.mean(x)
    sim_pxx, sim_freq = mlab.psd(x, NFFT=len(x), Fs=fs,
                                 detrend=mlab.detrend_none,
                                 window=mlab.window_hanning)
    plt.loglog(sim_freq, sim_pxx, psd_freq, psd_amp)
    plt.legend(('sim', 'ref'))
    plt.grid(True)
    plt.show()
    print('test psd ok.')

def test_sensor_gen_imu():
    '''
    test acc_gen, gyro_gen, gps_gen, mag_gen
    '''
    # add error to acc data
    imu_data = np.genfromtxt(data_path + '//imu.txt', delimiter=' ')
    acc_true = imu_data[:, 1:4]
    acc_err = {'b': np.array([6.0e-3, 1.0e-3, 0.0e-3])*0.0,
               'b_drift': np.array([0.02e-2, 0.02e-2, 0.02e-2])*0.0,
               'b_corr': np.array([60.0, 60.0, 60.0]),
               'vrw': np.array([0.05/60, 0.05/60, 0.05/60])*1.0}
    acc_mea = pathgen.acc_gen(acc_true, acc_err, [], fs)
    # add error to gyro data
    gyro_true = imu_data[:, 4:7]
    gyro_err = {'b': np.array([0.0*D2R, 0.0*D2R, 0.0*D2R]),
                'b_drift': np.array([10.0*D2R, 10.0*D2R, 10.0*D2R])/3600.0*0.0,
                'b_corr':np.array([float("inf"), float("inf"), float("inf")]),
                'arw': np.array([0.75*D2R/60, 0.75*D2R/60, 0.75*D2R/60])*1.0}
    gyro_mea = pathgen.gyro_gen(gyro_true, gyro_err, fs)
    # add error to odometer data
    odo_data = np.genfromtxt(data_path + '//odo.txt', delimiter=' ')
    odo_true = odo_data[:, 2::]
    odo_err = {'scale': np.array([1.1, 1.1, 1.1]),
               'std': np.array([0.1, 0.2, 0.3])}
    odo_mea = pathgen.odo_gen(odo_true, odo_err)
    # save to file
    np.savetxt(data_path+'//acc_m.txt', acc_mea)
    np.savetxt(data_path+'//gyro_m.txt', gyro_mea)
    np.savetxt(data_path+'//odo_m.txt', odo_mea)
    print('test_sensor_gen_imu OK.')

def test_inclinometer_ecf():
    '''
    test dynamic inclinometer.
    '''
    from filter import butterworth
    from filter import filter
    from fusion import mahony
    # read imu data
    imu_data = np.genfromtxt(data_path + '//imu.txt', delimiter=' ')
    n = imu_data.shape[0]
    # add error to acc data
    acc_true = imu_data[:, 1:4]
    acc_err = {'b': np.array([0.0e-3, 0.0e-3, -0.0e-3]),
               'b_drift': np.array([1.0e-3, 1.0e-3, 1.0e-3]),
               'b_corr': np.array([60.0, 60.0, 60.0]),
               'vrw': np.array([0.03/60, 0.03/60, 0.03/60])}
    acc_mea = pathgen.acc_gen(acc_true, acc_err, [], fs)
    # add error to gyro data
    gyro_true = imu_data[:, 4:7]
    gyro_err = {'b': np.array([0.0*D2R, 0.0*D2R, 0.0*D2R]),
                'b_drift': np.array([0.005*D2R, 0.005*D2R, 0.005*D2R]),
                'b_corr': np.array([10.0, 10.0, 10.0]),
                'arw': np.array([2*D2R/60, 2*D2R/60, 2*D2R/60])}
    gyro_mea = pathgen.gyro_gen(gyro_true, gyro_err, fs)
    #### dynamic inclinomter
    # data buffer for filter
    filter_order = 2
    bf_acc_raw = buffer.Buffer(3, filter_order+1)
    bf_acc_filtered = buffer.Buffer(3, filter_order+1)
    bf_gyro_raw = buffer.Buffer(3, filter_order+1)
    bf_gyro_filtered = buffer.Buffer(3, filter_order+1)
    # lowpass filter design
    fc_acc = np.array([1, 1, 1])
    fc_gyro = np.array([5, 5, 5])
    num_acc = np.zeros((3, filter_order+1))
    den_acc = np.zeros((3, filter_order+1))
    num_gyro = np.zeros((3, filter_order+1))
    den_gyro = np.zeros((3, filter_order+1))
    for i in range(0, 3):
        [rtn, num_acc[i], den_acc[i]] = butterworth.butter_lpf(fc_acc[i], fs, filter_order)
        [rtn, num_gyro[i], den_gyro[i]] = butterworth.butter_lpf(fc_gyro[i], fs, filter_order)
    lpf_acc = filter.Filter(num_acc, den_acc)
    lpf_gyro = filter.Filter(num_gyro, den_gyro)
    # dynamic inclinometer
    yaw = np.zeros((n,))
    pitch = np.zeros((n,))
    roll = np.zeros((n,))
    fusion = mahony.MahonyFilter(1.0/fs)
    for i in range(0, n):
        if i == 25:
            print(i)
        bf_acc_raw.put(acc_mea[i, :])
        bf_gyro_raw.put(gyro_mea[i, :])
        lpf_acc.flt(bf_acc_raw, bf_acc_filtered)
        lpf_gyro.flt(bf_gyro_raw, bf_gyro_filtered)
        [rtn, acc_filtered] = bf_acc_filtered.get(0)
        [rtn, gyro_filtered] = bf_gyro_filtered.get(0)
        fusion.update(gyro_filtered, acc_filtered, np.array([0.0, 0.0, 0.0]))
        #fusion.update(gyro_mea[i, :], acc_mea[i, :], np.array([0.0, 0.0, 0.0]))
        cn2b = attitude.quat2dcm(fusion.q)
        angles = attitude.dcm2euler(cn2b, 'zyx')
        yaw[i] = angles[0]
        pitch[i] = angles[1]
        roll[i] = angles[2]
    # save results
    #'''
    np.savetxt(data_path + '//acc_m.txt', acc_mea)
    np.savetxt(data_path + '//gyro_m.txt', gyro_mea)
    np.savetxt(data_path + '//yaw.txt', yaw)
    np.savetxt(data_path + '//pitch.txt', pitch)
    np.savetxt(data_path + '//roll.txt', roll)
    #'''
    nav_data = np.genfromtxt(data_path + '//nav.txt', delimiter=' ')
    att = nav_data[:, 7:10]
    plt.figure('pitch')
    plt.plot(pitch/D2R - att[:, 1]/D2R)
    plt.figure('roll')
    plt.plot(roll/D2R - att[:, 2]/D2R)
    plt.show()
    print('Mahony dynamic inclinometer OK.')

def test_inclinometer_kf():
    '''
    test inclinometer_kf
    '''
    from filter import butterworth
    from filter import filter
    from fusion import inclinometer_kf
    # read imu data
    imu_data = np.genfromtxt(data_path + '//imu.txt', delimiter=' ')
    n = imu_data.shape[0]
    # add error to acc data
    acc_true = imu_data[:, 1:4]
    acc_err = {'b': np.array([0.0e-3, 0.0e-3, -0.0e-3]),
               'b_drift': np.array([1.0e-3, 1.0e-3, 1.0e-3]),
               'b_corr': np.array([60.0, 60.0, 60.0]),
               'vrw': np.array([0.03/60, 0.03/60, 0.03/60])}
    acc_mea = pathgen.acc_gen(acc_true, acc_err, [], fs)
    # add error to gyro data
    gyro_true = imu_data[:, 4:7]
    gyro_err = {'b': np.array([1.0*D2R, -1.0*D2R, 2.0*D2R]),
                'b_drift': np.array([0.005*D2R, 0.005*D2R, 0.005*D2R]),
                'b_corr': np.array([100.0, 100.0, 100.0]),
                'arw': np.array([2*D2R/60, 2*D2R/60, 2*D2R/60])}
    gyro_mea = pathgen.gyro_gen(gyro_true, gyro_err, fs)
    np.savetxt(data_path + '//acc_m.txt', acc_mea)
    np.savetxt(data_path + '//gyro_m.txt', gyro_mea)
    #### dynamic inclinomter
    # data buffer for filter
    filter_order = 2
    bf_acc_raw = buffer.Buffer(3, filter_order+1)
    bf_acc_filtered = buffer.Buffer(3, filter_order+1)
    bf_gyro_raw = buffer.Buffer(3, filter_order+1)
    bf_gyro_filtered = buffer.Buffer(3, filter_order+1)
    # lowpass filter design
    fc_acc = np.array([1, 1, 1])
    fc_gyro = np.array([5, 5, 5])
    num_acc = np.zeros((3, filter_order+1))
    den_acc = np.zeros((3, filter_order+1))
    num_gyro = np.zeros((3, filter_order+1))
    den_gyro = np.zeros((3, filter_order+1))
    for i in range(0, 3):
        [rtn, num_acc[i], den_acc[i]] = butterworth.butter_lpf(fc_acc[i], fs, filter_order)
        [rtn, num_gyro[i], den_gyro[i]] = butterworth.butter_lpf(fc_gyro[i], fs, filter_order)
    lpf_acc = filter.Filter(num_acc, den_acc)
    lpf_gyro = filter.Filter(num_gyro, den_gyro)
    # dynamic inclinometer
    yaw = np.zeros((n,))
    pitch = np.zeros((n,))
    roll = np.zeros((n,))
    sigma_nb = np.array([1.0e-3, 1.0e-3, 1.0e-3])
    sigma_nw = np.array([2.0, 2.0, 2.0]) * D2R/60.0/math.sqrt(1.0/fs)
    sigma_na = np.array([0.03, 0.03, 0.03]) /60.0/math.sqrt(1.0/fs)
    # sigma_nl = np.array([1.0e-1, 1.0e-1, 1.0e-1])
    # tao = 1.0
    #fusion = inclinometer_kf.InclinometerKF_la(fs, sigma_nb, sigma_nw, sigma_na, sigma_nl, tao)
    fusion = inclinometer_kf.InclinometerKF(fs, sigma_nb, sigma_nw, sigma_na)
    for i in range(0, n):
        if i == 25:
            print(i)
        bf_acc_raw.put(acc_mea[i, :])
        bf_gyro_raw.put(gyro_mea[i, :])
        lpf_acc.flt(bf_acc_raw, bf_acc_filtered)
        lpf_gyro.flt(bf_gyro_raw, bf_gyro_filtered)
        [rtn, acc_filtered] = bf_acc_filtered.get(0)
        [rtn, gyro_filtered] = bf_gyro_filtered.get(0)
        fusion.update(gyro_mea[i, :], acc_mea[i, :])
        #fusion.update(gyro_mea[i, :], acc_mea[i, :], np.array([0.0, 0.0, 0.0]))
        angles = fusion.angles
        yaw[i] = angles[0]
        pitch[i] = angles[1]
        roll[i] = angles[2]
    # save results
    np.savetxt(data_path + '//yaw.txt', yaw)
    np.savetxt(data_path + '//pitch.txt', pitch)
    np.savetxt(data_path + '//roll.txt', roll)
    #
    nav_data = np.genfromtxt(data_path + '//nav.txt', delimiter=' ')
    att = nav_data[:, 7:10]
    plt.figure('pitch')
    plt.plot(pitch/D2R - att[:, 1]/D2R)
    plt.figure('roll')
    plt.plot(roll/D2R - att[:, 2]/D2R)
    plt.show()
    print('EKF dynamic inclinometer OK.')

def test_kml_gen():
    '''
    test kml_gen
    '''
    from kml_gen import kml_gen
    nav_data = np.genfromtxt(data_path + '//nav.txt', delimiter=' ')
    lla = nav_data[0:40000:, 1:4]       # there is a data limit in Google Earth
    lla[:, 2] = lla[:, 2]
    kml_contents = kml_gen.kml_gen_from_lla(lla, data_path + '//template.kml')
    kml_file = data_path + '//route_ge.kml'
    fp = open(kml_file, 'w')
    fp.write(kml_contents)
    fp.close()
    print('test kml_gen ok.')

def test_google_earth():
    '''
    test google earth
    '''
    from google_earth import google_earth as ge
    # gen kml file
    test_kml_gen()
    # open Google Earth
    ge.ge_connect()
    # read kml file into Google Earth
    ge.ge_openKmlFile(data_path + '//route_ge.kml')

def test_google_earth_oo():
    '''
    test google earth
    '''
    from google_earth import google_earth_oo as ge
    # gen kml file
    test_kml_gen()
    # open Google Earth
    hge = ge.GoogleEarth()
    # read kml file into Google Earth
    hge.load_kml(data_path + '//route_ge.kml')

if __name__ == '__main__':
    # test_path_gen()
    # test_sensor_gen()
    # test_sensor_gen_imu()
    # test_allan()
    test_allan_for_comparison()
    # test_filter_design()
    # test_buffer()
    # test_filter()
    # test_inclinometer_ecf()
    # test_inclinometer_kf()
    # test_psd()

    # import cProfile
    # import pstats
    # cProfile.run('test_path_gen()', 'stats.cprofile')
    # p = pstats.Stats('stats.cprofile')
    # p.sort_stats('time').print_stats(10)

    # test_kml_gen()
    # test_google_earth_oo()   # this only works in windows
