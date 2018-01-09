# -*- coding: utf-8 -*-
# Fielname = path_gen.py

"""
Trajectory generation for IMU+GNSS simulation
Created on 2017-09-12
20171027:   Add maneuver capability to limit acceleration, angular acceleration
            and angular velocity.
20171028:   Remove magnetic inclination when reference frame is a virtual inertial
            frame (ref_frame==1).
@author: dongxiaoguang
"""

# import
import os
import math
import numpy as np
from attitude import attitude
from geoparams import geoparams
from geoparams import geomag
from psd import time_series_from_psd

# global
VERSION = '1.0'
D2R = math.pi/180

def acc_gen(fs, ref_a, acc_err, vib_def=None):
    """
    Add error to true acc data according to acclerometer model parameters
    Args:
        fs: sample frequency, Hz.
        ref_a: nx3 true acc data, m/s2.
        acc_err: accelerometer error parameters.
            'b': 3x1 acc constant bias, m/s2.
            'b_drift': 3x1 acc bias drift, m/s2.
            'vrw': 3x1 velocity random walk, m/s2/root-Hz.
        vib_def: Vibration model and parameters. Vibration type can be random, sinunoida or
            specified by single-sided PSD.
            Generated vibrating acc is expressed in the body frame.
            'type' == 'random':
                Normal distribution. 'amp' gives the 1sigma values, units: m/s2
            'type' == 'sinunoidal'
                Sinunoidal vibration. 'amp' gives the amplitude of the sine wave, units: m/s2.
            'type' == 'psd'. Single sided PSD.
                'freq':  frequency, in unit of Hz
                'x': x axis, in unit of m2/s4/Hz.
                'y': y axis, in unit of m2/s4/Hz.
                'z': z axis, in unit of m2/s4/Hz.
    Returns:
        a_mea: nx3 measured acc data
    """
    dt = 1.0/fs
    # total data count
    n = ref_a.shape[0]
    ## simulate sensor error
    # static bias
    acc_bias = acc_err['b']
    # bias drift
    acc_bias_drift = bias_drift(acc_err['b_corr'], acc_err['b_drift'], n, fs)
    # vibrating acceleration
    acc_vib = np.zeros((n, 3))
    if vib_def is not None:
        if vib_def['type'].lower() == 'psd':
            acc_vib[:, 0] = time_series_from_psd.time_series_from_psd(vib_def['x'],
                                                                      vib_def['freq'], fs, n)[1]
            acc_vib[:, 1] = time_series_from_psd.time_series_from_psd(vib_def['y'],
                                                                      vib_def['freq'], fs, n)[1]
            acc_vib[:, 2] = time_series_from_psd.time_series_from_psd(vib_def['z'],
                                                                      vib_def['freq'], fs, n)[1]
        elif vib_def['type'] == 'random':
            acc_vib[:, 2] = vib_def['amp'] * np.random.randn(n)
        elif vib_def['type'] == 'sinusoidal':
            acc_vib[:, 2] = vib_def['amp'] * np.sin(2.0*math.pi*vib_def['freq']*dt*np.arange(n))
    # accelerometer white noise
    acc_noise = np.random.randn(n, 3)
    acc_noise[:, 0] = acc_err['vrw'][0] / math.sqrt(dt) * acc_noise[:, 0]
    acc_noise[:, 1] = acc_err['vrw'][1] / math.sqrt(dt) * acc_noise[:, 1]
    acc_noise[:, 2] = acc_err['vrw'][2] / math.sqrt(dt) * acc_noise[:, 2]
    # true + constant_bias + bias_drift + noise
    a_mea = ref_a + acc_bias + acc_bias_drift + acc_noise + acc_vib
    return a_mea

def gyro_gen(fs, ref_w, gyro_err):
    """
    Add error to true gyro data according to gyroscope model parameters
    Args:
        fs: sample frequency, Hz.
        ref_w: nx3 true acc data, rad/s.
        gyro_err: gyroscope error parameters.
            'b': 3x1 constant gyro bias, rad/s.
            'b_drift': 3x1 gyro bias drift, rad/s.
            'arw': 3x1 angle random walk, rad/s/root-Hz.
    Returns:
        w_mea: nx3 measured gyro data
    """
    dt = 1.0/fs
    # total data count
    n = ref_w.shape[0]
    ## simulate sensor error
    # static bias
    gyro_bias = gyro_err['b']
    # bias drift Todo: first-order Gauss-Markov model
    gyro_bias_drift = bias_drift(gyro_err['b_corr'], gyro_err['b_drift'], n, fs)
    # gyroscope white noise
    gyro_noise = np.random.randn(n, 3)
    gyro_noise[:, 0] = gyro_err['arw'][0] / math.sqrt(dt) * gyro_noise[:, 0]
    gyro_noise[:, 1] = gyro_err['arw'][1] / math.sqrt(dt) * gyro_noise[:, 1]
    gyro_noise[:, 2] = gyro_err['arw'][2] / math.sqrt(dt) * gyro_noise[:, 2]
    # true + constant_bias + bias_drift + noise
    w_mea = ref_w + gyro_bias + gyro_bias_drift + gyro_noise
    return w_mea

def bias_drift(corr_time, drift, n, fs):
    """
    Bias drift (instability) model for accelerometers or gyroscope.
    If correlation time is valid (positive and finite), a first-order Gauss-Markov model is used.
    Otherwise, a simple normal distribution model is used.
    Args:
        corr_time: 3x1 correlation time, sec.
        drift: 3x1 bias drift std, rad/s.
        n: total data count
        fs: sample frequency, Hz.
    Returns
        sensor_bias_drift: drift of sensor bias
    """
    # 3 axis
    sensor_bias_drift = np.zeros((n, 3))
    for i in range(0, 3):
        if not math.isinf(corr_time[i]):
            # First-order Gauss-Markov
            a = 1 - 1/fs/corr_time[i]
            b = 1/fs*drift[i]
            #sensor_bias_drift[0, :] = np.random.randn(3) * drift
            drift_noise = np.random.randn(n-1, 3)
            for j in range(1, n):
                sensor_bias_drift[j, i] = a*sensor_bias_drift[j-1, i] + b*drift_noise[j-1, i]
        else:
            # normal distribution
            sensor_bias_drift[:, i] = drift[i] * np.random.randn(n)
    return sensor_bias_drift

def gps_gen(ref_gps, gps_err, gps_type=0):
    '''
    Add error to true GPS data according to GPS receiver error parameters
    Args:
        ref_gps: If gps_type is 0, [Lat, Lon, Alt, vx, vy, vz], [rad, rad, m].
                 If gps_type is 1, [x, y, z, vx, vy, vz], [m, m, m].
                 ref_gps data are expressed in the navigation frame.
        gps_err: GPS reeceiver parameters.
            'stdm': RMS position error, [m, m, m].
            'stdv': RMS velocity error, [m/s, m/s, m/s].
        gps_type: GPS data type.
            0: default, position is in the form of [Lat, Lon, Alt], rad, m
            1: position is in the form of [x, y, z], m
    Returns:
        gps_mea: ref_gps with error.
    '''
    # total data count
    n = ref_gps.shape[0]
    # If position is in the form of LLA, convert gps_err['stdm'] to LLA error
    if gps_type == 0:   # GPS is in the form of LLA, stdm meter to rad
        earth_param = geoparams.geo_param(ref_gps[0, 1:4])
        gps_err['stdm'][0] = gps_err['stdm'][0] / earth_param[0]
        gps_err['stdm'][1] = gps_err['stdm'][1] / earth_param[1] / earth_param[4]
    ## simulate GPS error
    pos_noise = gps_err['stdm'] * np.random.randn(n, 3)
    vel_noise = gps_err['stdv'] * np.random.randn(n, 3)
    gps_mea = np.hstack([ref_gps[:, 0:3] + pos_noise,
                         ref_gps[:, 3:6] + vel_noise])
    return gps_mea

def odo_gen(ref_odo, odo_err):
    '''
    Add error to true odometer data.
    Args:
        ref_odo: nx3, true odometer data, m/s.
        odo_err: odometer error profile.
            'scale': 3x1, scale factor error.
            'std': 3x1, RMS velocity error.
    Returns:
        odo_mea: nx3, measured odometer output.
    '''
    n = ref_odo.shape[0]
    odo_noise = np.random.randn(n, 3)
    scale_factor = odo_err['scale']
    odo_noise[:, 0] = scale_factor[0]*ref_odo[:, 0] + odo_err['std'][0]*odo_noise[:, 0]
    odo_noise[:, 1] = scale_factor[1]*ref_odo[:, 1] + odo_err['std'][1]*odo_noise[:, 1]
    odo_noise[:, 2] = scale_factor[2]*ref_odo[:, 2] + odo_err['std'][2]*odo_noise[:, 2]
    return odo_noise

def mag_gen(ref_mag, mag_err):
    """
    Add error to magnetic data.
    Args:
        ref_mag: nx3 true magnetic data, uT.
        mag_err: Magnetometer error parameters.
            'si': 3x3 soft iron matrix
            'hi': hard iron array, [ox, oy, oz], uT
            'std': RMS of magnetometer noise, uT
    Returns:
        mag_mea: ref_mag with error, mag_mea = si * (ref_mag + hi) + noise
    """
    # total data count
    n = ref_mag.shape[0]
    # add error
    mag_mea = ref_mag + mag_err['hi']
    mag_mea = mag_mea.dot(mag_err['si'].T)
    mag_noise = mag_err['std'] * np.random.randn(n, 3)
    return mag_mea + mag_noise

def path_gen(ini_pos_vel_att, motion_def, output_def, mobility, ref_frame=0, magnet=False):
    """
    Generate IMU and GPS or odometer data file according to initial position\velocity\attitude,
    motion command and simulation mode.
    The navigation frame is NED. The body frame is front-right-downward. True IMU sensor
    data, position\velocity\attitude data and GPS or odometry measurement data are
    generated and stored in files.
    Units for position in LLA [Lat Lon Alt] form are [rad rad m], units for position in [x y z]
    form is [m m m], units for angles are rad, time unit is sec, unless specified otherwise.
    Args:
        ini_pos_vel_att: 9x1 initial position, velocity and attitude.
            3x1 position in the form of [Lat, Lon, Alt].
            3x1 velocity in the body frame.
            3x1 attitude in Euler angles [yaw, pitch, roll], rotation sequency is zyx.
        motion_def: nx6 motion definitions. Each row defines command of a motion segment.
            motion_def[:,0]: motion type.
                1: motion params directly give body frame velocity and Euler angles change rate.
                2: absolute att and absolute vel to rech.
                3: relative att and vel change.
                4: absolute att, relative vel.
                5: relative att, absolute vel.
            motion_def[:,1:5]: motion params = [Att command, vel along body x axis command].
            motion_def[:,5] = maximum time for the given segment, sec.
        output_def: [[simulation_over_sample_rate imu_freq];[1 gps_freq] or [2 odo_freq]], Hz.
        mobility: [max_acceleration, max_angular_acceleration, max_angular_velocity]
        ref_frame: reference frame used as the navigation frame,
            0: NED (default).
            1: a virtual inertial frame, with constant g and z axis pointing along g.
        magnet:
            False: Geomagnetic field in the body frame will not be calculaed.
            True: Geomagnetic field in the body frame will be calculaed.
                For ref_frame==0, N is geographic north, and there is declination;
                For ref_frame==1, there is no declination.
    Returns:
        path_results. Resutls of path generation.
            'status':  True: Everything is OK.
                        False:
            'imu':      True/ideal IMU measurements. Each line is organized as [index, acc, gyro],
                        index is an interger and increases at imu_output_freq.
            'nav':      True position, velocity and attitude (Euler angles, ZYX).
                        ref_frame==0, [index, absolute_position_lla, velocity_in_body_frame, attitude],
                        ref_frame==1, [index, absolute_position_xyz, velocity_in_body_frame, attitude],
                        Index is synced with index in mimu.csv.
            'mag':      True/ideal geomagneti field in the body frame.
                        [index, magx, magy, magz], uT, index synced with mimu.csv index.
            'gps':      True GPS measurements.
                        ref_frame==0, [index, absolute_position_lla, velocity_in_navigation_frame],
                        ref_frame==1, [index, absolute_position_xyz, velocity_in_navigation_frame],
                        GPS data are down sampled to gps_freq, index synced with mimu.csv index.
            'odo':      True odometer measurements.
                        [index, travel_distance, velocity_in_body_frame].
                        Odometry are down sampled to odd_freq, index synced with mimu.csv index.
    """
    ### path generation results
    path_results = {'status': True,
                    'imu': [],
                    'nav': [],
                    'mag': [],
                    'gps': [],
                    'odo': []}

    ### sim freq and data output freq
    out_freq = output_def[0, 1]     # IMU output frequency
    sim_osr = output_def[0, 0]      # simulation over sample ratio w.r.t IMU output freq
    sim_freq = sim_osr * out_freq   # simulation frequency
    dt = 1.0 / sim_freq             # simulation period

    ### Path gen command filter to make trajectory smoother
    alpha = 0.9                     # for the low pass filter of the motion commands
    filt_a = alpha * np.eye(3)
    filt_b = (1-alpha) * np.eye(3)
    max_acc = mobility[0]           # 10.0m/s2, max acceleratoin
    max_dw = mobility[1]            # 0.5rad/s2    # max angular acceleration, rad/s/s
    max_w = mobility[2]             # 1.0rad/s       # max angular velocity, rad/s
    kp = 5.0                        # kp and kd are PD controller params
    kd = 10.0
    att_converge_threshold = 1e-4   # threshold to determine if the command is completed
    vel_converge_threshold = 1e-4
    att_dot = np.zeros(3)           # Euler angle change rate
    vel_dot_b = np.zeros(3)         # Velocity change rate in the body frame

    ### convert time duration to simulation cycles
    sim_count_max = 0
    for i in range(0, motion_def.shape[0]):
        seg_count = motion_def[i, 5] * out_freq         # max count for this segment
        sim_count_max += seg_count                      # data count of all segments
        motion_def[i, 5] = round(seg_count * sim_osr)   # simulation count

    ### create output arrays
    sim_count_max = int(sim_count_max)
    imu_data = np.zeros((sim_count_max, 7))
    nav_data = np.zeros((sim_count_max, 10))
    enable_gps_or_odo = False
    if output_def.shape[0] >= 2:
        if output_def[1, 0] == 1:
            enable_gps_or_odo = True
            gps_data = np.zeros((sim_count_max, 7))
            output_def[1, 1] = sim_osr * round(out_freq / output_def[1, 1])
        elif output_def[1, 0] == 2:
            enable_gps_or_odo = True
            odo_data = np.zeros((sim_count_max, 5))
            output_def[1, 1] = sim_osr * round(out_freq / output_def[1, 1])
        else:
            output_def[1, 0] = -1
    if magnet:
        mag_data = np.zeros((sim_count_max, 4))

    ### start computations
    sim_count = 0               # number of total simulation data
    acc_sum = np.zeros(3)       # accum of over sampled simulated acc data
    gyro_sum = np.zeros(3)      # accum of over sampled simulated gyro data
    odo_dist = 0                # accum of travel distance
    ## initialize
    pos_n = ini_pos_vel_att[0:3]                # ini pos, LLA
    vel_b = ini_pos_vel_att[3:6]                # ini vel
    att = ini_pos_vel_att[6:9]                  # ini att
    c_nb = attitude.euler2dcm(att, 'zyx').T     # b to n
    vel_n = c_nb.dot(vel_b)
    pos_delta_n = np.zeros(3)                   # pos change
    earth_param = geoparams.geo_param(pos_n)    # geo parameters
    g = earth_param[2]                          # local gravity at ini pos
    if magnet:                                  # geomagnetic parameters at the initial position
        gm = geomag.GeoMag("WMM.COF")
        geo_mag = gm.GeoMag(pos_n[0]/D2R, pos_n[1]/D2R, pos_n[2]) # units in nT and deg
        geo_mag_n = np.array([geo_mag.bx, geo_mag.by, geo_mag.bz])
        geo_mag_n = geo_mag_n / 1000.0          # nT to uT
        if ref_frame == 1:                      # remove inclination
            geo_mag_n[0] = math.sqrt(geo_mag_n[0]*geo_mag_n[0] + geo_mag_n[1]*geo_mag_n[1])
            geo_mag_n[1] = 0.0
    ## start trajectory generation
    if ref_frame == 1:      # if using virtual inertial frame, convert LLA to ECEF xyz
        pos_n = geoparams.lla2xyz(pos_n)
    idx_high_freq = 0       # data index for imu, nav, mag
    idx_low_freq = 0        # data index for gps, odo
    for i in range(0, motion_def.shape[0]):
        com_type = round(motion_def[i, 0])     # command type of this segment
        # get command of this segment
        '''if i == 2:
            print("haha")'''
        motion_com = parse_motion_def(motion_def[i], att, vel_b)
        if com_type == 1:
            att_dot_com = motion_com[0]
            vel_dot_com = motion_com[1]
        else:
            att_com = motion_com[0]
            vel_com_b = motion_com[1]
        # initialize the filter states to last att and vel
        att_com_filt = att
        vel_com_b_filt = vel_b
        # generate trajectory according to the command of this segment
        sim_count_max = sim_count + motion_def[i, 5]    # max cycles to execute command of this seg
        com_complete = 0                                # complete command of this seg, go to next
        while (sim_count < sim_count_max) and (com_complete == 0):
            # update simulation counter
            sim_count += 1
            # determine the inputs
            if com_type == 1:
                att_dot = filt_a.dot(att_dot) + filt_b.dot(att_dot_com)         # filter input
                vel_dot_b = filt_a.dot(vel_dot_b) + filt_b.dot(vel_dot_com)
                # vel_dot_b = np.array(vel_dot_com)
            else:
                att_com_filt = filt_a.dot(att_com_filt) + filt_b.dot(att_com)   # filter command
                vel_com_b_filt = filt_a.dot(vel_com_b_filt) + filt_b.dot(vel_com_b)
                # Close the loop. Velocity change is acceleration and sudden change in acceleration
                # is reasonable. Attitude change is angular velocity and sudden change in angular
                # velocity means inifite torque, which is unreasonable. So a simple PD controller
                # is used here to track the commanded attitude.
                # acc
                vel_dot_b = (vel_com_b_filt - vel_b) / dt
                vel_dot_b[vel_dot_b > max_acc] = max_acc          # limit acceleration
                vel_dot_b[vel_dot_b < -max_acc] = -max_acc
                # w
                att_dot_dot = kp*(att_com - att) + kd*(0 - att_dot) # feedback control
                att_dot_dot[att_dot_dot > max_dw] = max_dw          # limit w change rate
                att_dot_dot[att_dot_dot < -max_dw] = -max_dw
                att_dot = att_dot + att_dot_dot*dt
                att_dot[att_dot > max_w] = max_w                    # limit att change rate
                att_dot[att_dot < -max_w] = -max_w
                # Complete the command of this segment?
                if (np.sqrt(np.dot(att-att_com, att-att_com)) < att_converge_threshold and
                        np.sqrt(np.dot(vel_b-vel_com_b, vel_b-vel_com_b)) < vel_converge_threshold):
                    com_complete = 1
                    #att_dot = (att_com - att) / dt
                    #vel_dot_b = (vel_com_b - vel_b) / dt
            # compute IMU outputs
            imu_results = calc_true_sensor_output(pos_n+pos_delta_n, vel_b, att, c_nb, vel_dot_b,
                                                  att_dot, ref_frame, g)
            acc = imu_results[0]
            gyro = imu_results[1]
            pos_dot_n = imu_results[3]  # lla change rate if NED, vel_n if virtual inertial
            # update IMU results
            acc_sum = acc_sum + acc
            gyro_sum = gyro_sum + gyro
            # update GPS results

            # update odometer results
            odo_vel = vel_b

            # Write the results. Simulation data are down sampled according to freq specified
            # in output_def.
            # IMU measurement and navigation results
            if (sim_count % sim_osr) == 0:
                # average over sampled IMU data
                acc_avg = acc_sum / sim_osr
                gyro_avg = gyro_sum / sim_osr
                # write to files
                #imu_data[idx_high_freq, :] = np.hstack((idx_high_freq, acc_avg, gyro_avg))
                imu_data[idx_high_freq, 0] = idx_high_freq
                imu_data[idx_high_freq, 1] = acc_avg[0]
                imu_data[idx_high_freq, 2] = acc_avg[1]
                imu_data[idx_high_freq, 3] = acc_avg[2]
                imu_data[idx_high_freq, 4] = gyro_avg[0]
                imu_data[idx_high_freq, 5] = gyro_avg[1]
                imu_data[idx_high_freq, 6] = gyro_avg[2]
                nav_data[idx_high_freq, 0] = idx_high_freq
                # nav data
                nav_data[idx_high_freq, 1] = pos_n[0] + pos_delta_n[0]
                nav_data[idx_high_freq, 2] = pos_n[1] + pos_delta_n[1]
                nav_data[idx_high_freq, 3] = pos_n[2] + pos_delta_n[2]
                nav_data[idx_high_freq, 4] = vel_b[0]
                nav_data[idx_high_freq, 5] = vel_b[1]
                nav_data[idx_high_freq, 6] = vel_b[2]
                nav_data[idx_high_freq, 7] = att[0]
                nav_data[idx_high_freq, 8] = att[1]
                nav_data[idx_high_freq, 9] = att[2]
                # next cycle
                acc_sum = np.zeros(3)
                gyro_sum = np.zeros(3)
                # update magnetometer results and write to file
                if magnet:
                    geo_mag_b = c_nb.T.dot(geo_mag_n)
                    #mag_data[idx_high_freq, :] = np.hstack((idx_high_freq, geo_mag_b))
                    mag_data[idx_high_freq, 0] = idx_high_freq
                    mag_data[idx_high_freq, 1] = geo_mag_b[0]
                    mag_data[idx_high_freq, 2] = geo_mag_b[1]
                    mag_data[idx_high_freq, 3] = geo_mag_b[2]
                # index increment
                idx_high_freq += 1
            # GPS or odometer measurement
            if enable_gps_or_odo:
                if (sim_count % output_def[1, 1]) == 0:     # measurement period
                    if output_def[1, 0] == 1:               # GPS
                        gps_data[idx_low_freq, 0] = idx_low_freq
                        gps_data[idx_low_freq, 1] = pos_n[0] + pos_delta_n[0]
                        gps_data[idx_low_freq, 2] = pos_n[1] + pos_delta_n[1]
                        gps_data[idx_low_freq, 3] = pos_n[2] + pos_delta_n[2]
                        gps_data[idx_low_freq, 4] = vel_n[0]
                        gps_data[idx_low_freq, 5] = vel_n[1]
                        gps_data[idx_low_freq, 6] = vel_n[2]
                    elif output_def[1, 0] == 2:             # odometer
                        #odo_data[idx_low_freq, :] = np.hstack((idx_low_freq,
                        #                                       odo_dist, odo_vel))
                        odo_data[idx_low_freq, 0] = idx_low_freq
                        odo_data[idx_low_freq, 1] = odo_dist
                        odo_data[idx_low_freq, 2] = odo_vel[0]
                        odo_data[idx_low_freq, 3] = odo_vel[1]
                        odo_data[idx_low_freq, 4] = odo_vel[2]
                    # index increment
                    idx_low_freq += 1
            pos_delta_n = pos_delta_n + pos_dot_n*dt    # accumulated pos change
            odo_dist = odo_dist + np.sqrt(np.dot(vel_b, vel_b))*dt
            vel_b = vel_b + vel_dot_b*dt
            att = att + att_dot*dt
            c_nb = attitude.euler2dcm(att, 'zyx').T     # b to n
            vel_n = c_nb.dot(vel_b)

        # if command is completed, att_dot and vel_dot should be set to zero
        if com_complete == 1:
            att_dot = np.zeros(3)
            vel_dot_b = np.zeros(3)
    # return generated data
    path_results['imu'] = imu_data[0:idx_high_freq, :]
    path_results['nav'] = nav_data[0:idx_high_freq, :]
    if magnet:
        path_results['mag'] = mag_data[0:idx_high_freq, :]
    if output_def[1, 0] == 1:
        path_results['gps'] = gps_data[0:idx_low_freq, :]
    elif output_def[1, 0] == 2:
        path_results['odo'] = odo_data[0:idx_low_freq, :]
    return path_results

def calc_true_sensor_output(pos_n, vel_b, att, c_nb, vel_dot_b, att_dot, ref_frame, g):
    """
    Calculate true IMU results from attitude change rate and velocity
    change rate.
    attitude change rate is input in the form of Euler angle derivatives and
    converted into angular velocity. Velocity change rate is expressed in
    the body frame. Position change rate is also calculated. If simulation is
    done in the NED frame, the position change rate is in the form of Lat, Lon
    and alt derivatives. Otherwise, it is given in m/s.
    Args:
        pos_n: For NED, it is the absolute LLA position. Otherwise, it is relative
            motion.
        vel_b: Velocity in the body frame, m/s.
        att: Euler angles, [yaw pitch roll], rot seq is ZYX, rad.
        c_nb: Transformation matrix from b to n corresponding to att.
        vel_dot_b: Velocity change rate in the body frame, m/s/s
        att_dot: Euler angle change rate, [yaw_d, pitch_d, roll_d], rad/s
        ref_frame: See doc of function PathGen.
        g: Gravity, only used when ref_frame==1, m/s/s.
    Returns:
        [0]: 3x1 true accelerometer output in the body frame, m/s/s
        [1]: 3x1 true gyro output in the body frame, rad/s
        [2]: 3x1 velocity change rate in the navigation frame, m/s/s
        [3]: 3x1 position change rate in the navigation frame, m/s
    """
    # velocity in N
    vel_n = c_nb.dot(vel_b)

    # Calculate rotation rate of n w.r.t e in n and e w.r.t i in n
    # For the NED frame, the NED frame rotation and Earth rotation rate is calculated
    # For the virtual inertial frame, they are not needed and simply set to zeros.
    w_en_n = np.zeros(3)
    w_ie_n = np.zeros(3)
    if ref_frame == 0:
        earth_param = geoparams.geo_param(pos_n)
        rm = earth_param[0]
        rn = earth_param[1]
        g = earth_param[2]
        sl = earth_param[3]
        cl = earth_param[4]
        w_ie = earth_param[5]
        rm_effective = rm + pos_n[2]
        rn_effective = rn + pos_n[2]
        gravity = np.array([0, 0, -g])
        w_en_n[0] = vel_n[1] / rn_effective              # wN
        w_en_n[1] = -vel_n[0] / rm_effective             # wE
        w_en_n[2] = -vel_n[1] * sl /cl / rn_effective    # wD
        w_ie_n[0] = w_ie * cl
        w_ie_n[2] = -w_ie * sl
    else:
        gravity = [0, 0, -g]

    # Calculate rotation rate of b w.r.t n expressed in n.
    # Calculate rotation rate from Euler angle derivative using ZYX rot seq.
    sh = math.sin(att[0])
    ch = math.cos(att[0])
    w_nb_n = np.zeros(3)
    w_nb_n[0] = -sh*att_dot[1] + c_nb[0, 0]*att_dot[2]
    w_nb_n[1] = ch*att_dot[1] + c_nb[1, 0]*att_dot[2]
    w_nb_n[2] = att_dot[0] + c_nb[2, 0]*att_dot[2]
    # Calculate rotation rate from rotation quaternion
    # w_nb_n = np.zeros(3)

    # Velocity derivative
    vel_dot_n = c_nb.dot(vel_dot_b) + attitude.cross3(w_nb_n, vel_n)
    # Position derivative
    pos_dot_n = np.zeros(3)
    if ref_frame == 0:
        pos_dot_n[0] = vel_n[0] / rm_effective      # Lat
        pos_dot_n[1] = vel_n[1] / rn_effective / cl # Lon
        pos_dot_n[2] = -vel_n[2]                    # Alt
    else:
        pos_dot_n[0] = vel_n[0]
        pos_dot_n[1] = vel_n[1]
        pos_dot_n[2] = vel_n[2]
    # Gyroscope output
    gyro = c_nb.T.dot(w_nb_n + w_en_n + w_ie_n)
    # Acceleration output
    w_ie_b = c_nb.T.dot(w_ie_n)
    acc = vel_dot_b + attitude.cross3(w_ie_b+gyro, vel_b) + c_nb.T.dot(gravity)
    return acc, gyro, vel_dot_n, pos_dot_n

def parse_motion_def(motion_def_seg, att, vel):
    """
    Parse the command of a segment in motion_def.
    Args:
        motion_def_seg: a segment in motion_def
        att: current attitude
        vel: current velocity
    Returns:
        [0]: Target attitude
        [1]: Target velocity
    """
    if motion_def_seg[0] == 1:
        att_com = [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = [motion_def_seg[4], 0, 0]
    elif motion_def_seg[0] == 2:   # abs att and abs vel
        att_com = [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = [motion_def_seg[4], 0, 0]
    elif motion_def_seg[0] == 3:   # rel att and rel vel
        att_com = att + [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = vel + [motion_def_seg[4], 0, 0]
    elif motion_def_seg[0] == 4:   # abs att and rel vel
        att_com = [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = vel + [motion_def_seg[4], 0, 0]
    elif motion_def_seg[0] == 5:   # rel att and abs vel
        att_com = att + [motion_def_seg[1], motion_def_seg[2], motion_def_seg[3]]
        vel_com = [motion_def_seg[4], 0, 0]
    return att_com, vel_com
