# -*- coding: utf-8 -*-
# Fielname = imu_sim.py

"""
SIM class.
Created on 2017-12-19
@author: dongxiaoguang
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from mpl_toolkits.mplot3d import Axes3D
from ..pathgen import pathgen
from ..attitude import attitude
from ..kml_gen import kml_gen

D2R = math.pi/180
# built-in mobility
high_mobility = np.array([1.0, 0.5, 2.0])   # m/s/s, rad/s/s, rad/s

class Sim(object):
    '''
    Simulation class.
    '''
    def __init__(self, fs, imu, motion_def, ref_frame=0,\
                 mode=None, env=None, algorithm=None):
        '''
        Args:
            fs: [fs_imu, fs_gps, fs_mag], Hz.
                fs_imu: The sample rate of IMU.
                fs_gps: The sample rate of GPS.
                fs_mag: not used now. The sample rate of the magnetometer is
                    the same as that of the imu.

            imu: See IMU in imu_model.py

            motion_def: a .csv file to define the waypoints
                row 1: header line for initial states
                row 2: initial states, which include:
                    col 1-3: initial position (LLA, deg, meter),
                    col 4-6: initial velocity in body frame(m/s),
                    col 7-9: initial attitude (Euler angles, deg)
                row 3: header line for motion command
                row >=2: motion commands, which include
                    col 1: motion type. The following types are supported:
                        1: Euler angles change rate and body frame velocity change rate.
                        2: absolute att and absolute vel to rech.
                        3: relative att and vel change.
                        4: absolute att, relative vel.
                        5: relative att, absolute vel.
                    col 2-7: motion command (deg, m/s).
                        [yaw, pitch, roll, vx (velocity along body x axis), reserved, reserved].
                        For motion type 1, the above Euler angles and velocity should be
                        change rate, corresponding units are (deg/s, m/s/s).
                    col 8: maximum time for the given segment, sec.
                    col 9: reserved.

            ref_frame: reference frame used as the navigation frame,
                        0: NED (default), with x axis pointing along geographic north,
                           y axis pointing eastward,
                           z axis pointing downward.
                        1: a virtual inertial frame with constant g,
                           x axis pointing along magnetic north,
                           z axis pointing along g,
                           y axis completing a right-handed coordinate system.
                           Notice: For this virtual inertial frame, position is indeed the sum of
                           the initial position in ecef and the relative position in the virutal
                           inertial frame.

            mode: simu mode could be a string to specify a built-in mode:
                'flight':
                ...
                This is not implemented yet. A built-in 'high_mobility' mode is used.
                or a numpy array of size (3,) to customize the sim mode.
                    [max_acceleration, max_angular_acceleration, max_angular_velocity],
                    in units of [m/s/s, deg/s/s, deg/s]

            env: vibration model. There are three kinds of vibration models:
                '[nx ny nz]g-random': normal-distribution random vibration, rms is n*9.8 m/s^2
                '[nx ny nz]-random': normal-distribution random vibration, rms is n m/s^2
                '[nx ny nz]g-mHz-sinusoidal': sinusoidal vibration of m Hz, amplitude is n*9.8 m/s^2
                '[nx ny nz]-mHz-sinusoidal': sinusoidal vibration of m Hz, amplitude is n m/s^2
                numpy array of (n,4): single-sided PSD. Each row is [freq, x, y, z], m^2/s^4/Hz

            algorithm: a user defined algorithm.
        '''
        ########## configure simulation ##########
        self.sim_count = 1          # simulation count
        self.sim_complete = False   # simulation complete successfully
        self.sim_results = False    # simulation results is generated
        # sample rate
        self.fs = Sim_data(name='fs',\
                           description='Sample frequency of imu',\
                           units=['Hz'],\
                           plottable=False)
        self.fs.data = fs[0]
        # reference frame
        self.ref_frame = Sim_data(name='ref_frame',\
                                  description='Reference frame',\
                                  plottable=False)
        if ref_frame == 0 or ref_frame == 1:
            self.ref_frame.data = ref_frame
        else:
            self.ref_frame.data = 0      # default frame is NED
        # IMU model
        self.imu = imu              # imu config

        ########## possible data generated by simulation ##########
        # reference data
        self.time = Sim_data(name='time',\
                             description='sample time',\
                             units=['sec'])
        self.gps_time = Sim_data(name='gps_time',\
                                 description='GPS sample time',\
                                 units=['sec'])
        self.ref_pos = Sim_data(name='ref_pos',\
                                description='true pos in the navigation frame',\
                                units=['rad', 'rad', 'm'],\
                                output_units=['deg', 'deg', 'm'],\
                                legend=['ref_pos_x', 'ref_pos_y', 'ref_pos_z'])
        if self.ref_frame.data == 1:
            self.ref_pos.units = ['m', 'm', 'm']
            self.ref_pos.output_units = ['m', 'm', 'm']
        self.ref_vel = Sim_data(name='ref_vel',\
                                description='true vel in the body frame',\
                                units=['m/s', 'm/s', 'm/s'],\
                                legend=['ref_vel_x', 'ref_vel_y', 'ref_vel_z'])
        self.ref_att_euler = Sim_data(name='ref_att_euler',\
                                description='true attitude (Euler angles, ZYX)',\
                                units=['rad', 'rad', 'rad'],\
                                output_units=['deg', 'deg', 'deg'],\
                                legend=['ref_Yaw', 'ref_Pitch', 'ref_Roll'])
        self.ref_att_quat = Sim_data(name='ref_att_quat',\
                                     description='true attitude (quaternion)',\
                                     legend=['q0', 'q1', 'q2', 'q3'])
        self.ref_gyro = Sim_data(name='ref_gyro',\
                                 description='true angular velocity',\
                                 units=['rad/s', 'rad/s', 'rad/s'],\
                                 output_units=['deg/s', 'deg/s', 'deg/s'],\
                                 legend=['ref_gyro_x', 'ref_gyro_y', 'ref_gyro_z'])
        self.ref_accel = Sim_data(name='ref_accel',\
                                  description='True accel',\
                                  units=['m/s^2', 'm/s^2', 'm/s^2'],\
                                  legend=['ref_accel_x', 'ref_accel_y', 'ref_accel_z'])
        self.ref_gps = Sim_data(name='ref_gps',\
                                description='true GPS pos/vel',\
                                units=['rad', 'rad', 'm', 'm/s', 'm/s', 'm/s'],\
                                output_units=['deg', 'deg', 'm', 'm/s', 'm/s', 'm/s'],\
                                legend=['ref_gps_x', 'ref_gps_y', 'ref_gps_z',\
                                        'ref_gps_vx', 'ref_gps_vy', 'ref_gps_vz'])
                                # downsampled true pos/vel
        if self.ref_frame.data == 1:
            self.ref_gps.units = ['m', 'm', 'm', 'm/s', 'm/s', 'm/s']
            self.ref_gps.output_units = ['m', 'm', 'm', 'm/s', 'm/s', 'm/s']
        self.ref_mag = Sim_data(name='ref_mag',\
                                description='true magnetic field',\
                                units=['uT', 'uT', 'uT'],\
                                legend=['ref_mag_x', 'ref_mag_y', 'ref_mag_z'])

        # simulation results
        self.algo_time = Sim_data(name='algo_time',\
                             description='sample time from algo',\
                             units=['sec'])
        self.pos = Sim_data(name='pos',\
                            description='simulation position from algo',\
                            units=self.ref_pos.units,\
                            legend=['pos_x', 'pos_y', 'pos_z'])
        self.vel = Sim_data(name='vel',\
                            description='simulation velocity from algo',\
                            units=['m/s', 'm/s', 'm/s'],\
                            legend=['vel_x', 'vel_y', 'vel_z'])
        self.att_quat = Sim_data(name='att_quat',\
                                 description='simulation attitude (quaternion)  from algo',\
                                 legend=['q0', 'q1', 'q2', 'q3'])
        self.att_euler = Sim_data(name='att_euler',
                                  description='simulation attitude (Euler, ZYX)  from algo',\
                                  units=['rad', 'rad', 'rad'],\
                                  output_units=['deg', 'deg', 'deg'],\
                                  legend=['Yaw', 'Pitch', 'Roll'])
        self.gyro = Sim_data(name='gyro',\
                             description='gyro measurements',\
                             units=['rad/s', 'rad/s', 'rad/s'],\
                             output_units=['deg/s', 'deg/s', 'deg/s'],\
                             legend=['gyro_x', 'gyro_y', 'gyro_z'])
        self.accel = Sim_data(name='accel',\
                              description='accel measurements',\
                              units=['m/s^2', 'm/s^2', 'm/s^2'],\
                              legend=['accel_x', 'accel_y', 'accel_z'])
        self.gps = Sim_data(name='gps',\
                            description='GPS measurements',\
                            units=self.ref_gps.units,\
                            output_units=self.ref_gps.output_units,\
                            legend=['gps_x', 'gps_y', 'gps_z', 'gps_vx', 'gps_vy', 'gps_vz'])
        self.mag = Sim_data(name='mag',\
                            description='magnetometer measurements',\
                            units=['uT', 'uT', 'uT'],\
                            legend=['mag_x', 'mag_y', 'mag_z'])
        self.wb = Sim_data(name='wb',\
                           description='gyro bias estimation',\
                           units=['rad/s', 'rad/s', 'rad/s'],\
                           output_units=['deg/s', 'deg/s', 'deg/s'],\
                           legend=['gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z'])
        self.ab = Sim_data(name='ab',\
                           description='accel bias estimation',\
                           units=['m/s^2', 'm/s^2', 'm/s^2'],\
                           legend=['accel_bias_x', 'accel_bias_y', 'accel_bias_z'])
        self.allan_t = Sim_data(name='allan_t',\
                             description='Allan var time',\
                             units=['s'])
        self.ad_gyro = Sim_data(name='ad_gyro',\
                                description='Allan deviation of gyro',\
                                units=['rad/s', 'rad/s', 'rad/s'],\
                                logx=True, logy=True,\
                                legend=['AD_wx', 'AD_wy', 'AD_wz'])
        self.ad_accel = Sim_data(name='ad_accel',\
                                 description='Allan deviation of accel',\
                                 units=['m/s^2', 'm/s^2', 'm/s^2'],\
                                 logx=True, logy=True,\
                                 legend=['AD_ax', 'AD_ay', 'AD_az'])

        ########## supported data ##########
        '''
        each item in the supported data should be either scalar or numpy.array of size(n, dim),
        or a dict of the above two, dict keys are simulatoin count: 0, 1, 2, 3, ...
        n is the sample number, dim is a set of data at time tn. For example, accel is nx3,
        att_quat is nx4, allan_t is (n,)
        '''
        # data that can be used as input to the algorithm
        '''
        There are two kinds of data that can be used as algorithm input: constant that stays the
        same for all simulations, varying that varies for different simulations.
        For example, fs stays the same for all simulations, reference data stay the same for all
        simulations, and sensor data vary for different simulations.
        '''
        self.supported_in_constant = {
            self.fs.name: self.fs,
            self.time.name: self.time,
            self.ref_frame.name: self.ref_frame,
            self.ref_pos.name: self.ref_pos,
            self.ref_vel.name: self.ref_vel,
            self.ref_att_euler.name: self.ref_att_euler,
            self.ref_gyro.name: self.ref_gyro,
            self.ref_accel.name: self.ref_accel}
        self.supported_in_varying = {
            self.gyro.name: self.gyro,
            self.accel.name: self.accel}
        if self.imu.gps:    # optional GPS
            self.supported_in_constant[self.ref_gps.name] = self.ref_gps
            self.supported_in_constant[self.gps_time.name] = self.gps_time
            self.supported_in_varying[self.gps.name] = self.gps
        if self.imu.magnetometer:   # optional mag
            self.supported_in_constant[self.ref_mag.name] = self.ref_mag
            self.supported_in_varying[self.mag.name] = self.mag
        # algorithm output that can be handled by Sim class
        # algorithm outputs vary for different simulations
        self.supported_out = {
            self.algo_time.name: self.algo_time,
            self.pos.name: self.pos,
            self.vel.name: self.vel,
            self.att_quat.name: self.att_quat,
            self.att_euler.name: self.att_euler,
            self.wb.name: self.wb,
            self.ab.name: self.ab,
            self.allan_t.name: self.allan_t,
            self.ad_gyro.name: self.ad_gyro,
            self.ad_accel.name: self.ad_accel}

        # all available data
        self.res = {}
        # all available data for plot
        self.supported_plot = {}
        # error terms we are interested in
        self.interested_error = {self.att_euler.name: 'angle',
                                 self.pos.name: None,
                                 self.vel.name: None}
        # associated data mapping
        self.data_map = {self.ref_att_euler.name: [self.ref_att_quat, self.__euler2quat_zyx],
                         self.ref_att_quat.name: [self.ref_att_euler, self.__quat2euler_zyx],
                         self.att_euler.name: [self.att_quat, self.__euler2quat_zyx],
                         self.att_quat.name: [self.att_euler, self.__quat2euler_zyx]}

        # summary
        self.sum = ''

        ########## read motion definition ##########
        self.ini_pos_n = None
        self.ini_vel_b = None
        self.ini_att = None
        self.motion_def = None
        self.parse_motion(motion_def)

        ########## generate GPS or not ##########
        # output definitions
        self.output_def = np.array([[1.0, self.fs.data], [1.0, self.fs.data]])
        if self.imu.gps:
            self.output_def[1, 0] = 1.0
            self.output_def[1, 1] = fs[1]
        else:
            self.output_def[1, 0] = -1.0

        ########## sim mode-->vehicle maneuver capability ##########
        self.parse_mode(mode)

        ########## environment-->vibraition params ##########
        self.vib_def = None
        self.parse_env(env)

        ########## check algorithm ##########
        self.algo = algorithm
        if algorithm is not None:
            self.check_algo()

    def run(self, num_times=1):
        '''
        run simulation.
        Args:
            num_times: run the simulation for num_times times with given IMU error model.
        '''
        self.sim_count = int(num_times)
        if self.sim_count < 1:
            self.sim_count = 1
        ########## generate reference data ##########
        rtn = pathgen.path_gen(np.hstack((self.ini_pos_n, self.ini_vel_b, self.ini_att)),
                               self.motion_def, self.output_def, self.mobility,
                               self.ref_frame.data, self.imu.magnetometer)
        # save reference data
        self.time.data = rtn['nav'][:, 0] / self.fs.data
        self.ref_pos.data = rtn['nav'][:, 1:4]
        self.ref_vel.data = rtn['nav'][:, 4:7]
        self.ref_att_euler.data = rtn['nav'][:, 7:10]
        self.ref_accel.data = rtn['imu'][:, 1:4]
        self.ref_gyro.data = rtn['imu'][:, 4:7]
        if self.imu.gps:
            self.gps_time.data = rtn['gps'][:, 0] / self.fs.data
            self.ref_gps.data = rtn['gps'][:, 1:7]
        if self.imu.magnetometer:
            self.ref_mag.data = rtn['mag'][:, 1:4]
        ########## simulation ##########
        for i in range(0, self.sim_count):
            # generate sensor data
            self.accel.data[i] = pathgen.acc_gen(self.fs.data, self.ref_accel.data,
                                                 self.imu.accel_err, self.vib_def)
            # np.savetxt(i_str+'_accel.txt', self.accel[i])
            self.gyro.data[i] = pathgen.gyro_gen(self.fs.data, self.ref_gyro.data,\
                                                 self.imu.gyro_err)
            # np.savetxt(i_str+'_gyro.txt', self.gyro[i])
            if self.imu.gps:
                self.gps.data[i] = pathgen.gps_gen(self.ref_gps.data, self.imu.gps_err,\
                                                   self.ref_frame.data)
                # np.savetxt(i_str+'_gps.txt', self.gps[i])
            if self.imu.magnetometer:
                self.mag.data[i] = pathgen.mag_gen(self.ref_mag.data, self.imu.mag_err)
                # np.savetxt(i_str+'_mag.txt', self.mag[i])
            # run specified algorithm
            if self.algo is not None:
                # input
                algo_input = []
                for j in self.algo.input:
                    if j in self.supported_in_constant:
                        algo_input.append(self.supported_in_constant[j].data)
                    else:
                        algo_input.append(self.supported_in_varying[j].data[i])
                # run
                self.algo.run(algo_input)
                # output
                algo_results = self.algo.get_results()
                nout = len(self.algo.output)
                for j in range(nout):
                    self.supported_out[self.algo.output[j]].data[i] = algo_results[j]
                # reset algorithm for next run
                self.algo.reset()
        # simulation complete successfully
        self.sim_complete = True

    def results(self, data_dir=None, gen_kml=False):
        '''
        simulation results.
        Args:
            data_dir: if not None, save simulation data to files.
                if data_dir is a valid directory, data files will be saved in data_idr,
                else, data files will be saved in the default directory './data/'
            gen_kml: generate .kml files using the reference position and simulation position
        Returns: a dict contains all simulation results.
        '''
        if self.sim_complete:
            '''
            generate a dict to tell what simulation results can be acquired, and what results
            can be plotted.
            '''
            #### generate results containing all available data
            # data from pathgen are available after simulation
            self.res = self.supported_in_constant.copy()
            self.res.update(self.supported_in_varying)
            # add user specified algorithm output to results
            if self.algo is not None:
                for i in self.algo.output:
                    self.res[i] = self.supported_out[i]
            # add associated data.
            # for example, if quaternion is available, Euler angles will be generated
            self.__add_associated_data_to_results()

            #### generate supported plot
            self.supported_plot = self.res.copy()
            self.supported_plot.pop('fs')
            self.supported_plot.pop('ref_frame')
            # print(self.res)
            # print(self.supported_plot)

            '''
            Save results to file.
            Simulation results include a summary file containing statistics of the simulation
            and .csv files containing all data generated.
            '''
            # check data dir
            if data_dir is not None:    # data_dir specified, meaning to save .csv files
                data_dir = self.check_data_dir(data_dir)
                # save data files
                for i in self.supported_plot:
                    self.supported_plot[i].save_to_file(data_dir)
            if gen_kml is True:       # want to gen kml without specifying the data_dir
                if data_dir is None:
                    data_dir = self.check_data_dir(data_dir)
                self.__save_kml_files(data_dir)
            # simulation summary and save summary to file
            self.__summary(data_dir)  # generate summary

            self.sim_results = True
            return self.res
        else:
            print("Call Sim.run() to run the simulaltion first.")
            return None

    def __save_kml_files(self, data_dir):
        '''
        generate kml files from reference position and simulation position.
        Args:
            data_dir: kml files are saved in data_dir
        '''
        convert_xyz_to_lla = False
        if self.ref_frame.data == 1:
            convert_xyz_to_lla = True
        # ref position
        if 'ref_pos' in self.res:
            kml_contents = kml_gen.kml_gen(self.res['ref_pos'].data,\
                                    name='ref_pos',\
                                    convert_to_lla=convert_xyz_to_lla)
            kml_file = data_dir + '//ref_pos.kml'
            fp = open(kml_file, 'w')
            fp.write(kml_contents)
            fp.close()
        # simulation position
        if 'pos' in self.res:
            for i in range(0, len(self.res['pos'].data)):
                pos_name = 'pos_' + str(i)
                kml_contents = kml_gen.kml_gen(self.res['pos'].data[i],\
                                        name=pos_name,\
                                        convert_to_lla=convert_xyz_to_lla)
                kml_file = data_dir + '//' + pos_name + '.kml'
                fp = open(kml_file, 'w')
                fp.write(kml_contents)
                fp.close()

    def __summary(self, data_dir):
        '''
        Summary of sim results.
        '''
        #### simulation config
        # sample frequency
        self.sum += self.fs.description + ': [' +\
                    self.fs.name + '] = ' +\
                    str(self.fs.data) + ' ' +\
                    self.fs.units[0] + '\n'
        # simulation time duration
        self.sum += 'Simulation time duration: ' + \
                    str(len(self.time.data)/self.fs.data) + ' s' + '\n'
        # simulation times
        self.sum += 'Simulation runs: ' + str(self.sim_count) + '\n'
        if data_dir is not None:
            self.sum += 'Simulation results are saved to ' + data_dir + '\n'
        #### supported plot
        self.sum += "\nPlots of the following results are supported:\n"
        for i in self.supported_plot:
            self.sum += '\t' + i + ': ' + self.supported_plot[i].description + '\n'

        #### error of algorithm output
        if self.algo is not None:
            for i in self.algo.output:
                # is is not in interested data but has associated data?
                if i not in self.interested_error and i in self.data_map:
                    i = self.data_map[i][0].name
                ref_name = 'ref_' + i
                if i in self.interested_error and ref_name in self.res:
                    self.sum += '\n-----------statistics for ' +\
                                self.res[i].description + ' (end point error)\n'
                    err = np.zeros((self.sim_count, 3))
                    for j in range(self.sim_count):
                        err[j, :] = self.res[i].data[j][-1, :] - self.res[ref_name].data[-1, :]
                        # angle error should be within [-pi, pi]
                        if self.interested_error[i] == 'angle':
                            err[j, 0] = attitude.angle_range_pi(err[j, 0])
                            err[j, 1] = attitude.angle_range_pi(err[j, 1])
                            err[j, 2] = attitude.angle_range_pi(err[j, 2])
                    # print(err)
                    scale = 1.0
                    if self.res[i].output_units[0] == 'deg' and self.res[i].units[0] == 'rad':
                        scale = 1.0/D2R
                    self.sum += '--Max error: ' +\
                                str(scale * np.max(np.abs(err), 0)) +\
                                ' ' + self.res[i].output_units[0] + '\n'
                    self.sum += '--Avg error: ' +\
                                str(scale * np.average(err, 0)) +\
                                ' ' + self.res[i].output_units[0] + '\n'
                    self.sum += '--STD of error: ' +\
                                str(scale * np.std(err, 0))  +\
                                ' ' + self.res[i].output_units[0] + '\n'
        print(self.sum)

        #### Allan analysis results ####

        #### save summary to file
        if data_dir is not None:
            try:
                with open(data_dir + '//summary.txt', 'w') as file_summary:
                    file_summary.write(self.sum + '\n')
            except:
                raise IOError('Unable to save summary to %s.'% data_dir)

    def plot(self, what_to_plot, sim_idx=None, opt=None):
        '''
        Plot specified results.
        Args:
            what_to_plot: a string list to specify what to plot. See manual for details.
            sim_idx: specify the simulation index. This can be an integer, or a list or tuple.
                Each element should be within [0, num_times-1]. Default is None, and plot data
                of all simulations.
            opt: a dict to specify plot options. its keys are composed of elements in what_to_plot.
                values can be:
                    'error': plot the error of the data specified by what_to_plot w.r.t ref
                    '3d': 3d plot
        '''
        if self.sim_results is False:
            print("Call Sim.run() and then Sim.results() to run the simulaltion first.")
            return
        # check sim_idx
        if sim_idx is None:                 # no index specified, plot all data
            sim_idx = list(range(self.sim_count))
        elif isinstance(sim_idx, int):      # scalar input, convert to list
            sim_idx = [sim_idx]
        elif isinstance(sim_idx, float):
            sim_idx = [int(sim_idx)]
        invalid_idx = []
        for i in range(0, len(sim_idx)):    # a list specified, remove invalid values
            sim_idx[i] = int(sim_idx[i])
            if sim_idx[i] >= self.sim_count or sim_idx[i] < 0:
                invalid_idx.append(sim_idx[i])
                print('sim_idx[%s] = %s exceeds max simulation count: %s.'%\
                      (i, sim_idx[i], self.sim_count))
        for i in invalid_idx:
            sim_idx.remove(i)

        # data to plot
        for i in what_to_plot:
            # print("data to plot: %s"% i)
            # get plot options
            ref = None
            plot3d = None
            # this data has plot options?
            if isinstance(opt, dict):
                if i in opt:
                    if opt[i].lower() == '3d':
                        plot3d = True
                    elif opt[i].lower() == 'error':
                        ref_name = 'ref_' + i   # this data have reference, error can be calculated
                        if ref_name in self.supported_plot:
                            ref = self.supported_plot[ref_name]
                        else:
                            print(i + ' has no reference.')
            # x axis data
            x_axis = self.time
            if i in self.supported_plot:
                ## choose proper x axis data for specific y axis data
                # plot only once
                if i in self.supported_in_constant:
                    if i == self.ref_gps.name or i == self.gps_time.name:
                        x_axis = self.gps_time
                    # self.supported_plot[i].plot(x_axis, ref=ref, plot3d=plot3d)
                # plot data of all simulation counts
                elif i in self.supported_out:   # algo output
                    if self.algo_time.name in self.res:
                        x_axis = self.algo_time
                    else:
                        if i == self.ad_gyro.name or i == self.ad_accel.name or\
                           i == self.allan_t.name:
                            x_axis = self.allan_t
                        elif i == self.gps.name:
                            x_axis = self.gps_time
                self.supported_plot[i].plot(x_axis, key=sim_idx, ref=ref, plot3d=plot3d)
            else:
                print('Unsupported plot: %s.'% i)
                # print("Only the following data are available for plot:")
                # print(list(self.supported_plot.keys()))
                # raise ValueError("Unsupported data to plot: %s."%data)
        # show figures
        plt.show()

    def parse_motion(self, motion_def):
        '''
        Get initial pos/vel/att and motion command from a .csv file.
        Args:
            motion_def: initial state and motion command file.
        '''
        try:
            ini_state = np.genfromtxt(motion_def, delimiter=',', skip_header=1, max_rows=1)
            waypoints = np.genfromtxt(motion_def, delimiter=',', skip_header=3)
        except:
            raise ValueError('motion definition file must have nine columns \
                              and at least four rows (two header rows + at least two data rows).')
        self.ini_pos_n = ini_state[0:3]
        self.ini_pos_n[0] = self.ini_pos_n[0] * D2R
        self.ini_pos_n[1] = self.ini_pos_n[1] * D2R
        self.ini_vel_b = ini_state[3:6]
        self.ini_att = ini_state[6:9] * D2R
        if waypoints.ndim == 1:
            waypoints = waypoints.reshape((1, len(waypoints)))
        self.motion_def = waypoints[:, [0, 1, 2, 3, 4, 7]]
        self.motion_def[:, 1:4] = self.motion_def[:, 1:4] * D2R

    def parse_mode(self, mode):
        '''
        Parse mode. Not completely implemented yet.
        Args:
            mode: simualtion mode
        '''
        if mode is not None:
            if isinstance(mode, str):               # choose built-in mode
                mode = mode.lower()
                if 'flight' in mode:
                    self.mobility = high_mobility
                elif 'land' in mode:
                    self.mobility = high_mobility
                elif 'ship' in mode:
                    self.mobility = high_mobility
                else:
                    self.mobility = high_mobility
            elif isinstance(mode, np.ndarray):      # customize the sim mode
                if mode.shape == (3,):
                    self.mobility[0] = mode[0]
                    self.mobility[1] = mode[1] * D2R
                    self.mobility[2] = mode[2] * D2R
                else:
                    raise TypeError('mode should be of size (3,)')
            else:
                raise TypeError('mode should be a string or a numpy array of size (3,)')
        else:
            self.mobility = high_mobility

    def parse_env(self, env):
        '''
        Parse env.
        Args:
            env: vibration model
        '''
        if env is None:
            self.vib_def = None
        self.vib_def = {}
        if isinstance(env, str):        # specify simple vib model
            env = env.lower()
            if 'random' in env:         # normal distribution
                self.vib_def['type'] = 'random'
                env = env.replace('-random', '')
            elif 'sinusoidal' in env:   # sinusoidal vibration
                self.vib_def['type'] = 'sinusoidal'
                env = env.replace('-sinusoidal', '')
                if env[-2:] == 'hz':
                    try:
                        idx_first_mark = env.find('-')
                        self.vib_def['freq'] = math.fabs(float(env[idx_first_mark+1:-2]))
                        env = env[:idx_first_mark]
                    except:
                        raise ValueError('env = \'%s\' is not valid (invalid vib freq).'% env)
                else:
                    raise ValueError('env = \'%s\' is not valid (No vib freq).'% env)
            else:
                raise ValueError('env = \'%s\' is not valid.'% env)
            vib_amp = 1.0   # vibration amplitude, 1sigma for random, peak value for sinusoidal
            if env[-1] == 'g' or env[-1] == 'G':
                vib_amp = 9.8
                env = env[:-1]  # remove 'g' or 'G'
            try:
                env = env[1:-1] # remove '[]' or '()'
                env = env.split(' ')
                vib_amp *= np.array(env, dtype='float64')
                self.vib_def['x'] = vib_amp[0]
                self.vib_def['y'] = vib_amp[1]
                self.vib_def['z'] = vib_amp[2]
            except:
                raise ValueError('Cannot convert \'%s\' to float'% env)
        elif isinstance(env, np.ndarray):           # customize the vib model with PSD
            if env.ndim == 2 and env.shape[1] == 4: # env is a np.array of size (n,4)
                self.vib_def['type'] = 'psd'
                n = env.shape[0]
                half_fs = 0.5*self.fs.data
                if env[-1, 0] > half_fs:
                    n = np.where(env[:, 0] > half_fs)[0][0]
                self.vib_def['freq'] = env[:n, 0]
                self.vib_def['x'] = env[:n, 1]
                self.vib_def['y'] = env[:n, 2]
                self.vib_def['z'] = env[:n, 3]
        else:
            raise TypeError('env should be a string or a numpy array of size (n,2)')

    def check_algo(self):
        '''
        Generate expressions to handle algorithm input and output.
        Args:
            algorithm: user specified algorithm class
        Returns:
            Raise ValueError if algorithm has no input or output;
            Raise ValueError if algorithm input and output have unsupported elements
            Raise TypeError if algorithm input or output is not a list or tuple
        '''
        try:
            n_in = len(self.algo.input)
            n_out = len(self.algo.output)
            # algorithm must have at least one input and one output
            if n_in < 1 or n_out < 1:
                raise ValueError
            # prepare algorithm input and output
            for i in self.algo.input:
                if not i in self.supported_in_constant and not i in self.supported_in_varying:
                    print('Unsupported algorithm input: %s'% i)
                    raise ValueError
            for i in self.algo.output:
                if not i in self.supported_out:
                    print('Unsupported algorithm output: %s'% i)
                    raise ValueError
        except ValueError:
            raise ValueError('check input and output definitions of the algorithm.')
        except:
            raise TypeError('algorithm input or output is not a valid list or tuple.')

    def check_data_dir(self, data_dir):
        '''
        check if data_dir is a valid dir. If not, use the default dir.
        check if the data_dir exists. If not, create it.
        Args:
            data_dir: all generated files are saved in data_dir
        Returns:
            data_dir: valid data dir.
        '''
        import os
        import time
        # check data dir
        if data_dir is None:
            data_dir = os.path.abspath('.//demo_saved_data//')
        if not os.path.exists(data_dir):
            data_dir = os.path.abspath('.//demo_saved_data//')
        else:
            data_dir = os.path.abspath(data_dir)
        if data_dir[-1] != '//':
            data_dir = data_dir + '//'
        data_dir = data_dir + time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime()) + '//'
        data_dir = os.path.abspath(data_dir)
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        return data_dir

    def __add_associated_data_to_results(self):
        '''
        Check if some data in self.res have associated data. If so, calculate the associated data
        and add the data in self.res.
        For example, pathgen generates Euler angles, this procedure will calculate the
        coresponding quaternions and add those in self.res.
        '''
        # search for data to add to results and generate associated data
        data_to_add = []
        for i in self.res:
            if i in self.data_map:
                self.data_map[i][1](self.res[i], self.data_map[i][0])
                data_to_add.append(i)
        # add generated associated data to results
        for i in data_to_add:
            self.res[self.data_map[i][0].name] = self.data_map[i][0]

    def __quat2euler_zyx(self, src, dst):
        '''
        quaternion to Euler angles (zyx)
        '''
        if isinstance(src.data, np.ndarray):
            n = src.data.shape[0]
            dst.data = np.zeros((n, 3))
            for j in range(n):
                dst.data[j, :] = attitude.quat2euler(src.data[j, :])
        elif isinstance(src.data, dict):
            for i in src.data:
                n = src.data[i].shape[0]
                euler = np.zeros((n, 3))
                for j in range(n):
                    euler[j, :] = attitude.quat2euler(src.data[i][j, :])
                dst.data[i] = euler
        else:
            raise ValueError('%s is not a dict or numpy array.'% src.name)

    def __euler2quat_zyx(self, src, dst):
        '''
        Euler angles (zyx) to quaternion
        '''
        # array
        if isinstance(src.data, np.ndarray):
            n = src.data.shape[0]
            dst.data = np.zeros((n, 4))
            for j in range(n):
                dst.data[j, :] = attitude.euler2quat(src.data[j, :])
        # dict
        elif isinstance(src.data, dict):
            for i in src.data:
                n = src.data[i].shape[0]
                quat = np.zeros((n, 4))
                for j in range(n):
                    quat[j, :] = attitude.euler2quat(src.data[i][j, :])
                dst.data[i] = quat
        else:
            raise ValueError('%s is not a dict or numpy array.'% src.name)

class Sim_data(object):
    '''
    Simulation data
    '''
    def __init__(self, name, description,\
                 units=None, output_units=None,\
                 plottable=True, logx=False, logy=False,\
                 grid='on', legend=None):
        '''
        Set up data properties and plot properties. All data are stored in a dict: self.data.
        Each key of this dict corresponds to a set of data. self.data[key] is of size mxn.
        m is the number of samples of this set of data. n is the dimension of this set of data.
        m may vary through different set of data. n must be same for all sets of data.
        Args:
            name: string name of the data
            description: string description of the data
            units: a tuple or list of strings to specify units of data.
                The length of units is the same as columns of each set of data in self.data.
            output_units: a tuple or list of strings to specify units of data when we plot or
                save the data to files. Sim_data.plot and Sim_data.save_to_file will automatically
                convert units if necessary.
                If this is set to None, output_units will be the same as units, and no unit
                conversion is needed.
            logx: plot this data with log scaling on x axis
            logy: plot this data with log scaling on y axis
            grid: if this is not 'off', it will be changed to 'on'
            legend: tuple or list of strings to specify legend of data.
                The length of units is the same as columns of each set of data in self.data.
        '''
        self.name = name
        self.description = description
        # units of self.data
        if units is None:
            self.units = []
        else:
            self.units = list(units)
        # output units should have same length as units
        if output_units is None:
            self.output_units = self.units
        else:
            self.output_units = list(output_units)
            len_in = len(self.units)
            len_out = len(self.output_units)
            if len_in > len_out:
                for i in range(len_out, len_in):
                    self.output_units.append(self.units[i])
            elif len_in < len_out:
                for i in range(len_in, len_out):
                    self.units.append(self.output_units[i])
        self.plottable = plottable
        self.logx = logx
        self.logy = logy
        self.grid = 'on'
        if grid.lower() == 'off':
            self.grid = grid
        self.legend = legend
        # a dict to store data, each key corresponds to a set of data
        # or a numpy array of size(m,n)
        # or a scalar
        self.data = {}

    def plot(self, x, key=None, ref=None, plot3d=False):
        '''
        Plot self.data[key]
        Args:
            key is a tuple or list of keys
            x: x axis data
        '''
        if self.plottable:
            if isinstance(self.data, dict):
                self.plot_dict(x, key, ref, plot3d)
            else:
                self.plot_array(x, ref, plot3d)

    def plot_dict(self, x, key, ref=None, plot3d=False):
        '''
        self.data is a dict. plot self.data according to key
        '''
        for i in key:
            y_data = self.data[i]
            # x axis
            if isinstance(x.data, dict):
                x_data = x.data[i]
            else:
                x_data = x.data
            # error
            if ref is not None:
                if isinstance(ref.data, dict):
                    ref_data = ref.data[i]
                else:
                    ref_data = ref.data
                try:
                    y_data = y_data - ref_data
                    if self.units == ['rad', 'rad', 'rad']:
                        y_data = y_data % attitude.TWO_PI
                        idx = y_data > math.pi
                        y_data[idx] = y_data[idx] - attitude.TWO_PI
                except:
                    print('ref data shape: ', ref_data.shape)
                    print('simulation data shape: ', y_data.shape)
                    raise ValueError('Check input data ref and self.data dimension.')
            # unit conversion
            y_data = self.convert_unit(y_data)
            # plot
            if plot3d:
                plot3d_in_one_figure(y_data,\
                                     title=self.name + '_' + str(i),\
                                     grid=self.grid,\
                                     legend=self.legend)
            else:
                plot_in_one_figure(x_data, y_data,\
                                   logx=self.logx, logy=self.logy,\
                                   title=self.name + '_' + str(i),\
                                   xlabel=x.name + ' (' + x.output_units[0] + ')',\
                                   ylabel=self.name + ' (' + str(self.output_units) + ')',\
                                   grid=self.grid,\
                                   legend=self.legend)

    def plot_array(self, x, ref=None, plot3d=False):
        '''
        self.data is a numpy.array
        '''
        # x axis
        if isinstance(x.data, dict):
            x_data = x.data[0]
        else:
            x_data = x.data
        # error
        y_data = self.data
        if ref is not None:
            try:
                y_data = self.data - ref
                if self.units == ['rad', 'rad', 'rad']:
                        y_data = y_data % attitude.TWO_PI
                        idx = y_data > math.pi
                        y_data[idx] = y_data[idx] - attitude.TWO_PI
            except:
                print(ref.shape)
                print(self.data.shape)
                raise ValueError('Check input data ref and self.data dimension.')
        # unit conversion
        y_data = self.convert_unit(y_data)
        # plot
        if plot3d:
            plot3d_in_one_figure(y_data,\
                                 title=self.name,\
                                 grid=self.grid,\
                                 legend=self.legend)
        else:
            plot_in_one_figure(x_data, y_data,\
                               logx=self.logx, logy=self.logy,\
                               xlabel=x.name + ' (' + x.output_units[0] + ')',\
                               ylabel=self.name + ' (' + str(self.output_units) + ')',\
                               title=self.name,\
                               grid=self.grid,\
                               legend=self.legend)

    def save_to_file(self, data_dir):
        '''
        Save self.data to files.
        Args:
            data_dir: directory for the data files.
        '''
        #### generate header
        # how many columns in each set of data? 0 if scalar
        cols = 0
        if isinstance(self.data, dict):
            for i in self.data.keys():
                if self.data[i].ndim > 1:
                    cols = self.data[i].shape[1]
                break   # each set of data in data should have the same number of columns
        else:
            if self.data.ndim > 1:
                cols = self.data.shape[1]
        # add the name and unit of each column to header
        header_line = ''
        if cols > 0:    # more than one column
            for i in range(cols):
                # units
                str_unit = ''
                if i < len(self.output_units):
                    str_unit = ' (' + self.output_units[i] + ')'
                # add a column
                if cols == len(self.legend):    # legend available
                    header_line += self.legend[i] + str_unit + ','
                else:                           # legend not available
                    header_line += self.name + '_' + str(i) + str_unit + ','
            # remove the trailing ','
            header_line = header_line[0:-1]
        else:           # only one column
            str_unit = ''
            if len(self.output_units) > 0:
                str_unit = ' (' + self.output_units[0] + ')'
            header_line = self.name + str_unit
        #### save data and header to .csv files
        if isinstance(self.data, dict):
            for i in self.data:
                file_name = data_dir + '//' + self.name + '_' + str(i) + '.csv'
                np.savetxt(file_name, self.convert_unit(self.data[i]),\
                           header=header_line, delimiter=',', comments='')
        else:
            file_name = data_dir + '//' + self.name + '.csv'
            np.savetxt(file_name, self.convert_unit(self.data),\
                       header=header_line, delimiter=',', comments='')

    def convert_unit(self, data):
        '''
        Unit conversion.
        Args:
            data: convert data units from units to output_units,\
                data should be a numpy array of size(n,) or (n,m).
                n is data length, m is data dimension.
        Returns:
            data: data after unit conversion.
        '''
        # check if unit conversion is needed and calculate the scale
        m = len(self.output_units)
        scale = self.unit_conversion_scale()
        # unit conversion
        x = data.copy()
        if x.ndim == 2:
            for i in range(min(m, x.shape[1])):
                if scale[i] != 0.0:
                    x[:, i] = x[:, i] * scale[i]
        elif x.ndim == 1:
            if scale[0] != 0.0:
                x = x * scale[0]
        else:
            raise ValueError('data should a 1D or 2D array, ndim = %s'% data.ndim)
        return x

    def unit_conversion_scale(self):
        '''
        Calculate unit conversion scale.
        '''
        m = len(self.output_units)
        scale = np.zeros((m,))
        for i in range(m):
            # deg to rad
            if self.units[i] == 'deg' and self.output_units[i] == 'rad':
                scale[i] = D2R
            elif self.units[i] == 'deg/s' and self.output_units[i] == 'rad/s':
                scale[i] = D2R
            # rad to deg
            elif self.units[i] == 'rad' and self.output_units[i] == 'deg':
                scale[i] = 1.0/D2R
            elif self.units[i] == 'rad/s' and self.output_units[i] == 'deg/s':
                scale[i] = 1.0/D2R
        return scale

def plot_in_one_figure(x, y, logx=False, logy=False,\
                       title='Figure', xlabel=None, ylabel=None,\
                       grid='on', legend=None):
    '''
    Create a figure and plot x/y in this figure.
    Args:
        x: x axis data, np.array of size (n,) or (n,1)
        y: y axis data, np.array of size (n,m)
        title: figure title
        xlabel: x axis label
        ylabel: y axis label
        gird: if this is not 'off', it will be changed to 'on'
        legend: tuple or list of strings of length m.
    '''
    # create figure and axis
    fig = plt.figure(title)
    axis = fig.add_subplot(111)
    lines = []
    try:
        dim = y.ndim
        if dim == 1:
            if logx and logy:   # loglog
                line, = axis.loglog(x, y)
            elif logx:          # semilogx
                line, = axis.semilogx(x, y)
            elif logy:          # semilogy
                line, = axis.semilogy(x, y)
            else:               # plot
                line, = axis.plot(x, y)
            lines.append(line)
        elif dim == 2:
            for i in range(0, y.shape[1]):
                if logx and logy:   # loglog
                    line, = axis.loglog(x, y[:, i])
                elif logx:          # semilogx
                    line, = axis.semilogx(x, y[:, i])
                elif logy:          # semilogy
                    line, = axis.semilogy(x, y[:, i])
                else:               # plot
                    line, = axis.plot(x, y[:, i])
                lines.append(line)
        else:
            raise ValueError
    except:
        print('x-axis data len: ', x.shape)
        print('y-axis data shape: ', y.shape)
        raise ValueError('Check input data y.')
    # label
    if xlabel is not None:
        plt.xlabel(xlabel)
    if ylabel is not None:
        plt.ylabel(ylabel)
    # legend
    if legend is not None:
        plt.legend(lines, legend)
    # grid
    if grid.lower() != 'off':
        plt.grid()

def plot3d_in_one_figure(y, title='Figure', grid='on', legend=None):
    '''
    Create a figure and plot 3d trajectory in this figure.
    Args:
        y: y axis data, np.array of size (n,3)
        title: figure title
        gird: if this is not 'off', it will be changed to 'on'
        legend: tuple or list of strings of length 3.
    '''
    # create figure and axis
    fig = plt.figure(title)
    axis = fig.add_subplot(111, projection='3d', aspect='equal')
    try:
        dim = y.ndim
        if dim == 2:    # y must be an numpy array of size (n,3), dim=2
            if y.shape[1] != 3:
                raise ValueError
            else:
                axis.plot(y[:, 0], y[:, 1], y[:, 2])
        else:
            raise ValueError
    except:
        print(y.shape)
        raise ValueError('Check input data y.')
    # label
    if isinstance(legend, (tuple, list)):
        n = len(legend)
        if n != 3:
            legend = ['x', 'y', 'z']
    else:
        legend = ['x', 'y', 'z']
    axis.set_xlabel(legend[0])
    axis.set_ylabel(legend[1])
    axis.set_zlabel(legend[2])
    # grid
    if grid.lower() != 'off':
        plt.grid()
