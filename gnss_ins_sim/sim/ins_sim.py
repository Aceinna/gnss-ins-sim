# -*- coding: utf-8 -*-
# Filename: ins_sim.py

"""
INS simulation engine.
Created on 2018-04-24
@author: dongxiaoguang
"""

import os
import time
import math
import numpy as np
from .ins_data_manager import InsDataMgr
from .ins_algo_manager import InsAlgoMgr
from ..pathgen import pathgen
from ..attitude import attitude
from ..geoparams import geoparams

# version info
NAME = 'gnss-ins-sim'
VERSION = '3.0.0_alpha'

# built-in mobility
high_mobility = np.array([1.0, 0.5, 2.0])   # m/s/s, rad/s/s, rad/s

class Sim(object):
    '''
    INS simulation engine.
    '''
    def __init__(self, fs, motion_def, ref_frame=0, imu=None,\
                 mode=None, env=None, algorithm=None):
        '''
        Args:
            fs: [fs_imu, fs_gps, fs_mag], Hz.
                fs_imu: The sample rate of IMU. This is also the sampel rate of the simulatino.
                fs_gps: The sample rate of GPS.
                fs_mag: not used now. The sample rate of the magnetometer is
                    the same as that of the imu.

            motion_def: If you want to do simulation with logged data files, motion_def should be
                a directory contains the data files. Data files should be named as data_name.csv.
                Supported data names are algorithm input. (Refer to readme.md)
                If you do not have logged data files and want to generate sensor data from a motion
                definition file,  motion_def should be a csv file to define the waypoints.
                The .csv file should be organized as follows:
                row 1: header line for initial states
                row 2: initial states, which include:
                    col 1-3: initial position (LLA, deg, meter),
                    col 4-6: initial velocity in body frame(m/s),
                    col 7-9: initial attitude (Euler angles that rotate the reference frame to the
                        body frame according to the ZYX rotation sequence, deg).
                row 3: header line for motion command
                row >=4: motion commands, which include
                    col 1: motion type. The following types are supported:
                        1: Euler angles change rate and body frame velocity change rate.
                        2: absolute att and absolute vel to reach.
                        3: relative att and vel change.
                        4: absolute att, relative vel.
                        5: relative att, absolute vel.
                    col 2-7: motion command (deg, m/s).
                        [yaw, pitch, roll, vx (velocity along body x axis), vy, vz].
                        For motion type 1, the above Euler angles and velocity should be
                        change rate, corresponding units are (deg/s, m/s/s).
                    col 8: maximum time for the given segment, sec. Max time together with the
                        param "mode" determines if this command can be executed successfully.
                        If actual executing time is less than max time, the remaining time will
                        not be used and the next command will be executed immediately. If the
                        command cannot be finished within max time, the next command will be
                        executed after max time. If you want to fully control execution time by
                        your own, you should always choose motion type to be 1.
                    col 9: gps visibility, should be 1 or 0.
                motion_def can also be a string that contains the same contents as the csv file
                mentioned above.

            ref_frame: reference frame used as the navigation frame and the attitude reference.
                        0: NED (default), with x axis pointing along geographic north,
                           y axis pointing eastward,
                           z axis pointing downward.
                           Position will be expressed in LLA form, and the reference velocity of
                           the vehicle relative to the ECEF frame will be expressed in the NED
                           frame, and GPS velocity will be expressed in the NED frame.
                        1: a virtual inertial frame with constant g,
                           x axis pointing along geographic/magnetic north,
                           z axis pointing along g,
                           y axis completing a right-handed coordinate system.
                           Position and velocity will both be in the [x y z] form in this frame.
                           **Notice: For this virtual inertial frame, position is indeed the sum of
                           the initial position in ecef and the relative position in the virutal
                           inertial frame. Indeed, two vectors expressed in different frames should
                           not be added. This is done in this way here just to preserve all useful
                           information to generate .kml files.
                           Keep this in mind if you use this result.

            imu: Define the IMU error model. See IMU in imu_model.py.
                If you want to do simulation with logged data files, set imu=None.
                If you do not have logged data files and want to generate sensor data from a motion
                definition file, you should specify the IMU model.

            mode: simulation mode could be a string to specify a built-in mode:
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

            algorithm: a user defined algorithm or list of algorithms. If there are multiple
                algorithms, all algorithms should have the same input and output.
        '''
        # version info of gnss-ins-sim
        self.name = NAME
        self.version = VERSION
        # simulation input
        self.fs = fs
        self.imu = imu
        self.mode = mode
        self.env = env
        if ref_frame == 0 or ref_frame == 1:
            self.ref_frame = ref_frame
        else:
            self.ref_frame = 0      # default frame is NED
        # simulation status
        self.sim_count = 1          # simulation count
        self.sim_complete = False   # simulation complete successfully
        self.sim_results = False    # simulation results is generated
        # simulation data manager
        self.dmgr = InsDataMgr(fs, self.ref_frame)
        self.data_src = motion_def
        self.data_from_files = False
        # algorithm manager
        self.amgr = InsAlgoMgr(algorithm)

        # associated data mapping. this is a dict in the following form:
        #   {'dst_name': ['src_name', routine_convert_src_to_dst]}
        self.data_map = {\
            self.dmgr.ref_att_euler.name: [self.dmgr.ref_att_quat.name, self.__quat2euler_zyx],
            self.dmgr.ref_att_quat.name: [self.dmgr.ref_att_euler.name, self.__euler2quat_zyx],
            self.dmgr.att_euler.name: [self.dmgr.att_quat.name, self.__quat2euler_zyx],
            self.dmgr.att_quat.name: [self.dmgr.att_euler.name, self.__euler2quat_zyx]}

        # error terms we are interested in
        self.interested_error = {self.dmgr.att_euler.name: 'angle',
                                 self.dmgr.pos.name: None,
                                 self.dmgr.vel.name: None}

        # summary
        self.sum = ''

    def run(self, num_times=1):
        '''
        run simulation.
        Args:
            num_times: run the simulation for num_times times with given IMU error model.
        '''
        self.sim_count = int(num_times)
        if self.sim_count < 1:
            self.sim_count = 1

        #### generate sensor data from file or pathgen
        self.__gen_data()

        #### run algorithms
        if self.amgr.algo is not None:
            # tell data manager the output of the algorithm
            self.dmgr.set_algo_output(self.amgr.output)
            # get algo input data
            algo_input = self.dmgr.get_data(self.amgr.input)
            # run the algo and get algo output
            algo_output = self.amgr.run_algo(algo_input, range(self.sim_count))
            # add algo output to ins_data_manager
            for i in range(len(self.amgr.output)):
                self.dmgr.add_data(self.amgr.output[i], algo_output[i])
        # simulation complete successfully
        self.sim_complete = True

        #### generate associated data
        self.__add_associated_data_to_results()

    def results(self, data_dir=None, err_stats_start=0, gen_kml=False, extra_opt=''):
        '''
        Simulation results.
        Save results to .csv files containing all data generated.
        Simulation results also include a summary file containing statistics of the simulation.
        Args:
            data_dir: if not None, save simulation data to files.
                If data_dir is a valid directory, data files will be saved in data_idr.
                Otherwise, data files will be saved in the default directory './demo_saved_data/'.
                The file name is in the form of data_name-key.csv. data_name is the name of the
                data as defined in ins_data_manager. key is determined by the algorithm name and
                simulation run.
                E.g., if have two algorithms in one simulation [algo_with_name, algo_no_name]
                and the simulation is ran for 10 times. algo_with_name has a name field 'tilt'
                and algo_no_name does not. Both algorithms output Euler angles. Then the results
                of the 0-th simulation run are saved to files named att_euler-tilt_0.csv and
                att_euler-algo1.csv. That is, if the algorithm has name, the name is used in the
                key. If the algorithm has no name, a name "algo"+order of the algorithm in the
                algorithm list is used as the algorithm name in the key.
            err_stats_start: When calculating the error statistics, this argument specify the
                starting point in seconds from where the error statistics are calculated. If it
                is -1, end-point error statistics will be calculated. Any other negative value
                will be the same as 0. If err_stats_start exceeds the max number of data points,
                it will be converted to 0.
            gen_kml: True to generate two .kml files containing the reference position and the
                    simulation position (output by algorithms), respectively.
            extra_opt: Extra options to generate the results. It can be a string option to
                calculate errors. The following options are supported:
                    'ned': NED position error.
        Returns: a dict contains all simulation results.
        '''
        if self.sim_complete:
            #### check data dir
            data_saved = []
            if data_dir is not None:    # data_dir specified, meaning to save .csv files
                data_dir = self.__check_data_dir(data_dir)
                # save data files
                data_saved = self.dmgr.save_data(data_dir)

            #### generate .kml files
            if gen_kml is True:       # want to gen kml without specifying the data_dir
                if data_dir is None:
                    data_dir = ''
                    data_dir = self.__check_data_dir(data_dir)
                self.dmgr.save_kml_files(data_dir)

            #### simulation summary and save summary to file
            self.__summary(data_dir, data_saved,
                           err_stats_start=err_stats_start, extra_opt=extra_opt)

            #### simulation results are generated
            self.sim_results = True

            #### available data
            return self.dmgr.available
        else:
            print("Call Sim.run() to run the simulaltion first.")
            return None

    def plot(self, what_to_plot, sim_idx=None, opt=None, extra_opt=''):
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
                    'projection': project the 3D plot on xy, xz and yz plane, respectively.
            extra_opt: only strings to specify matplotlib properties is supported.
        '''
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
        # generate keys to index simulation data
        for data in what_to_plot:
            # generate keys for this var, only algo output has algo name in keys
            data_from_algo = self.__data_from_algo_output(data)
            if any(data_from_algo):
                keys = []
                for i in range(self.amgr.nalgo):
                    # results from different algorithms are indexed by algo_name and simulation runs
                    # this result is not generated by this algo and this result is not associated
                    #   with any output of this algo
                    if data_from_algo[i]:
                        # this is output is generated by this algo
                        algo_name = self.amgr.get_algo_name(i)
                        for j in range(len(sim_idx)):
                            keys.append(algo_name+'_'+str(j))
                    else:
                        continue
            else:
                keys = sim_idx
            # plot data
            is_angle = False
            if data in self.interested_error:
                is_angle = self.interested_error[data] == 'angle'
            self.dmgr.plot(data, keys, is_angle, opt, extra_opt)
        # show figures
        self.dmgr.show_plot()

    def get_names_of_available_data(self):
        '''
        Get a list of the names of available data in the simulation
        '''
        return self.dmgr.available

    def get_data(self, data_names):
        '''
        Get data section of data_names.
        Args:
            data_names: a list of data names
        Returns:
            data: a list of data corresponding to data_names. Each element in the list
                can be a scalar, an array or a dict.
                If there is any unavailable data in data_names, return None.
        '''
        return self.dmgr.get_data(data_names).copy()

    def get_data_properties(self, data_name):
        '''
        Get the properties of the data specified by data_name.
        Args:
            data_name: a string to specify the data
        Returns:
            [description, units, plottable, logx, logy, legend]
        '''
        return self.dmgr.get_data_properties(data_name)

    def __summary(self, data_dir, data_saved, err_stats_start=0, extra_opt=''):
        '''
        Summary of sim results.
        '''
        #### simulation config
        self.sum += '\n------------------------------------------------------------\n'
        # sample frequency
        self.sum += self.dmgr.fs.description + ': [' +\
                    self.dmgr.fs.name + '] = ' +\
                    str(self.dmgr.fs.data) + ' ' +\
                    self.dmgr.fs.units[0] + '\n'
        # reference frame
        self.sum += self.dmgr.ref_frame.description + ': ' + str(self.dmgr.ref_frame.data) + '\n'
        # simulation time duration
        self.sum += 'Simulation time duration: ' + \
                    str(len(self.dmgr.time.data)/self.dmgr.fs.data) + ' s' + '\n'
        # simulation times
        self.sum += 'Simulation runs: ' + str(self.sim_count) + '\n'

        #### save data
        if data_dir is not None:
            self.sum += '\n------------------------------------------------------------\n'
            self.sum += 'Simulation results are saved to ' + data_dir + '\n'
            self.sum += 'The following results are saved:\n'
            for i in data_saved:
                self.sum += '\t' + i  + ': ' + self.dmgr.get_data_all(i).description + '\n'

        #### error statistics of algorithm output
        err_stats_header_line = False
        for data_name in self.interested_error:
            if data_name not in self.dmgr.available:
                continue
            is_angle = self.interested_error[data_name] == 'angle'
            err_stats = self.dmgr.get_error_stats(data_name, err_stats_start=err_stats_start,\
                                                angle=is_angle, use_output_units=True,\
                                                extra_opt=extra_opt)
            if err_stats_header_line is not None:
                # There is error stats, add a headerline
                if err_stats_header_line is False:
                    err_stats_header_line = True
                    self.sum += '\n------------------------------------------------------------\n'
                    self.sum += 'The following are error statistics.'
                # Units of the error stats
                err_units = err_stats['units']
                self.sum += '\n-----------statistics for ' +\
                            self.dmgr.get_data_all(data_name).description +\
                            ' (in units of ' +\
                            err_units +')\n'
                if isinstance(err_stats['max'], dict):
                    for sim_run in sorted(err_stats['max'].keys()):
                        self.sum += '\tSimulation run ' + str(sim_run) + ':\n'
                        self.sum += '\t\t--Max error: ' + str(err_stats['max'][sim_run]) + '\n'
                        self.sum += '\t\t--Avg error: ' + str(err_stats['avg'][sim_run]) + '\n'
                        self.sum += '\t\t--Std of error: ' + str(err_stats['std'][sim_run]) + '\n'
                else:
                    self.sum += '\t--Max error: ' + str(err_stats['max']) + '\n'
                    self.sum += '\t--Avg error: ' + str(err_stats['avg']) + '\n'
                    self.sum += '\t--Std of error: ' + str(err_stats['std']) + '\n'

        print(self.sum)

        #### Allan analysis results ####
        '''
        to be added
        This is not added because the Allan dev curve may not be normal sometimes.
        Fitting this curve will result in wrong parameters.
        '''

        #### save summary to file
        if data_dir is not None:
            try:
                with open(data_dir + '//summary.txt', 'w') as file_summary:
                    file_summary.write(self.sum + '\n')
            except:
                raise IOError('Unable to save summary to %s.'% data_dir)

    def __gen_data(self):
        '''
        Generate data
        '''
        if os.path.isdir(self.data_src):    # gen data from files in a directory
            self.data_src = os.path.abspath(self.data_src)
            self.__gen_data_from_files()
            self.data_from_files = True
        else: # gen data from motion definitions
            self.__gen_data_from_pathgen()

    def __gen_data_from_files(self):
        '''
        Generate data from files
        '''
        for i in os.listdir(self.data_src):
            data_name, data_key = self.__get_data_name_and_key(i)
            if self.dmgr.is_supported(data_name):
                full_file_name = self.data_src + '//' + i
                # read data in file
                data = np.genfromtxt(full_file_name, delimiter=',', skip_header=1)
                # get data units in file
                units = self.__get_data_units(full_file_name)
                # see if position info mathes reference frame
                if data_name == self.dmgr.ref_pos.name or data_name == self.dmgr.pos.name:
                    data, units = self.__convert_pos(data, units, self.dmgr.ref_frame.data)
                # print([data_name, data_key, units])
                self.dmgr.add_data(data_name, data, data_key, units)

    def __gen_data_from_pathgen(self):
        '''
        Generate data from pathgen.
        '''
        # read motion definition
        [ini_pva, motion_def] = self.__parse_motion()
        # output definitions
        output_def = np.array([[1.0, self.fs[0]], [1.0, self.fs[0]], [1.0, self.fs[0]]])
        if self.imu.gps:
            output_def[1, 0] = 1.0
            output_def[1, 1] = self.fs[1]
        else:
            output_def[1, 0] = -1.0
        if self.imu.odo:
            output_def[2, 0] = 1.0
        else:
            output_def[2, 0] = -1.0
        # sim mode-->vehicle maneuver capability
        mobility = self.__parse_mode(self.mode)

        # generate reference data and add data to ins_data_manager
        rtn = pathgen.path_gen(ini_pva, motion_def, output_def, mobility,
                               self.ref_frame, self.imu.magnetometer)
        self.dmgr.add_data(self.dmgr.time.name, rtn['nav'][:, 0] / self.fs[0])
        self.dmgr.add_data(self.dmgr.ref_pos.name, rtn['nav'][:, 1:4])
        self.dmgr.add_data(self.dmgr.ref_vel.name, rtn['nav'][:, 4:7])
        self.dmgr.add_data(self.dmgr.ref_att_euler.name, rtn['nav'][:, 7:10])
        self.dmgr.add_data(self.dmgr.ref_accel.name, rtn['imu'][:, 1:4])
        self.dmgr.add_data(self.dmgr.ref_gyro.name, rtn['imu'][:, 4:7])
        if self.imu.gps:
            self.dmgr.add_data(self.dmgr.gps_time.name, rtn['gps'][:, 0] / self.fs[0])
            self.dmgr.add_data(self.dmgr.ref_gps.name, rtn['gps'][:, 1:7])
            self.dmgr.add_data(self.dmgr.gps_visibility.name, rtn['gps'][:, 7])
        if self.imu.magnetometer:
            self.dmgr.add_data(self.dmgr.ref_mag.name, rtn['mag'][:, 1:4])
        if self.imu.odo:
            self.dmgr.add_data(self.dmgr.ref_odo.name, rtn['odo'][:, 2])
        # generate sensor data
        # environment-->vibraition params
        vib_def = self.__parse_env(self.env)
        for i in range(self.sim_count):
            accel = pathgen.acc_gen(self.fs[0], self.dmgr.ref_accel.data,
                                    self.imu.accel_err, vib_def)
            self.dmgr.add_data(self.dmgr.accel.name, accel, key=i)
            gyro = pathgen.gyro_gen(self.fs[0], self.dmgr.ref_gyro.data,\
                                    self.imu.gyro_err)
            self.dmgr.add_data(self.dmgr.gyro.name, gyro, key=i)
            if self.imu.gps:
                gps = pathgen.gps_gen(self.dmgr.ref_gps.data, self.imu.gps_err,\
                                                   self.ref_frame)
                self.dmgr.add_data(self.dmgr.gps.name, gps, key=i)
            if self.imu.magnetometer:
                mag = pathgen.mag_gen(self.dmgr.ref_mag.data, self.imu.mag_err)
                self.dmgr.add_data(self.dmgr.mag.name, mag, key=i)
            if self.imu.odo:
                odo = pathgen.odo_gen(self.dmgr.ref_odo.data, self.imu.odo_err)
                self.dmgr.add_data(self.dmgr.odo.name, odo, key=i)

    def __get_data_name_and_key(self, file_name):
        '''
        Get data name and data key from the file_name.
        Args:
            file_name: a string formatted as name_key. For example, accel-0.csv means the file
                contains 0-th set of accelerometer measurements. The data_name is accel, and the
                data key is 0. ref_accel.csv means teh file contains reference acceleratons (no
                error). The data_name is ref_accel, and data_key is None.
        Returns:
            data_name: name of data in this file. If the file is not a .csv file, data_name is
                None.
            data_key: key of data in this file. If the data has no key (e.g. ref_accel.csv),
                data_key is None.
        '''
        data_name = None
        data_key = None
        file_name = file_name.lower()
        if file_name[-4::] == '.csv':
            data_name = file_name[0:-4]
            # file name contains a key
            i = data_name.rfind('-')
            if i != -1:
                data_key = data_name[i+1::]
                data_name = data_name[0:i]
                if data_key.isdigit():
                    data_key = int(data_key)
        return data_name, data_key

    def __get_data_units(self, file_name):
        '''
        Get data units in file. Units information should be provided in the first row.
        Args:
            file_name: full file name
        Returns:
            units: a list of units corresponding to each column in the file. If not untis
                found, units is None.
        '''
        units = None
        fp = open(file_name)
        line = fp.readline()
        fp.close()
        line = line.split(',')
        tmp_units = []
        for i in line:
            left_bracket = i.find('(')
            right_bracket = i.rfind(')')
            if left_bracket != -1 and right_bracket != -1 and right_bracket > left_bracket:
                tmp_units.append(i[left_bracket+1:right_bracket])
        if len(tmp_units) == len(line):
            units = tmp_units
        return units

    def __data_from_algo_output(self, data_name):
        '''
        Check if data corresponding to data_name are from algo output or associated
        data of algo output. For example, if an algo outputs quaternions, euler angles
        can be calculated from quaternions accordig to self.data_map.
        Args:
            data_name: a string data name.
        Returns: A list of True or False. Its length is equal to the number of algos.
            True of false representing if this data of data_name are or can be 
            calculated from the output of this algo.
        '''
        rtn = []
        for i in range(self.amgr.nalgo):
            algo_output = self.amgr.algo[i].output
            rtn.append(data_name in algo_output or\
                       data_name in self.data_map and self.data_map[data_name][0] in algo_output)
        return rtn

    def __parse_motion(self):
        '''
        Get initial pos/vel/att and motion command from a .csv file specified by self.data_src.
        Returns: a list containing:
            inis_pos_n: initial position, [Lat Lon Alt], units: rad, rad, m.
            ini_vel_b: initial velocity in the body frame, units: m/s.
            ini_att: initial Euler angles, ZYX rotation sequence, units: rad
            motion_def: motion commands, units: rad, rad/s, m, m/s.
        '''
        try:
            # If self.data_src is a string of motion definitions, convert it to a list of strings
            # so genfromtxt can read it.
            if not os.path.isfile(self.data_src):
                self.data_src = list(self.data_src.split('\n'))
            ini_state = np.genfromtxt(self.data_src, delimiter=',', skip_header=1, max_rows=1)
            waypoints = np.genfromtxt(self.data_src, delimiter=',', skip_header=3)
        except:
            raise ValueError('motion definition file/string must have nine columns \
                              and at least four rows (two header rows + at least two data rows).')
        ini_pos_n = ini_state[0:3]
        ini_pos_n[0] = ini_pos_n[0] * attitude.D2R
        ini_pos_n[1] = ini_pos_n[1] * attitude.D2R
        ini_vel_b = ini_state[3:6]
        ini_att = ini_state[6:9] * attitude.D2R
        if waypoints.ndim == 1: # if waypoints is of size (n,), change it to (1,n)
            waypoints = waypoints.reshape((1, len(waypoints)))
        motion_def = waypoints[:, [0, 1, 2, 3, 4, 5, 6, 7, 8]]
        # convert deg to rad
        motion_def[:, 1:4] = motion_def[:, 1:4] * attitude.D2R
        # replace nan with 0.0, doing this to be compatible with older version motion def files.
        motion_def[np.isnan(motion_def)] = 0.0

        return [np.hstack((ini_pos_n, ini_vel_b, ini_att)), motion_def]

    def __parse_mode(self, mode):
        '''
        Parse mode. Not completely implemented yet.
        Args:
            mode: simualtion mode
        '''
        if mode is not None:
            if isinstance(mode, str):               # choose built-in mode
                mode = mode.lower()
                if 'flight' in mode:
                    mobility = high_mobility
                elif 'land' in mode:
                    mobility = high_mobility
                elif 'ship' in mode:
                    mobility = high_mobility
                else:
                    mobility = high_mobility
            elif isinstance(mode, np.ndarray):      # customize the sim mode
                if mode.shape == (3,):
                    mobility = mode.copy()
                    mobility[1] = mobility[1] * attitude.D2R
                    mobility[2] = mobility[2] * attitude.D2R
                else:
                    raise TypeError('mode should be of size (3,)')
            else:
                raise TypeError('mode should be a string or a numpy array of size (3,)')
        else:
            mobility = high_mobility
        return mobility

    def __parse_env(self, env):
        '''
        Parse env.
        Args:
            env: vibration model
        '''
        vib_def = {}
        if env is None:
            vib_def = None
        elif isinstance(env, str):        # specify simple vib model
            env = env.lower()
            if 'random' in env:         # normal distribution
                vib_def['type'] = 'random'
                env = env.replace('-random', '')
            elif 'sinusoidal' in env:   # sinusoidal vibration
                vib_def['type'] = 'sinusoidal'
                env = env.replace('-sinusoidal', '')
                if env[-2:] == 'hz':
                    try:
                        idx_first_mark = env.find('-')
                        vib_def['freq'] = math.fabs(float(env[idx_first_mark+1:-2]))
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
                vib_def['x'] = vib_amp[0]
                vib_def['y'] = vib_amp[1]
                vib_def['z'] = vib_amp[2]
            except:
                raise ValueError('Cannot convert \'%s\' to float'% env)
        elif isinstance(env, np.ndarray):           # customize the vib model with PSD
            if env.ndim == 2 and env.shape[1] == 4: # env is a np.array of size (n,4)
                vib_def['type'] = 'psd'
                n = env.shape[0]
                half_fs = 0.5*self.fs.data
                if env[-1, 0] > half_fs:
                    n = np.where(env[:, 0] > half_fs)[0][0]
                vib_def['freq'] = env[:n, 0]
                vib_def['x'] = env[:n, 1]
                vib_def['y'] = env[:n, 2]
                vib_def['z'] = env[:n, 3]
            else:
                raise TypeError('env should be of size (n,2)')
        else:
            raise TypeError('env should be a string or a numpy array of size (n,2)')
        return vib_def

    def __check_data_dir(self, data_dir):
        '''
        check if data_dir is a valid dir. If not, use the default dir.
        check if the data_dir exists. If not, create it.
        Args:
            data_dir: all generated files are saved in data_dir
        Returns:
            data_dir: valid data dir.
        '''
        # check data dir
        # data_dir is not specified, automatically create one
        if data_dir == '':
            data_dir = os.path.abspath('.//demo_saved_data//')
            if data_dir[-1] != '//':
                data_dir = data_dir + '//'
            data_dir = data_dir + time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime()) + '//'
            data_dir = os.path.abspath(data_dir)
        # create data dir
        if not os.path.exists(data_dir):
            try:
                data_dir = os.path.abspath(data_dir)
                os.makedirs(data_dir)
            except:
                raise IOError('Cannot create dir: %s.'% data_dir)
        return data_dir

    def __add_associated_data_to_results(self):
        '''
        Check if some data in self.res have associated data. If so, calculate the associated data
        and add the data in self.res.
        For example, pathgen generates Euler angles, this procedure will calculate the
        coresponding quaternions and add those in self.res.
        '''
        for i in self.data_map:
            # data available and its associated data are supported
            src_name = self.data_map[i][0]
            if src_name in self.dmgr.available and self.dmgr.is_supported(i):
                src_data = self.dmgr.get_data([src_name])[0]
                # src_data is a dict, add associated data of all keys
                if isinstance(src_data, dict):
                    for key in src_data:
                        if not self.dmgr.is_available(i, key):
                            self.dmgr.add_data(i, self.data_map[i][1](src_data[key]), key)
                else:
                    if not self.dmgr.is_available(i):
                        self.dmgr.add_data(i, self.data_map[i][1](src_data))

    def __quat2euler_zyx(self, src):
        '''
        quaternion to Euler angles (zyx)
        '''
        if isinstance(src, np.ndarray):
            n = src.shape[0]
            dst = np.zeros((n, 3))
            for j in range(n):
                dst[j, :] = attitude.quat2euler(src[j, :])
            return dst
        elif isinstance(src, dict):
            dst = {}
            for i in src:
                n = src[i].shape[0]
                euler = np.zeros((n, 3))
                for j in range(n):
                    euler[j, :] = attitude.quat2euler(src[i][j, :])
                dst[i] = euler
            return dst
        else:
            raise ValueError('%s is not a dict or numpy array.'% src.name)

    def __euler2quat_zyx(self, src):
        '''
        Euler angles (zyx) to quaternion
        '''
        # array
        if isinstance(src, np.ndarray):
            n = src.shape[0]
            dst = np.zeros((n, 4))
            for j in range(n):
                dst[j, :] = attitude.euler2quat(src[j, :])
            return dst
        # dict557
        elif isinstance(src, dict):
            dst = {}
            for i in src:
                n = src[i].shape[0]
                quat = np.zeros((n, 4))
                for j in range(n):
                    quat[j, :] = attitude.euler2quat(src[i][j, :])
                dst[i] = quat
            return dst
        else:
            raise ValueError('%s is not a dict or numpy array.'% src.name)

    def __convert_pos(self, data, units, ref_frame):
        '''
        Convert position data into a proper form.
        For example, if units are [deg deg m] or [rad rad m] and ref_frame is 1, convertion
        is needed. LLA form position will be converted to [x y z] form. Vice Versa.
        Args:
            data: nx3 numpy array, can be in [Lat Lon Alt] or [x y z] form.
            units: units of the data.
            ref_frame: reference frame of the simulation. 0:NED, 1:virtual inertial
        Returns:
            data: nx3 numpy array after convertion.
            units: units of converted dta
        '''
        if ref_frame == 1:
            # deg to rad
            if units == ['deg', 'deg', 'm']:
                units = ['rad', 'rad', 'm']
                data[:, 0] = data[:, 0] * attitude.D2R
                data[:, 1] = data[:, 1] * attitude.D2R
            # lla2ned
            if units == ['rad', 'rad', 'm']:
                units = ['m', 'm', 'm']
                # relative motion in ECEF
                data = geoparams.lla2ecef_batch(data)
                ini_pos_ecef = data[0, :]   # initial ECEF position
                data = data - ini_pos_ecef
                # relative motion in ECEF to NED, NED defined by first LLA
                c_ne = attitude.ecef_to_ned(data[0, 0], data[0, 1])
                data = data.dot(c_ne.T)
                data = data + ini_pos_ecef
        elif ref_frame == 0:
            # ned2lla or ecef2lla
            #  Because if the data are in NED or ECEF is unknown, this is not supported.
            if units == ['m', 'm', 'm']:
                units = ['rad', 'rad', 'm']
                print("Unsupported position conversion from xyz to LLA.")
        return data, units
