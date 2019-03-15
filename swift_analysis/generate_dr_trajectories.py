# Copyright (C) 2018 Swift Navigation Inc.
# Contact: <dev@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import os
import sys
import glob
import shutil
import argparse
import tempfile
import contextlib
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
from demo_algorithms import free_integration

# stddev of initial velocity error in m/s
INIT_VEL_ERROR = 0.01
# stddev of initial attitude error in deg
INIT_ATT_ERROR = 0.05

G = 9.80665 # m/s^2
UG = G / 1.0e6 # m/s^2


# Vibrations experienced during 55mph highway 101 driving
DEFAULT_VIBRATIONS = '[0.0256 0.0174 0.855]g-random'

'''
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

'''

# IMU noise parameters
IMU_MODELS = {
# TODO(niels)
'bmi160': {'gyro_b': np.array([0.0, 0.0, 0.0]),
             'gyro_arw': np.array([0.3160, 0.3160, 0.3160]),
             'gyro_b_stability': np.array([4.3730, 4.3730, 4.3730]),
             'gyro_b_corr': np.array([202.68, 202.68, 202.68]),
             'accel_b': np.array([0.0, 0.0, 0.0]),
             'accel_vrw': np.array([1., 1., 1.]) * 0.07322625555,
             'accel_b_stability': np.array([1., 1., 1.]) * 33.9700 * UG,
             'accel_b_corr': np.array([48.24, 48.24, 48.24])},

'imu381': {'gyro_b': np.array([0.0, 0.0, 0.0]),
           'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0,
           'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0,
           'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
           'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
           'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0,
           'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0,
           'accel_b_corr': np.array([200.0, 200.0, 200.0]),
           'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0},

'terrible': {'gyro_b': np.array([0.0, 0.0, 0.0]),
             'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0e2,
             'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0e2,
             'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
             'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
             'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0e2,
             'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0e2,
             'accel_b_corr': np.array([200.0, 200.0, 200.0]),
             'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0}
}

# Conversion constants.
D2R = np.pi / 180.0
MPH2MS = 0.44704

def make_motion_def(args):
    init_header = ",".join(["ini lat (deg)",
                            "ini lon (deg)",
                            "ini alt (m)",
                            "ini vx_body (m/s)",
                            "ini vy_body (m/s)",
                            "ini vz_body (m/s)",
                            "ini yaw (deg)",
                            "ini pitch (deg)",
                            "ini roll (deg)"])
    traj_header = ",".join(["command type",
                            "yaw (deg)",
                            "pitch (deg)",
                            "roll (deg)",
                            "vx_body (m/s)",
                            "vy_body (m/s)",
                            "vz_body (m/s)",
                            "command duration (s)",
                            "GPS visibility"])

    speed_ms = args.speed * MPH2MS
    init_values = [32., 120., 0., speed_ms, 0., 0., 0., 0., 0.]
    traj_values = [1, 0., 0., 0., 0., 0., 0., args.dur, 0]
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
        f.write(init_header + "\n")
        f.write(",".join([str(v) for v in init_values]) + "\n")
        f.write(traj_header + "\n")
        f.write(",".join([str(v) for v in traj_values]) + "\n")
        return f.name
    
def read_initial_condition(motion_def):
    ini_pos_vel_att = np.genfromtxt(motion_def,
                                    delimiter=',', 
                                    skip_header=1, 
                                    max_rows=1)
    ini_pos_vel_att[0] = ini_pos_vel_att[0] * D2R
    ini_pos_vel_att[1] = ini_pos_vel_att[1] * D2R
    ini_pos_vel_att[6:9] = ini_pos_vel_att[6:9] * D2R
    return ini_pos_vel_att

def perturbed_initial_condition(ini_pos_vel_att):
    ini_vel_err = np.random.multivariate_normal(np.zeros((3,)), 
                                                np.eye(3) * INIT_VEL_ERROR)
    ini_att_err = np.random.multivariate_normal(np.zeros((3,)),
                                                np.eye(3) * INIT_ATT_ERROR) * D2R
    ini_pos_vel_att[3:6] += ini_vel_err
    ini_pos_vel_att[6:9] += ini_att_err
    return ini_pos_vel_att

def run_and_save_results(args, motion_def):
    resultsdir = args.outdir
    stagingdir = os.path.join(resultsdir, "staging")
    # Before running, warn if the resultsdir holds existing results.
    if glob.glob("{}/dr_*.csv".format(resultsdir)):
        print("WARNING: DR trajectory results already exist in the given results directory.")
        print("         You may end up with a mismatched set.")
    # Setup and run N simulations.
    imu = imu_model.IMU(accuracy=IMU_MODELS[args.imu], axis=6, gps=False)
    ini_pos_vel_att = read_initial_condition(motion_def)
    for i in range(args.N):
        if args.enable_init_error:
            init_cond = perturbed_initial_condition(ini_pos_vel_att)  
        else:
            init_cond = ini_pos_vel_att
        if args.enable_vibrations:
            env=DEFAULT_VIBRATIONS
        else:
            env=None
        algo = free_integration.FreeIntegration(init_cond)
        sim = ins_sim.Sim([args.fs, 0.0, 0.0],
                           motion_def,
                           ref_frame=0,
                           imu=imu,
                           mode=None,
                           env=env,
                           algorithm=algo)
        sim.run(1)
        # We don't care for the printed results.
        with open(os.devnull, 'w') as devnull:
            stdout = sys.stdout
            sys.stdout = devnull
            sim.results(stagingdir, end_point=True)
            sys.stdout = stdout
        collate_sim_results(stagingdir, os.path.join(resultsdir, "dr_{}.csv".format(i)))
    shutil.rmtree(stagingdir)

def collate_sim_results(simresultsdir, outfile):
    SIM_RESULTS_FILES = ['time.csv', 
                         'pos-algo0_0.csv', 
                         'ref_pos.csv',
                         'accel-0.csv', 
                         'gyro-0.csv']
    paths = [os.path.join(simresultsdir, f) for f in SIM_RESULTS_FILES]
    df = pd.concat([pd.read_csv(os.path.join(simresultsdir, f)) for f in SIM_RESULTS_FILES],
                   axis=1)
    df.to_csv(outfile, index=None) 

if __name__ == "__main__":
    # Collect arguments.
    parser = argparse.ArgumentParser(description='Generate DR trajectories.')
    parser.add_argument('--outdir', type=str, required=True,
                        help='Output directory.')
    parser.add_argument('--N', type=int, required=True,
                        help='Number of trajectories to generate.')
    parser.add_argument('--dur', type=int, required=True,
                        help='Duration of trajectories in seconds.')
    parser.add_argument('--speed', type=float, required=True,
                        help='Speed in mph.')
    parser.add_argument('--imu', choices=IMU_MODELS.keys(), required=True,
                        help='IMU sensor noise parameters to use.')
    parser.add_argument('--fs', type=float, default=100.,
                        help='IMU sample rate.')
    parser.add_argument('--enable-vibrations', action='store_true', default=False,
                        help='Enable simulated vibrations.')
    parser.add_argument('--enable-init-error', action='store_true', default=False,
                        help='Enable small errors in the initial state estimate.')
    args = parser.parse_args()
    # Generate and run motion defs.
    motion_def = make_motion_def(args)
    run_and_save_results(args, motion_def)
    os.remove(motion_def)

