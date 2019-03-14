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
import argparse
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))

import numpy as np
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
from demo_algorithms import free_integration

# stddev of initial velocity error in m/s
INIT_VEL_ERROR = 0.01
# stddev of initial attitude error in deg
INIT_ATT_ERROR = 0.05

IMU_MODELS = {
'bmi160': {},
'imu381': {'gyro_b': np.array([0.0, 0.0, 0.0]),
           'gyro_arw': np.array([0.25, 0.25, 0.25]) * 1.0,
           'gyro_b_stability': np.array([3.5, 3.5, 3.5]) * 1.0,
           'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
           'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
           'accel_vrw': np.array([0.03119, 0.03009, 0.04779]) * 1.0,
           'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]) * 1.0,
           'accel_b_corr': np.array([200.0, 200.0, 200.0]),
           'mag_std': np.array([0.2, 0.2, 0.2]) * 1.0}
}

D2R = np.pi / 180.0

def make_motion_def(args):
    return None
    
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
    ini_pos_vel_att[3:6] += init_vel_err
    ini_pos_vel_att[6:9] += init_att_err
    return ini_pos_vel_att


def run_and_save_results(args, motion_def):
    imu = imu_model.IMU(accuracy=IMU_MODELS[args.imu], axis=6, gps=False)
    ini_pos_vel_att = read_initial_condition(motion_def)
    for i in range(args.N):
        if args.enable_init_error:
            init_cond = perturbed_initial_condition(ini_pos_vel_att)  
        else:
            init_cond = ini_pos_vel_att
        algo = free_integration.FreeIntegration(init_cond)
        sim = ins_sim.Sim([args.fs, 0.0, 0.0],
                           motion_def,
                           ref_frame=0,
                           imu=imu,
                           mode=None,
                           env=None,
                           algorithm=algo)
        sim.run(1)
        sim.results(args.outdir, end_point=True)

if __name__ == "__main__":
    # Collect arguments.
    parser = argparse.ArgumentParser(description='Generate DR trajectories.')
    parser.add_argument('--outdir', type=str, required=True,
                        help='Output directory.')
    parser.add_argument('--N', type=int, required=True,
                        help='Number of trajectories to generate.')
    parser.add_argument('--dur', type=float, required=True,
                        help='Duration of trajectories.')
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
    motion_def = "../demo_motion_def_files/motion_def.csv"
    run_and_save_results(args, motion_def)

