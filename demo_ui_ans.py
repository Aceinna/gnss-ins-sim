# -*- coding: utf-8 -*-
# Filename: demo_ui_ans.py

"""
Demo of using ANS as GUI
Created on 2020-02-03
@author: dongxiaoguang
"""

import os

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
from gnss_ins_sim.gui import gui_ans


def test_path_gen():
    '''
    test ANS as GUI.
    '''
    #### simulation config
    motion_def_path = os.path.abspath('.//demo_motion_def_files//')
    fs = 100.0          # IMU sample frequency
    fs_gps = 10.0       # GPS sample frequency
    fs_mag = fs         # magnetometer sample frequency, not used for now

    #### choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    #### start simulation
    with open(motion_def_path+"//motion_def-3d.csv", 'r') as f:
        tmp = f.read()
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      tmp,
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run(1)
    # save simulation data to files
    sim.results('')

    #### GUI
    gui = gui_ans.GuiAns()
    gui.start(sim)

if __name__ == '__main__':
    test_path_gen()
