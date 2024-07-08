# -*- coding: utf-8 -*-
# Filename: demo_kml_gen.py

"""
The simplest demo of kml generation.
Created on 2018-01-23
@author: dongxiaoguang
"""

import os
import sys
import math
import numpy as np
from gnss_ins_sim.kml_gen import kml_gen2

# globals
D2R = math.pi/180

ins_cfg = {'delim':',', 'headerLines':1, 'idx_week':0, 'idx_tow':1, 'idx_lla':(4, 5, 6),\
           'idx_rpy':(12, 13, 14), 'track_icon':'ins'}
gnss_err_cfg = {'delim':',', 'headerLines':0, 'idx_week':0, 'idx_tow':1, 'idx_lla':(2, 3, 4, 8, 9),\
           'idx_rpy':(5, 6, 7), 'track_icon':'gnss'}
ins_err_cfg = {'delim':',', 'headerLines':0, 'idx_week':0, 'idx_tow':1, 'idx_lla':(2, 3, 4, 8, 9),\
           'idx_rpy':(5, 6, 7), 'track_icon':'ins'}
span_cfg = {'delim':None, 'headerLines':45, 'idx_week':0, 'idx_tow':1, 'idx_lla':(2, 3, 4),\
           'idx_rpy':(8, 9, 10), 'track_icon':'ref'}
compact_cfg = {'delim':',', 'headerLines':1, 'idx_week':0, 'idx_tow':0, 'idx_lla':(1, 2, 3),\
           'idx_rpy':(4, 5, 6), 'track_icon':'ins'}

def test_kml_gen(lla_file, cfg, dt, time_span):
    '''
    test only path generation in Sim.
    '''
    pos_info = np.genfromtxt(lla_file, delimiter=cfg['delim'], skip_header=cfg['headerLines'])
    time_stamp = {'week': pos_info[:, cfg['idx_week']], 'tow': pos_info[0:, cfg['idx_tow']]}
    lla = pos_info[:, cfg['idx_lla']]
    rpy = pos_info[:, cfg['idx_rpy']]
    lla[:, 0] = lla[:, 0] * D2R
    lla[:, 1] = lla[:, 1] * D2R
    if time_span != []:
        idx = (time_stamp['tow'] >= time_span[0]) & (time_stamp['tow'] <= time_span[1])
        time_stamp['week'] = time_stamp['week'][idx]
        time_stamp['tow'] = time_stamp['tow'][idx]
        lla = lla[idx, :]
        rpy = rpy[idx, :]

    kml_gen2.kml_gen(os.path.dirname(lla_file), pos=lla, rpy=rpy, time_stamp=time_stamp,\
                    dt=dt, name=os.path.basename(lla_file)[0:-4],\
                    track_icon=cfg['track_icon'])

if __name__ == '__main__':
    #### defaults
    in_data_dir = 'f:\\desktop\\tmp\\INS502\pl\\postpro\\'
    in_file_name = '20240222_053_AM_Antenna_DY_INS402_UM982.postproc.txt'
    in_file = os.path.abspath(in_data_dir + in_file_name)
    in_cfg = span_cfg
    in_dt = 0.1
    in_time_span = []
    in_time_span = [350500, 353800]

    #### args
    nargin = len(sys.argv)
    if nargin > 1:
        in_file = sys.argv[1]
        if nargin > 2:
            cmd_str = sys.argv[2]
            if cmd_str.lower() == 'ins':
                in_cfg = ins_cfg
            elif cmd_str.lower() == 'ins_err':
                in_cfg = ins_err_cfg
            elif cmd_str.lower() == 'gnss_err':
                in_cfg = gnss_err_cfg
            elif cmd_str.lower() == 'span':
                in_cfg = span_cfg
            elif cmd_str.lower() == 'compact':
                in_cfg = compact_cfg
            else:
                in_cfg = ins_cfg
            if nargin > 3:
                in_dt = float(sys.argv[3])
                in_time_span = []
                if nargin > 5:
                    in_time_span = [float(sys.argv[4]), float(sys.argv[5])]

    test_kml_gen(in_file, in_cfg, in_dt, in_time_span)
