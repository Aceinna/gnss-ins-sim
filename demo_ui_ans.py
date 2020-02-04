# -*- coding: utf-8 -*-
# Filename: demo_ui_ans.py

"""
Demo of using ANS as GUI
Created on 2020-02-03
@author: dongxiaoguang
"""

import os
import sys
import math
import collections
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

import json
import time
import tornado.websocket
import tornado.ioloop
import tornado.httpserver
import tornado.web

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0          # IMU sample frequency
fs_gps = 10.0       # GPS sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now

server_version = '1.1.1'
callback_rate = 50  # ms
class WSHandler(tornado.websocket.WebSocketHandler):
    '''
    doc
    '''

    def initialize(self, sim_data):
        '''
        doc
        '''
        # get sim data
        self.sim_data = sim_data[0]
        # load json file
        self.json_data = sim_data[1]
        self.output_message = self.json_data['userMessages']['outputPackets'][0]
        # index of data to be sent
        self.idx = 0
        self.idx_increment = round(callback_rate/(1000/fs))

    def open(self):
        self.callback = tornado.ioloop.PeriodicCallback(self.send_data, callback_rate)
        # self.callback.start()
        self.callback2 = tornado.ioloop.PeriodicCallback(self.detect_status, 500)
        # self.callback2.start()

    def detect_status(self):
        '''
        doc
        '''
        # self.write_message(json.dumps({ "messageType": "queryResponse",
        #                                 "data": {"packetType": "DeviceStatus",
        #                                          "packet": { "returnStatus":1}}}))

    def send_data(self):
        '''
        doc
        '''
        # all data played?
        if self.idx >= self.sim_data[0].shape[0]:
            return True
        # play data
        d = [('time', self.sim_data[0][self.idx]),
             ('xAccel', self.sim_data[1][0][self.idx][0]),
             ('yAccel', self.sim_data[1][0][self.idx][1]),
             ('zAccel', self.sim_data[1][0][self.idx][2]),
             ('xRate', self.sim_data[2][0][self.idx][0]),
             ('yRate', self.sim_data[2][0][self.idx][1]),
             ('zRate', self.sim_data[2][0][self.idx][2]),
             ('lat', self.sim_data[3][self.idx][0] * 180 / math.pi),
             ('lon', self.sim_data[3][self.idx][1] * 180 / math.pi),
             ('alt', self.sim_data[3][self.idx][2]),
             ('velNorth', self.sim_data[4][self.idx][0]),
             ('velEast', self.sim_data[4][self.idx][1]),
             ('velDown', self.sim_data[4][self.idx][2])]
        d = collections.OrderedDict(d)
        self.write_message(json.dumps({'messageType' : 'event', 'data' : {'newOutput' : d}}))
        # next data
        self.idx += self.idx_increment

        return True

    def on_message(self, message):
        message = json.loads(message)

        if not message.__contains__('messageType'):
            return

        # Except for a few exceptions stop the automatic message transmission if a message
        # is received
        if message['messageType'] != 'serverStatus' and\
           list(message['data'].keys())[0] != 'startLog' and\
           list(message['data'].keys())[0] != 'stopLog':
            self.callback.stop()

        if message['messageType'] == 'serverStatus':
            # load application type from firmware
            try:
                self.write_message(json.dumps({'messageType' : 'serverStatus',
                                               'data' : {'serverVersion' : server_version,
                                                         'serverUpdateRate' : callback_rate,
                                                         'packetType' : 'e2',
                                                         'deviceProperties' : self.json_data,
                                                         'deviceId' : 'gnss-ins-sim 0 1.1.2 SN:0',
                                                         'logging' : False,
                                                         'fileName' : ''}}))
            except Exception as e:
                print(e)
                self.write_message(json.dumps({"messageType": "queryResponse",
                                               "data": {"packetType": "DeviceStatus",
                                                        "packet": {"returnStatus":2}}}))

        elif message['messageType'] == 'requestAction':
            print(message['data'].keys())
            data = []
            if list(message['data'].keys())[0] == 'gA':
                data = [{"paramId": 0, "name": "Packet Type", "value": 'e2'},
                        {"paramId": 1, "name": "Packet Rate", "value": fs}]
                msg = json.dumps({"messageType" : "requestAction", "data" : {"gA" : data}})
                self.write_message(msg)
            elif list(message['data'].keys())[0] == 'uP':
                self.write_message(json.dumps({"messageType" : "requestAction",
                                               "data" : {"uP" : data}}))
            elif list(message['data'].keys())[0] == 'sC':
                time.sleep(0.5)
                self.write_message(json.dumps({"messageType" : "requestAction",
                                               "data" : {"sC" : {}}}))
            elif list(message['data'].keys())[0] == 'gV':
                data = 'gnss-ins-sim'
                msg = json.dumps({"messageType" : "completeAction",
                                  "data" : {"gV" : str(data)}})
                self.write_message(msg)
            elif list(message['data'].keys())[0] == 'startStream':
                self.callback.start()
                self.callback2.stop()
                self.write_message(json.dumps({"messageType" : "requestAction",
                                               "data" : {"startStream" : {}}}))
            elif list(message['data'].keys())[0] == 'stopStream':
                self.callback2.start()
                self.write_message(json.dumps({"messageType" : "requestAction",
                                               "data" : {"stopStream" : {}}}))
            elif list(message['data'].keys())[0] == 'startLog':
                pass
            elif list(message['data'].keys())[0] == 'stopLog':
                pass

    def on_close(self):
        self.callback.stop()
        self.callback2.stop()
        time.sleep(1.2)
        return False

    def check_origin(self, origin):
        return True

def test_path_gen(tcp_port=None):
    '''
    test ANS as GUI.
    '''
    #### choose a built-in IMU model, typical for IMU381
    imu_err = 'mid-accuracy'
    # generate GPS and magnetometer data
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    #### start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"//motion_def-long_drive.csv",
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run(1)
    # save simulation data to files
    sim.results('')

    ### GUI
    # tcp port
    start_tcp_port = 8000
    end_tcp_port = 8002
    # sim data
    sim_data = sim.dmgr.get_data(['time', 'accel', 'gyro', 'ref_pos', 'ref_vel'])
    # json
    folder_path = 'e:/py_projects/gnss-ins-sim/demo_saved_data/openimu.json'
    with open(folder_path) as json_file:
        try:
            json_data = json.load(json_file)
        except Exception as e:
            print(e)

    application = tornado.web.Application([(r'/', WSHandler, dict(sim_data=[sim_data, json_data]))])
    http_server = tornado.httpserver.HTTPServer(application)
    if tcp_port is None:
        tcp_port = start_tcp_port
        while True:
            try:
                http_server.listen(tcp_port)
                break
            except Exception as e:
                if tcp_port > end_tcp_port:
                    print('port conflict, please input port with command: -p port_number, type -h will get the help information!')
                    os._exit(0)
                tcp_port = tcp_port + 1
    else:
        http_server.listen(tcp_port)
    tornado.ioloop.IOLoop.instance().start()

if __name__ == '__main__':
    test_path_gen()
