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
import json
import time
import collections

import tornado.websocket
import tornado.ioloop
import tornado.httpserver
import tornado.web

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# json str
json_str = '''
{
	"name": "OpenIMU300_INS",
	"app_version": "OpenIMU300ZI INS 1.1.1",
	"appName": "OpenIMU300ZI INS 1.1.1",
	"type": "openimu",
	"description": "9-axis OpenIMU with INS application",
	"userConfiguration": [
		{ "paramId": 0, "category": "General","paramType": "select",
			"type": "char8", "name": "Packet Type", "argument": "type",
			"value_range":[-1,1],"value_accuracy":6,
			"options": ["z1", "a1", "a2", "s1", "e1", "e2"] },
		{ "paramId": 1, "category": "General","paramType": "select",
			"type": "int64", "name": "Packet Rate", "argument": "rate",
			"value_range":[-1,1],"value_accuracy":6,
			"options": [200, 100, 50, 20, 10, 5, 2, 0] },
		{ "paramId": 2, "category": "General","paramType": "select",
			"type": "int64", "name": "Play speed x", "argument": "speed",
			"value_range":[-1,1],"value_accuracy":6,
			"options": [1, 2, 5, 10, 20] }
	],
	"userMessages": {
		"inputPackets": [],
		"outputPackets": [
			{
				"name": "e2",
				"description": "INS Output Message",
				"payload": [
					{
						"type": "double",
						"name": "time",
						"unit": "s"
					},
					{
						"type": "double",
						"name": "xAccel",
						"unit": "g"
					},
					{
						"type": "double",
						"name": "yAccel",
						"unit": "g"
					},
					{
						"type": "double",
						"name": "zAccel",
						"unit": "g"
					},
					{
						"type": "double",
						"name": "xRate",
						"unit": "deg/s"
					},
					{
						"type": "double",
						"name": "yRate",
						"unit": "deg/s"
					},
					{
						"type": "double",
						"name": "zRate",
						"unit": "deg/s"
					},
					{
						"type": "double",
						"name": "velNorth",
						"unit": "m/s"
					},
					{
						"type": "double",
						"name": "velEast",
						"unit": "m/s"
					},
					{
						"type": "double",
						"name": "velDown",
						"unit": "m/s"
					},
					{
						"type": "double",
						"name": "lat",
						"unit": "deg"
					},
					{
						"type": "double",
						"name": "lon",
						"unit": "deg"
					},
					{
						"type": "double",
						"name": "alt",
						"unit": "m"
					}
				],
				"graphs": [
					{
						"name": "Geo Map",
						"units": "deg",
						"renderType":"map",
						"fields":[
							{ "name":"lat" , "display": "Latitude" },
							{ "name":"lon" , "display": "Longitude" }
						],
						"options":{
							"hasTrajectory":false
						}     
					},
					{
						"name": "Acceleration",
						"units": "g",
						"xAxis": "Time (s)",
						"yAxes": ["xAccel", "yAccel", "zAccel"],
						"colors": ["#FF0000", "#00FF00", "#0000FF"],
						"yMax": 8
					},
					{
						"name": "Angular-Rate",
						"units": "deg/s",
						"xAxis": "Time (s)",
						"yAxes": ["xRate", "yRate", "zRate"],
						"colors": ["#FF0000", "#00FF00", "#0000FF"],
						"yMax": 400
					},
					{
						"name": "Latitude/Longitude",
						"units": "deg",
						"xAxis": "Time (s)",
						"yAxes": ["lat", "lon"],
						"colors": ["#FF0000", "#00FF00"],
						"yMax": 200
					},
					{
						"name": "Altitude",
						"units": "m",
						"xAxis": "Time (s)",
						"yAxes": ["alt"],
						"colors": ["#FF0000"],
						"yMax": 100
					},
					{
						"name": "NED Velocity",
						"units": "m/s",
						"xAxis": "Time (s)",
						"yAxes": ["velNorth", "velEast", "velDown"],
						"colors": ["#FF0000", "#00FF00", "#0000FF"],
						"yMax": 100
					}
				]
			}
		]
	}
}
'''
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
        # get dat ODR
        self.fs = sim_data[0]
        # get sim data
        self.sim_data = sim_data[1]
        # load json file
        self.json_data = sim_data[2]
        self.output_message = self.json_data['userMessages']['outputPackets'][0]
        # index of data to be sent
        self.idx = 0
        self.idx_increment = round(callback_rate/(1000/self.fs))
        self.speed_times = 1

    def open(self):
        self.callback_send_data = tornado.ioloop.PeriodicCallback(self.send_data, callback_rate)
        # self.callback.start()
        self.callback_heartbeat = tornado.ioloop.PeriodicCallback(self.detect_status, 500)
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
            self.callback_send_data.stop()
            return True
        # play data
        d = [('time', self.sim_data[0][self.idx] * 1000),
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
        self.idx += self.idx_increment * self.speed_times

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
            self.callback_send_data.stop()

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
                        {"paramId": 1, "name": "Packet Rate", "value": self.fs},
                        {"paramId": 2, "name": "Play speed x", "value": self.speed_times}]
                msg = json.dumps({"messageType" : "requestAction", "data" : {"gA" : data}})
                self.write_message(msg)
            elif list(message['data'].keys())[0] == 'uP':
                data = message['data']['uP']
                if data['paramId'] == 0:
                    pass
                elif data['paramId'] == 1:
                    pass
                elif data['paramId'] == 2:
                    self.speed_times = data['value']
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
                # If play is finished, play from beginning again
                if self.idx >= self.sim_data[0].shape[0]:
                    self.idx = 0
                self.callback_send_data.start()
                self.callback_heartbeat.stop()
                self.write_message(json.dumps({"messageType" : "requestAction",
                                               "data" : {"startStream" : {}}}))
            elif list(message['data'].keys())[0] == 'stopStream':
                self.callback_heartbeat.start()
                self.write_message(json.dumps({"messageType" : "requestAction",
                                               "data" : {"stopStream" : {}}}))

    def on_close(self):
        self.callback_send_data.stop()
        self.callback_heartbeat.stop()
        time.sleep(1.2)
        return False

    def check_origin(self, origin):
        return True

def test_path_gen(tcp_port=None):
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
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+"//motion_def-3d.csv",
                      ref_frame=0,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run(1)
    # save simulation data to files
    sim.results('')

    #### GUI
    # tcp port
    start_tcp_port = 8000
    end_tcp_port = 8002
    # sim data
    sim_data = sim.get_data(['time', 'accel', 'gyro', 'ref_pos', 'ref_vel'])
    # json
    json_data = json.loads(json_str)

    application = tornado.web.Application([(r'/', WSHandler,\
        dict(sim_data=[fs, sim_data, json_data]))])
    http_server = tornado.httpserver.HTTPServer(application)
    if tcp_port is None:
        tcp_port = start_tcp_port
        while True:
            try:
                http_server.listen(tcp_port)
                break
            except:
                if tcp_port == end_tcp_port:
                    print('port conflict, please input port with command: '\
                        '-p port_number, type -h will get the help information!')
                    sys.exit()
                tcp_port = tcp_port + 1
    else:
        http_server.listen(tcp_port)
    tornado.ioloop.IOLoop.instance().start()

if __name__ == '__main__':
    test_path_gen()
