# -*- coding: utf-8 -*-
# Fielname = gui_ans.py

"""
Use the Aceinna navigation studio as the GUI.
Created on 2020-02-17
@author: dongxiaoguang
"""
import sys
import json
import time
import collections

import tornado.websocket
import tornado.ioloop
import tornado.httpserver
import tornado.web

from ..sim import ins_sim
from ..attitude import attitude

server_version = '1.1.1'

class WSHandler(tornado.websocket.WebSocketHandler):
    '''
    doc
    '''
    def initialize(self, sim_cfg):
        '''
        doc
        '''
        # call back interval
        self.send_data_interval = sim_cfg[0]
        self.heartbeat_interval = sim_cfg[1]
        # device info
        self.deviceInfo = sim_cfg[2]
        # UI configuration in json
        self.json_data = sim_cfg[3]
        # get sim data
        self.get_next_data = sim_cfg[4]
        # get setting
        self.get_setting = sim_cfg[5]
        # update setting
        self.update_setting = sim_cfg[6]

    def open(self):
        self.callback_send_data = tornado.ioloop.PeriodicCallback(self.send_data,\
                                                                  self.send_data_interval)
        # self.callback.start()
        self.callback_heartbeat = tornado.ioloop.PeriodicCallback(self.detect_status,\
                                                                  self.heartbeat_interval)
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
        latest_data = self.get_next_data()
        if latest_data is None:
            return
        d = collections.OrderedDict(latest_data)
        self.write_message(json.dumps({'messageType': 'event', 'data': {'newOutput': d}}))
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
                msg = json.dumps({'messageType' : 'serverStatus',
                                  'data' : {'serverVersion' : server_version,
                                            'serverUpdateRate' : self.send_data_interval,
                                            'packetType' : 'e2',
                                            'deviceProperties' : self.json_data,
                                            'deviceId' : self.deviceInfo,
                                            'logging' : False,
                                            'fileName' : ''}})
                self.write_message(msg)
            except Exception as e:
                print(e)
                msg = json.dumps({"messageType": "queryResponse",
                                  "data": {"packetType": "DeviceStatus",
                                           "packet": {"returnStatus":2}}})
                self.write_message(msg)
        elif message['messageType'] == 'requestAction':
            data = []
            if list(message['data'].keys())[0] == 'gA':
                data = self.get_setting(-1)
                msg = json.dumps({"messageType" : "requestAction", "data" : {"gA" : data}})
                self.write_message(msg)
            elif list(message['data'].keys())[0] == 'uP':
                self.update_setting(message['data']['uP']['paramId'],\
                                    message['data']['uP']['value'])
                self.write_message(json.dumps({"messageType" : "requestAction",
                                               "data" : {"uP" : data}}))
            elif list(message['data'].keys())[0] == 'sC':
                time.sleep(0.5)
                self.write_message(json.dumps({"messageType" : "requestAction",
                                               "data" : {"sC" : {}}}))
            elif list(message['data'].keys())[0] == 'gV':
                msg = json.dumps({"messageType" : "completeAction",
                                  "data" : {"gV" : self.deviceInfo}})
                self.write_message(msg)
            elif list(message['data'].keys())[0] == 'startStream':
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


class GuiAns(object):
    '''
    Use the Aceinna navigation studio as the GUI
    '''
    def __init__(self, tcp_port=None):
        '''
        Initialize parameters needed to connect to ANS.
        Args:
            tcp_port:
        '''
        # callback rate of data sending and hearbeat
        self.callbak_send_data_interval = 50    # ms
        self.callbak_hearbeat_interval = 500    # ms
        # control which data is fed to the Websocket
        self.idx = 0
        self.idx_step = 1
        self.num_of_samples = 0
        # Websocket server port
        self.tcp_port = tcp_port
        # device information
        self.device_info = ''
        # settings and graphs
        self.json = {}
        self.settings = {}
        # data and data name
        self.sim_fs = 1     # Hz
        self.sim_data = []
        self.sim_data_names = []
        # Indicate if this is the first run
        self.first_run = True

    def start(self, sim_obj: ins_sim.Sim, reset=False):
        '''
        doc
        '''
        # Start with the first sample of the simulation data.
        self.idx = 0

        # generate json and corresponding data
        if reset or self.first_run:
            self.gen_json_and_data(sim_obj)
        # update index increment step
        self.sim_fs = sim_obj.fs[0]
        self.update_idx_step()
        #start web server
        start_tcp_port = 8000
        end_tcp_port = 8002
        if self.first_run:
            self.device_info = self.gen_device_info(sim_obj)
            sim_cfg_and_data = [self.callbak_send_data_interval, self.callbak_hearbeat_interval,\
                                self.device_info, self.json, \
                                self.get_next_data, self.get_setting, self.update_setting]
            self.start_ws_server(start_tcp_port, end_tcp_port, sim_cfg_and_data)
        # First run completed
        self.first_run = False

    def get_next_data(self):
        '''
        doc
        '''
        rtn = None
        if self.idx < self.num_of_samples:
            rtn = []
            for i in range(len(self.sim_data_names)):
                if len(self.sim_data_names[i]) > 1:
                    for j in range(len(self.sim_data_names[i])):
                        rtn.append((self.sim_data_names[i][j], self.sim_data[i][self.idx][j]))
                else:
                    rtn.append((self.sim_data_names[i][0], self.sim_data[i][self.idx]))
        self.idx += self.idx_step
        return rtn

    def get_device_info(self):
        '''
        Get device info
        '''
        return self.device_info

    def get_setting(self, paramId):
        '''
        Get setting value according to paramId.
        '''
        rtn = []
        json_cfg = self.json['userConfiguration']
        for i in json_cfg:
            if paramId == -1 or paramId == i['paramId']:
                rtn.append({'paramId': i['paramId'],\
                            'name': i['name'],\
                            'value': self.settings[i['paramId']]})
        return rtn

    def update_setting(self, paramId, value):
        '''
        Update setting value.
        '''
        if paramId in self.settings.keys():
            self.settings[paramId] = value
            # update corresponding parameters, TBD
            self.update_idx_step()
            return True
        else:
            return False

    def gen_json_and_data(self, sim_obj: ins_sim.Sim):
        '''
        Generate json and corresponding data.
        Args:
            sim_obj: ins_sim.Sim object.
        '''
        self.json['name'] = sim_obj.name
        self.json['app_version'] = sim_obj.name
        self.json['appName'] = sim_obj.version
        self.json['type'] = ''
        self.json['description'] = ''
        self.json['userConfiguration'] = []
        self.json['userMessages'] = {}
        self.json['userMessages']['inputPackets'] = []
        self.json['userMessages']['outputPackets'] = [{}]
        self.json['userMessages']['outputPackets'][0]['graphs'] = []
        self.json['userMessages']['outputPackets'][0]['name'] = 'e2'
        self.add_setting(0, 'Packet Type', 'char8', 'select', 'General',\
                        ["e2"])
        self.add_setting(1, 'Packet Rate', 'int64', 'select', 'General',\
                        [100])
        self.add_setting(2, 'Play speed x', 'int64', 'select', 'General',\
                        [1, 2, 5, 10, 20])
        # gen a list of the names of data that can be plotted
        all_sim_data = sim_obj.get_names_of_available_data()
        for data_name in all_sim_data:
            # do not support GPS data graph
            if 'gps' in data_name:
                continue
            data_attrib = sim_obj.get_data_properties(data_name)
            # only that can be plotted is included
            if data_attrib[2]:
                tmp_sim_data = sim_obj.get_data([data_name])[0]
                # do not add a graph for time, the unit of time is converted to ms, which should
                # be deleted after the Web GUI is fixed.
                if data_name == 'time':
                    self.num_of_samples = tmp_sim_data.shape[0]
                    self.sim_data.append(tmp_sim_data)
                    self.sim_data_names.append(data_attrib[5])
                    continue
                # add graph
                if isinstance(tmp_sim_data, dict):
                    for key in tmp_sim_data.keys():
                        if 'pos' in data_name and 'rad' in data_attrib[1]:
                            tmp_sim_data[key][:, 0] *= attitude.R2D
                            tmp_sim_data[key][:, 1] *= attitude.R2D
                            data_attrib[1] = ['deg', 'deg', 'm']
                        self.sim_data.append(tmp_sim_data[key])
                        sim_data_name = [lgd+'_#'+str(key) for lgd in data_attrib[5]]
                        self.sim_data_names.append(sim_data_name)
                        self.add_graph(data_name, data_attrib[1], options={'yAxes':sim_data_name})
                else:
                    if 'pos' in data_name and 'rad' in data_attrib[1]:
                        tmp_sim_data[:, 0] *= attitude.R2D
                        tmp_sim_data[:, 1] *= attitude.R2D
                        data_attrib[1] = ['deg', 'deg', 'm']
                    self.sim_data.append(tmp_sim_data)
                    self.sim_data_names.append(data_attrib[5])
                    self.add_graph(data_name, data_attrib[1], options={'yAxes':data_attrib[5]})

    def update_idx_step(self):
        '''
        doc
        '''
        self.idx_step = round(self.callbak_send_data_interval/(1000/self.sim_fs) * \
                              self.settings[2])

    def start_ws_server(self, start_tcp_port, end_tcp_port, sim_fs_data_json):
        '''
        Start web socket server
        '''
        application = tornado.web.Application([(r'/', WSHandler,\
        dict(sim_cfg=sim_fs_data_json))])
        http_server = tornado.httpserver.HTTPServer(application)
        if self.tcp_port is None:
            self.tcp_port = start_tcp_port
            while True:
                try:
                    http_server.listen(self.tcp_port)
                    break
                except:
                    if self.tcp_port == end_tcp_port:
                        print('port conflict, please input port with command: '\
                            '-p port_number, type -h will get the help information!')
                        sys.exit()
                    self.tcp_port = self.tcp_port + 1
        else:
            http_server.listen(self.tcp_port)
        tornado.ioloop.IOLoop.instance().start()

    def add_setting(self, pid, pname, ptype, ui_type, ui_category, options):
        '''
        Add a setting.
        Args:
            pid: Parameter ID. An integer starts from 0.
            pname: Parameter name. A string.
            ptype: Parameter type. A string to specify the data type: char8, double, and so on.
            ui_type: UI type. A string to specify the GUI type.
                'select': a combo box.
                'input': an edit box.
                'Disabled': not in use.
            ui_category: A string to specify where the GUI control for this parameter is located.
                'General'
                'Advanced'
            options: According to different ui_type, this list has different meanings:
                For 'select', options provide available options as [opt1, opt2, opt3, ...].
                For 'input', options provide lower and upper limit as [min, max].
                The first element in the list is the default setting.
        '''
        tmp = {}
        tmp['paramId'] = pid
        tmp['name'] = pname
        tmp['type'] = ptype
        tmp['paramType'] = ui_type
        tmp['category'] = ui_category
        if ui_type == 'select':
            tmp['options'] = options
        elif ui_type == 'input':
            tmp['value_range'] = options
        self.json['userConfiguration'].append(tmp)
        self.settings[pid] = options[0]

    def add_graph(self, name, units=None, render_type='line chart', options=None):
        '''
        Add a graph.
        Args:
            name: A string of the name of the graph.
            units: A string to specify the units of the data in the graph.
            render_type: A string to specify the graph type:
                'map': a 2D geo map.
                'None': default, line plot.
            options: Graph options.
                For render_type being 'map', options specify if all samples are kept on the map.
                    True: all samples are shown on the map.
                    False: only the latest sample is shown on the map.
                For
        '''
        # position need a map and a line chart if reference frame is 0 (NED)
        if 'pos' in name:
            # the reference frame is 0 if untis[0] is different from units[2]
            if units[0] != units[2]:
                # add a map
                self.add_map(name, [units[0]], options)
                # add two line charts for [lat lon] and alt
                self.add_line_char(name+'_lat_lon', [units[0]],\
                                  {'yAxes': options['yAxes'][0:2], 'yMax': 180})
                self.add_line_char(name+'_alt', units[2],\
                                  {'yAxes': [options['yAxes'][2]], 'yMax': 10000})
        if render_type.lower() == 'line chart':
            self.add_line_char(name, units, options)
        elif render_type == 'map':
            self.add_map(name, units, options)

    def add_line_char(self, name, units=None, options=None):
        '''
        Add a line chart.
        Args:
            name: name of this chart.
            units: units of the y-axis data in this chart.
            options:
                'xAxis': specify the data of the x axis. default to {'name': 'time', 'unit': 's'}.
                'yAxes': specify the data of the y axis. The y-axis is default to a 3D vector.
                    So, yAxes is default to ['name_x', 'name_y', 'name_z'].
        '''
        tmp = {}
        tmp['name'] = name
        if units is None:
            tmp['units'] = ''
        else:
            tmp['units'] = units[0]
        if options is None:
            options = {}
        # x axis
        if 'xAxis' in options:
            tmp['xAxis'] = options['xAxis']
        else:
            tmp['xAxis'] = {'name': 'time', 'unit': 's'}
        # y axis
        if 'yAxes' in options:
            tmp['yAxes'] = options['yAxes']
        else:
            tmp['yAxes'] = [name+'_x', name+'_y', name+'_z']
        # color of each line
        if 'colors' in options:
            tmp['colors'] = options['colors']
        else:
            tmp['colors'] = self.gen_colors(len(tmp['yAxes']))
        # y limit
        if 'yMax' in options:
            tmp['yMax'] = options['yMax']
        else:
            tmp['yMax'] = 400
        # add to json
        self.json['userMessages']['outputPackets'][0]['graphs'].append(tmp)

    def add_map(self, name, units=None, options=None):
        '''
        Add a map graph.
        Args:
            name: name of the graph.
            units: units of the data in the graph.
            options:
                'yAxes': specify the name of data for latitude and longitude.
                    options['yAxes']=[lat_name, lon_name].
                    latitude name is default to 'lat', longitude name is default to 'lon'.
        '''
        tmp = {}
        tmp['name'] = 'map_' + name
        if units is None:
            tmp['units'] = ''
        else:
            tmp['units'] = units[0]
        tmp['renderType'] = 'map'
        if 'yAxes' in options:
            lat_name = options['yAxes'][0]
            lon_name = options['yAxes'][1]
        else:
            lat_name = 'lat'
            lon_name = 'lon'
        tmp['fields'] = [{'name': lat_name, 'display': 'Latitude'},\
                         {'name': lon_name, 'display': 'Longitude'}]
        tmp['options'] = {'hasTrajectory': False}
        # add to json
        self.json['userMessages']['outputPackets'][0]['graphs'].append(tmp)

    def gen_colors(self, n=1):
        '''
        Generate HEX-format RGB colors for line chart. Some of the colors are same as
        those in Matlab of versions previous to R2014b.
        Args:
            n: Number of colors.
        Returns:
            a list of n HEX-format colors
        '''
        # available corlors
        colors = ['#FF0000',\
                  '#00FF00',\
                  '#0000FF',\
                  '#00BFBF',\
                  '#BF00BF',\
                  '#BFBF00',\
                  '#404040']
        n_int = int(n)
        if n_int < 0:
            n_int = 1
        return colors[0:n_int]

    def gen_device_info(self, sim_obj: ins_sim.Sim):
        '''
        Generate device info as "device_name part_number version SN:serial_number". For example,
        "gnss-ins-sim 0 1.0.0 SN:0"
        Args:
            sim_obj:
        '''
        return sim_obj.name + ' 0 ' + sim_obj.version + ' SN:0'

if __name__ == "__main__":
    gui = GuiAns()
    gui.add_setting(0, 'Packet Type', 'char8', 'select', 'General',\
                    ["z1", "a1", "a2", "s1", "e1", "e2"])
    gui.add_setting(1, 'Packet Rate', 'int64', 'select', 'General',\
                    [200, 100, 50, 20, 10, 5, 2, 0])
    gui.add_setting(2, 'Play speed x', 'int64', 'select', 'General',\
                    [1, 2, 5, 10, 20])
    gui.add_graph('accel', 'm/s/s', 'line chart')
    print(gui.json)
