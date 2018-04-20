# -*- coding: utf-8 -*-
# Fielname = sim_data.py

"""
Simulation data class.
Created on 2017-12-19
@author: dongxiaoguang
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from mpl_toolkits.mplot3d import Axes3D
from ..attitude import attitude

D2R = math.pi/180

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
