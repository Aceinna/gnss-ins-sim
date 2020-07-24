# -*- coding: utf-8 -*-
# Fielname = sim_data_plot.py

"""
Simulation data plot.
Created on 2020-07-24
@author: dongxiaoguang
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from mpl_toolkits.mplot3d import Axes3D
from . import sim_data

def plot(x, y, key, plot3d, mpl_opt):
    '''
    Plot x and y.
    Args:
        x: x axis data.
        y: a sim_data object.
        key is a tuple or list of keys corresponding to y.data.
        plot3d: 1--3D plot, 2--3D plot projected on xy, xz and yz, otherwise--2D plot
        mpl_opt: strings to specify matplotlib properties.
    '''
    if isinstance(y.data, dict):
        plot_dict(x, y, key, plot3d, mpl_opt)
    else:
        plot_array(x, y, plot3d, mpl_opt)


def plot_dict(x, y, key, plot3d=0, mpl_opt=''):
    '''
    self.data is a dict. plot self.data according to key
    Args:
        x: x axis data Sim_data object.
        y: a sim_data object.
        key: a list of keys to specify what data in y.data is plotted.
            If key is an empty list, plot all keys in y.data
        plot3d: 1--3D plot, 2--3D plot projected on xy, xz and yz, otherwise--2D plot
        mpl_opt: strings to specify matplotlib properties.
    '''
    if key == []:
        key = y.data.keys()
    for i in key:
        y_data = y.data[i]
        # x axis
        if isinstance(x.data, dict):
            if not x.data:  # x.data could be an empty dict
                x_data = None
            else:
                x_data = x.data[i]
        else:
            x_data = x.data
        # unit conversion
        y_data = sim_data.convert_unit(y_data, y.units, y.output_units)
        # plot
        if plot3d == 1:
            plot3d_in_one_figure(y_data,\
                                    title=y.name + '_' + str(i),\
                                    grid=y.grid,\
                                    legend=y.legend,\
                                    mpl_opt=mpl_opt)
        elif plot3d == 2:
            plot3d_proj_in_one_figure(y_data,\
                                        title=y.name + '_' + str(i),\
                                        grid=y.grid,\
                                        legend=y.legend,\
                                        mpl_opt=mpl_opt)
        else:
            plot_in_one_figure(x_data, y_data,\
                            logx=y.logx, logy=y.logy,\
                            title=y.name + '_' + str(i),\
                            xlabel=x.name + ' (' + x.output_units[0] + ')',\
                            ylabel=y.name + ' (' + str(y.output_units) + ')',\
                            grid=y.grid,\
                            legend=y.legend,\
                            mpl_opt=mpl_opt)

def plot_array(x, y, plot3d=0, mpl_opt=''):
    '''
    self.data is a numpy.array
    Args:
        x: x axis data Sim_data object.
        y: a sim_data object.
        plot3d: 1--3D plot, 2--3D plot projected on xy, xz and yz, otherwise--2D plot
    '''
    # x axis
    if isinstance(x.data, dict):
        if not x.data:  # x.data could be an empty dict
            x_data = None
        else:
            # randomly choose data of any key
            for i in x.data:
                x_data = x.data[i]
                break
    else:
        x_data = x.data
    # y axis
    y_data = y.data
    # unit conversion
    y_data = sim_data.convert_unit(y_data, y.units, y.output_units)
    # plot
    if plot3d == 1:
        plot3d_in_one_figure(y_data,\
                                title=y.name,\
                                grid=y.grid,\
                                legend=y.legend,\
                                mpl_opt=mpl_opt)
    elif plot3d == 2:
        plot3d_proj_in_one_figure(y_data,\
                                    title=y.name,\
                                    grid=y.grid,\
                                    legend=y.legend,\
                                    mpl_opt=mpl_opt)
    else:
        plot_in_one_figure(x_data, y_data,\
                        logx=y.logx, logy=y.logy,\
                        xlabel=x.name + ' (' + x.output_units[0] + ')',\
                        ylabel=y.name + ' (' + str(y.output_units) + ')',\
                        title=y.name,\
                        grid=y.grid,\
                        legend=y.legend,\
                        mpl_opt=mpl_opt)

def plot_in_one_figure(x, y, logx=False, logy=False,\
                       title='Figure', xlabel=None, ylabel=None,\
                       grid='on', legend=None,\
                       mpl_opt=''):
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
    # if not x data, generate default x data
    if x is None:
        x = np.array(range(y.shape[0]))
    try:
        dim = y.ndim
        if dim == 1:
            if logx and logy:   # loglog
                line, = axis.loglog(x, y, mpl_opt)
            elif logx:          # semilogx
                line, = axis.semilogx(x, y, mpl_opt)
            elif logy:          # semilogy
                line, = axis.semilogy(x, y, mpl_opt)
            else:               # plot
                line, = axis.plot(x, y, mpl_opt)
            lines.append(line)
        elif dim == 2:
            for i in range(0, y.shape[1]):
                if logx and logy:   # loglog
                    line, = axis.loglog(x, y[:, i], mpl_opt)
                elif logx:          # semilogx
                    line, = axis.semilogx(x, y[:, i], mpl_opt)
                elif logy:          # semilogy
                    line, = axis.semilogy(x, y[:, i], mpl_opt)
                else:               # plot
                    line, = axis.plot(x, y[:, i], mpl_opt)
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

def plot3d_in_one_figure(y, title='Figure', grid='on', legend=None, mpl_opt=''):
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
                axis.plot(y[:, 0], y[:, 1], y[:, 2], mpl_opt)
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

def plot3d_proj_in_one_figure(y, title='Figure', grid='on', legend=None, mpl_opt=''):
    '''
    Create a figure and plot 3d trajectory in this figure.
    Args:
        y: y axis data, np.array of size (n,3)
        title: figure title
        gird: if this is not 'off', it will be changed to 'on'
        legend: tuple or list of strings of length 3.
    '''
    # plot data
    try:
        dim = y.ndim
        if dim == 2:    # y must be an numpy array of size (n,3), dim=2
            if y.shape[1] != 3:
                raise ValueError
            else:
                # check label
                if isinstance(legend, (tuple, list)):
                    n = len(legend)
                    if n != 3:
                        legend = ['x', 'y', 'z']
                else:
                    legend = ['x', 'y', 'z']
                # check grid
                show_grid = False
                if grid.lower() != 'off':
                    show_grid = True
                # create figure and axis
                # xy
                fig = plt.figure(title)
                axis = fig.add_subplot(131, aspect='equal')
                axis.plot(y[:, 0], y[:, 1], mpl_opt)
                axis.set_xlabel(legend[0])
                axis.set_ylabel(legend[1])
                axis.grid(show_grid)
                # xz
                axis = fig.add_subplot(132, aspect='equal')
                axis.plot(y[:, 0], y[:, 2], mpl_opt)
                axis.set_xlabel(legend[0])
                axis.set_ylabel(legend[2])
                axis.grid(show_grid)
                # yz
                axis = fig.add_subplot(133, aspect='equal')
                axis.plot(y[:, 1], y[:, 2], mpl_opt)
                axis.set_xlabel(legend[1])
                axis.set_ylabel(legend[2])
                axis.grid(show_grid)
        else:
            raise ValueError
    except:
        print(y.shape)
        raise ValueError('Check input data y.')

def show_plot():
    '''
    Show all plots
    '''
    plt.show()
