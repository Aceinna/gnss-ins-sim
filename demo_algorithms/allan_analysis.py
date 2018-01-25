# -*- coding: utf-8 -*-
# Filename: allan_analysis.py

"""
Allan variance analysis.
This is a demo algorithm for Sim.
Reference: O.J. Woodman. An introduction to inertial navigation.
Created on 2017-12-20
@author: dongxiaoguang
"""

# import
import numpy as np
from gnss_ins_sim.allan import allan

'''
The algorithm should be a Python class with three memeber functions.
1. __init__: config the interface of the algorithm.
    input: a tuple or list of variables this algorithm requires;
           There are two kinds of variables: constant through different calls of this algorithm and
           varying through different calls of this algorithm.
           These variables are defined in imu_sim.py as self.supported_in_constant and
           self.supported_in_varying. Users should check the two variables for input that
           imu_sim.py can provide to your algorithms.
    output: a tuple or list of variables this algorithm returns;
            The algorithm outpu that can be understood by the imu_sim.py is defined in imu_sim.py as
            self.supported_out. All algorithm outputs are assumed varying through different calls of
            the algorithm.
    batch: This defines in which way the "run" method of the algorithm should be called.
           If True, input variables sampled at all times should be provided to the algorithm;
           If False, input variables should be provided to the algorithm sample by sample.
           Only "batch is True" is supported for now
2. run: run the algorithm.
    set_of_input: a tuple or list of input variables consistent with self.input.
3. get_results: return results of the algorithm.
    self.results： a tuple or list of results consistent with self.output.
'''
class Allan(object):
    '''
    Allan var. A demo algorithm for Sim
    '''
    def __init__(self):
        '''
        algorithm description
        '''
        self.input = ['fs', 'accel', 'gyro']
        self.output = ['allan_t', 'ad_accel', 'ad_gyro']
        self.batch = True   # Put all data from t0 to tf if True (default)
                            # Sequentially put data from t0 to tf if False
        self.results = None # results must be a list or tuple of data as explained in self.out

    def run(self, set_of_input):
        '''
        main procedure of the algorithm
        Args:
            set_of_input is a tuple or list consistent with self.input
        '''
        # get input
        fs = set_of_input[0]
        accel = set_of_input[1]
        gyro = set_of_input[2]
        # calculate
        avar_ax, tau = allan.allan_var(accel[:, 0], fs)
        avar_ay = allan.allan_var(accel[:, 1], fs)[0]
        avar_az = allan.allan_var(accel[:, 2], fs)[0]
        avar_wx = allan.allan_var(gyro[:, 0], fs)[0]
        avar_wy = allan.allan_var(gyro[:, 1], fs)[0]
        avar_wz = allan.allan_var(gyro[:, 2], fs)[0]
        # generate results, must be a tuple or list consistent with self.output
        self.results = [tau,\
                        np.sqrt(np.array([avar_ax, avar_ay, avar_az]).T),\
                        np.sqrt(np.array([avar_wx, avar_wy, avar_wz]).T)]

    def get_results(self):
        '''
        return algorithm results as specified in self.output
        '''
        return self.results

    def reset(self):
        '''
        Rest algorithm to uninitialized state for next run.
        '''
        pass
