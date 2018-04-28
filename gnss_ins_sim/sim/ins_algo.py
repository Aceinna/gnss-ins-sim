# -*- coding: utf-8 -*-
# Filename: ins_algo.py

"""
A base algorithm class.
Created on 2018-04-28
@author: dongxiaoguang
"""

class InsAlgo(object):
    '''
    A base algorithm class. User defined algorithms should be a subclass of this.
    An algorithm should at least contain three variables to describe its input, output
    and behaviour: self.input, self.output and self.batch.
        self.input: a list to define what the algorithm need as input.
        self.output: a list to define what the algorithm outputs.
        self.batch: a bool value to define if the algorithm runs in batch mode or
            the algorithm should be called per time step.
    An algorithm should at least contain three procedures.
        self.initialize(): Initialize/reset the algorithm.
        self.run(input): Feed input to the algorithm and then run the algorithm.
        self.get_results(): Get algorithm results as defined in self.output.

    '''
    def __init__(self, ini_params=None):
        '''
        Args:
            ini_params: initialization parameters. User should define their own ini_params
            and implement the self.initialize procedure to handle ini_params.
        '''
        # algorithm description
        self.input = ['fs', 'gyro', 'accel']
        self.output = ['att_quat']
        self.batch = True

    def run(self, set_of_input):
        '''
        Run the algorithm.
        If batch is True, set_of_input contains all simulation data and self.run is called
        only once. If batch is False, set_of_input contains data of one time step and self.run
        is called at each time step.
        Args:
            set_of_input: a tuple or list consistent with self.input.
        Returns:
            a tuple or list consistent with self.input.
            If batch is true, this is None. If batch is false, this contains
            algorithm output of this time step.
        '''
        if self.batch:
            pass
        else:
            pass


    def get_results(self):
        '''
        return algorithm results as specified in self.output
        '''
        pass

    def initialize(self, ini_params):
        '''
        Reset the fusion process to uninitialized state.
        This is needed when you want to run the algorithm for multiple sets of data
        in a simulation.
        '''
        pass
