# -*- coding: utf-8 -*-
# Filename: ins_algo_manager.py

"""
Manage all possible algorithms in an INS solution.
Created on 2018-04-28
@author: dongxiaoguang
"""

import copy

class InsAlgoMgr(object):
    '''
    A class that manages all algorithms in an INS solution.
    '''
    def __init__(self, algo):
        '''
        algo: a user defined algorithm, or a list of that.
        '''
        if algo is None:
            self.algo = None
        else:
            # self.algo is a list of algorithms
            if isinstance(algo, list):
                self.algo = algo
            else:
                self.algo = [algo]
        self.input = []
        self.output = []
        self.nin = 0
        self.nout = 0
        self.nalgo = 0
        self.input_alloc = []
        self.output_alloc = []
        # check algorithm
        if self.algo is not None:
            self.__check_algo()

    def run_algo(self, input_data, keys=None):
        '''
        Run the algorithm with given input
        Args:
            input_data: algorithm input. It is a list of data defined in self.input. Each element
                in the list can be a scalar, a numpy array or a dict of the above two.
                Scalars and numpy arrays stays the same for each run of the algorithm.
                Data in dict are chosen according to keys for each run of the algorithm.
            keys: a list of keys to index data in dicts in input. For example, we have multiple
                sets of gyro data: w={key0: set_of_data_#0, key1: set_of_data#1}.
                w is a element of input. keys should be [key0, key1]. For each run of the algo,
                gyro data is chosen accroding to the keys.
        Returns:
            results: a list containing data defined in self.output.  Each output in results is
                a dict with keys 'algorithm_name' + '_' + 'simulation run'. For example:
                algo0_0, algo0_1, algo1_0, algo1_1, ......
        '''
        if len(input_data) != self.nin:
            raise ValueError('Required %s input, but provide %s.'% (self.nin, len(input_data)))
        #### call the algorithm
        set_of_input = []
        # run the algorithm once
        results = []
        for i in range(self.nout):
            results.append({})
        # generate keys to run simulation
        if keys is None:
            keys = [0]  # defauly, only run once
            # if there are multiple simulations runs
            for i in input_data:
                if isinstance(i, dict):
                    keys = list(i.keys())
                    break
        # run each algorithm
        for i in range(self.nalgo):
            # algo name will be used as a key to index results of this algo
            this_algo_name = self.get_algo_name(i)
            # run the algorithm for each simulation
            for key in keys:
                self.algo[i].reset()    # reset/initialize before each run
                # prepare input
                set_of_input = []
                for j in self.input_alloc[i]:  # j is the index of input of this algo in self.input
                    if isinstance(input_data[j], dict):
                        if key in input_data[j]:
                            set_of_input.append(input_data[j][key])
                        else:
                            raise ValueError("set_of_input has keys %s, but you are requiring %s"\
                                            % (input_data[j].keys(), key))
                    else:
                        set_of_input.append(input_data[j])
                self.algo[i].run(copy.deepcopy(set_of_input))   # deepcopy to avoid being changed
                # get algorithm output of this run
                this_results = self.algo[i].get_results()
                # add algorithm output of this run to results
                for j in range(len(self.output_alloc[i])):
                    results[self.output_alloc[i][j]][this_algo_name+'_'+str(key)] = this_results[j]
        return results

    def get_algo_name(self, i):
        '''
        get the name of the i-th algo
        Args:
            i: index of the algorithm
        Returns:
            name of the i-th algo, None if there is no i-th algo
        '''
        if self.algo is None:
            return None
        elif i >= self.nalgo:
            return None
        else:
            if hasattr(self.algo[i], 'name'):
                return self.algo[i].name
            else:
                return 'algo' + str(i)

    def __check_algo(self):
        '''
        Generate expressions to handle algorithm input and output.
        Args:
            algorithm: user specified algorithm class
        Returns:
            Raise ValueError if algorithm has no input or output;
            Raise ValueError if algorithm input and output have unsupported elements
            Raise TypeError if algorithm input or output is not a list or tuple
        '''
        # check if all algorithms has at least one input and at least one output
        try:
            for algo in self.algo:
                if len(algo.input) < 1 or len(algo.output) < 1:
                    raise ValueError
        except:
            raise ValueError('algorithm input or output is not a valid list or tuple.')
        # get input for all algoriths
        for algo in self.algo:
            self.input = list(set(self.input).union(set(algo.input)))
            self.output = list(set(self.output).union(set(algo.output)))
        # how to allocate input/output to each algorithm
        for algo in self.algo:
            idx = []
            for i in algo.input:
                idx.append(self.input.index(i))
            self.input_alloc.append(idx)
            idx = []
            for i in algo.output:
                idx.append(self.output.index(i))
            self.output_alloc.append(idx)
        self.nin = len(self.input)
        self.nout = len(self.output)
        self.nalgo = len(self.algo)
