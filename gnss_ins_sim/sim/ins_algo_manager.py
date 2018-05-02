# -*- coding: utf-8 -*-
# Filename: ins_algo_manager.py

"""
Manage all possible algorithms in an INS solution.
Created on 2018-04-28
@author: dongxiaoguang
"""

class InsAlgoMgr(object):
    '''
    A class that manage all algorithms in an INS solution. Plann to support multiple algorithms
    in case users want to compare results of different algorithms in a simulation.
    '''
    def __init__(self, algo):
        '''
        algo: a user defined algorithm.
        '''
        self.algo = algo
        if self.algo is not None:
            self.__check_algo()
            self.algo.reset()

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
        try:
            n_in = len(self.algo.input)
            n_out = len(self.algo.output)
            # algorithm must have at least one input and one output
            if n_in < 1 or n_out < 1:
                raise ValueError
        except:
            print(self.algo.input)
            print(self.algo.output)
            raise ValueError('algorithm input or output is not a valid list or tuple.')

    def run_algo(self, input_data, keys=None):
        '''
        Run the algorithm with given input
        Args:
            input_data: algorithm input. It is a list of data defined in self.input. Each element
                in the list can be a scalar, a numpy array or a dict of the above two.
                Scalars and numpy arrays stays the same for each run of the algorithm.
                Data in dcits are chosen according to keys for each run of the algorithm.
            keys: a list of keys to index data in dicts in input. For example, we have multiple
                sets of gyro data: w={key0: set_of_data_#0, key1: set_of_data#1}.
                w is a element of input. keys should be [key0, key1]. For each run of the algo,
                gyro data is chosen accroding to the keys.
        Returns:
            data: a list of data defined in self.output. It is similiar to input and use the same
                key to index results of mutiple runs of the algorithm.
        '''
        nin = len(self.algo.input)
        nout = len(self.algo.output)
        if nin != len(input_data):
            raise ValueError('Required %s input, but provide %s.'% (nin, len(input_data)))
        #### call the algorithm
        set_of_input = []
        # run the algorithm once
        if keys is None:
            for ele in input_data:
                if isinstance(ele, dict):
                    set_of_input.append(ele[0])   # using only key=0. If no key=0, error
                else:
                    set_of_input.append(ele)
            self.algo.run(set_of_input)
            results = self.algo.get_results()
        # run the algorithm mutiple times according to keys
        else:
            results = []
            for i in range(nout):
                results.append({})
            for i in keys:
                # prepare the algorith input
                for ele in input_data:
                    if isinstance(ele, dict):
                        if i in ele:
                            set_of_input.append(ele[i])
                        else:
                            raise ValueError("set_of_input has keys %s, but you are requiring %s"\
                                             % (ele.keys(), i))
                    else:
                        set_of_input.append(ele)
                # run the algorithm once
                self.algo.run(set_of_input)
                # get algorithm output of this run
                this_results = self.algo.get_results()
                # add algorithm output of this run to results
                for j in range(nout):
                    results[j][i] = this_results[j]
                self.algo.reset()
        return results
