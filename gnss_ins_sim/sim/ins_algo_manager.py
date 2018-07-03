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
        if keys is None:
            # prepare input to the algorithms
            for ele in input_data:
                if isinstance(ele, dict):
                    set_of_input.append(ele[0])   # using only key=0. If no key=0, error
                else:
                    set_of_input.append(ele)
            # run the algorithms
            for i in range(self.nalgo):
                # algo name will be used as a key to index results of this algo
                this_algo_name = self.get_algo_name(i)
                # run the algo with given input
                self.algo[i].reset()    # reset/initialize before run
                self.algo[i].run(set_of_input)
                # get results of this algo
                this_results = self.algo[i].get_results()
                # put results of this algo into final results
                for j in range(self.nout):
                    results[j][this_algo_name] = this_results[j]
        # run the algorithm mutiple times according to keys
        else:
            for key in keys:
                # prepare the algorith input of this run
                set_of_input = []
                for ele in input_data:
                    if isinstance(ele, dict):
                        if key in ele:
                            set_of_input.append(ele[key])
                        else:
                            raise ValueError("set_of_input has keys %s, but you are requiring %s"\
                                             % (ele.keys(), key))
                    else:
                        set_of_input.append(ele)
                # run the algorithms
                for i in range(self.nalgo):
                    # algo name will be used as a key to index results of this algo
                    this_algo_name = self.get_algo_name(i)
                    # run the algorithm once
                    self.algo[i].reset()    # reset/initialize before run
                    self.algo[i].run(set_of_input)
                    # get algorithm output of this run
                    this_results = self.algo[i].get_results()
                    # add algorithm output of this run to results
                    for j in range(self.nout):
                        results[j][this_algo_name+'_'+str(key)] = this_results[j]
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
        try:
            ## get input and output info of the first algorithm
            self.input = self.algo[0].input
            self.output = self.algo[0].output
            self.nin = len(self.input)
            self.nout = len(self.output)
            self.nalgo = len(self.algo)
            # algorithm must have at least one input and one output
            if self.nin < 1 or self.nout < 1:
                self.algo = None
                print(self.input)
                print(self.output)
                raise ValueError
            ## check if the other algorithms have the same input and output as the first algorithm
            for i in range(1, self.nalgo):
                if self.algo[i].input != self.input or self.algo[i].output != self.output:
                    self.algo = None
                    raise ValueError("Algorithm #%s has different intput or output."% i)
        except:
            raise ValueError('algorithm input or output is not a valid list or tuple.')
