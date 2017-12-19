# -*- coding: utf-8 -*-
# Filename: filter.py

"""
General digital filter.
Created on 2017-10-09
@author: dongxiaoguang
"""

# import
import numpy as np
from buffer import buffer

# globals
VERSION = '1.0'

class Filter(object):
    '''
    n-dim general digitial filter.
    y/u = NUM/DEN. y is output, u is input.
    num = b[0] + b[1]z^(-1) + ... + b[n]z^(-n).
    den = a[0] + a[1]z^(-1) + ... + a[n]z^(-n), a[0]=1.
    '''
    def __init__(self, num, den):
        '''
        Initialize filter with NUM and DEN.
        Args:
            num: Numeriator of the filter. n-dim x (num_order+1).
            den: Denominator of the filter. n-dim x (den_order+1).
            num and den should have the same column number.
        Returns:
            Ture if OK, False if num and den are not valid.
        '''
        self.num = num
        self.den = den
        self.ln = num.shape[1]   # num order + 1
        self.ld = den.shape[1]   # den order + 1

    def flt(self, u, y):
        '''
        n-dim general digital filter.
        y/u = NUM/DEN. y is output, u is input.
        num = b[0] + b[1]z^(-1) + ... + b[n]z^(-n).
        den = a[0] + a[1]z^(-1) + ... + a[n]z^(-n), a[0]=1.
        Args:
            u: Buffer for input data.
            y: Buffer for output data. y.m = u.m
        Returns: True if OK, False if error.
        '''
        m = u.m
        nu = u.n
        ny = y.n
        # check if buffer depth is sufficient
        if (y.m != m) or (self.num.shape[0] != m) or (self.den.shape[0] != m) or\
           (nu < self.ln) or (ny < (self.ld-1)):
            return False
        ### begin filter
        tmp = np.zeros((m,))
        tmp_u = np.zeros((m,))
        tmp_y = np.zeros((m,))
        # tmp_u = a0*u[k] + ... + an*u[k-n], u[k] is the latest (get(0)) data in u
        for j in range(0, self.ln):
            [rtn, tmp] = u.get(j)
            if rtn is False:
                for i in range(0, m):
                    tmp[i] = 0.0
            for i in range(0, m):
                tmp_u[i] += self.num[i, j]*tmp[i]
        # tmp_y = b1*y[k-1] + ... + bn*y[k-n], y[k-1] is the latest data in y
        for j in range(0, self.ld-1):
            [rtn, tmp] = y.get(j)
            if rtn is False:
                for i in range(0, m):
                    tmp[i] = 0.0
            for i in range(0, m):
                tmp_y[i] += self.den[i, j+1]*tmp[i]
        # calculate filtered data
        for i in range(0, m):
            tmp[i] = (tmp_u[i] - tmp_y[i]) / self.den[i, 0]
        #### put filtered data into y
        y.put(tmp)
        return True
