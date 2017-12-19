# -*- coding: utf-8 -*-
# Filename: buffer.py

"""
Buffer class.
Created on 2017-09-29
@author: dongxiaoguang
"""

# import
import numpy as np

# globals
VERSION = '1.0'

class Buffer(object):
    '''
    An efficient ring buffer.
    '''
    def __init__(self, m, n):
        self.d = np.zeros((m, n))   # data storage, each column represents a set of data
        self.m = m                  # row number (data dimension)
        self.n = n                  # column number (max data can be stored)
        self.i = -1                 # index for data being put in the buffer, -1 means empty buffer
        self.full = 0               # 1 means buffer is full, 0 not
        self.num = 0                # number of data in the buffer

    def put(self, d):
        '''
        Put data into the buffer.
        Args:
            d: data to be put into the buffer, d should be an array of size (m,)
        Returns:
            True is OK, false if error.
        '''
        # check input
        if d.shape[0] != self.m:
            return False
        # update index
        self.i += 1
        if self.i == self.n:
            self.i = 0
        # update data count and buffer-full status
        if self.full == 0:          # buffer is not full yet
            self.num = self.i + 1
            if self.i == self.n-1:  # buffer become full
                self.full = 1
        for i in range(0, self.m):
            self.d[i, self.i] = d[i]
        return True

    def get(self, idx):
        '''
        Get data from the buffer according to idx.
        Args:
            idx: index of data to be read, idx=0 means the latest data,
                 idx=1 means data before the latest...
        Returns:
            status: True means OK, False means error.
            d: data read from the buffer.
        '''
        d = np.zeros((self.m,))
        if idx < 0 or idx >= self.n:       # index should be [0, n-1]
            return False, d
        # read data from buffer
        idx = self.i - idx
        if self.full == 1:      # buffer is full
            if idx < 0:
                idx += self.n
        else:                   # buffer is not full
            if idx < 0:         # index exceeds the storage range
                return False, d
        for i in range(0, self.m):
            d[i] = self.d[i, idx]
        return True, d

    def clear(self):
        '''
        Clear the buffer.
        Args:
        Returns:
        '''
        self.i = -1
        self.full = 0
        self.num = 0
    