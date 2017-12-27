# -*- coding: utf-8 -*-
# Filename: allan.py

"""
Allan variance analysis.
Reference: O.J. Woodman. An introduction to inertial navigation.
Created on 2017-09-22
@author: dongxiaoguang
"""

# import
import math
import numpy as np

# global
VERSION = '1.0'

def allan_var(x, fs):
    """
    Allan variance.
    Args:
        x: n samples array
        fs: sample frequency, Hz
    Returns:
        avar: Allan variance
        tau: average time, s
    """
    ts = 1.0 / fs
    n = len(x)      # number of samples
    max_sample_per_bin = int(math.floor(n/9.0)) # max samples in one bin, at least 9 bins required
    tau = np.zeros((max_sample_per_bin,))
    avar = np.zeros((max_sample_per_bin,))
    # calculate Allan var
    for i in range(1, max_sample_per_bin+1):
        nbins = int(math.floor(n/i))                 # number of bins
        if nbins < 9:
            break
        nsample_per_bin = int(math.floor(n/nbins))   # number of samples per bin
        tmp = np.reshape(x[0: nbins*nsample_per_bin], (nbins, nsample_per_bin))
        tmp = np.mean(tmp, 1)
        diff = tmp[1::] - tmp[0:-1]
        avar[i-1] = 0.5 / (nbins - 1) * np.sum(diff * diff)
        tau[i-1] = i * ts
    return avar, tau
