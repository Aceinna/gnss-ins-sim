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
    # how many tau will be calculated
    max_sample_per_bin = int(math.floor(n/9.0)) # max samples in one bin, at least 9 bins required
    if max_sample_per_bin * ts < 1: # not enough data
        return [], []
    multiplier = []
    nextpow10 = math.ceil(math.log10(max_sample_per_bin))
    scale = 0.1
    for i in range(0, nextpow10):
        scale *= 10
        for j in range(1, 10):
            tmp = int(j*scale)
            if tmp <= max_sample_per_bin:
                multiplier.append(tmp)
            else:
                break
    ntau = len(multiplier)
    tau = np.zeros((ntau,))
    avar = np.zeros((ntau,))
    # calculate Allan var
    for i in range(0, ntau):
        nsample_per_bin = multiplier[i]             # number of samples per bin
        nbins = int(math.floor(n/nsample_per_bin))  # number of bins
        if nbins < 9:
            break
        tmp = np.reshape(x[0: nbins*nsample_per_bin], (nbins, nsample_per_bin))
        tmp = np.mean(tmp, 1)
        diff = tmp[1::] - tmp[0:-1]
        avar[i] = 0.5 / (nbins - 1) * np.sum(diff * diff)
        tau[i] = nsample_per_bin * ts
    return avar, tau
