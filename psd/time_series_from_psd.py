# -*- coding: utf-8 -*-
# Fielname = time_series_from_psd.py

"""
Generate time series from a given PSD.
Created on 2017-11-02
@author: dongxiaoguang
"""

# import
import math
import numpy as np

# global
VERSION = '1.0'

def time_series_from_psd(sxx, freq, fs, n):
    """
    Generate 1-D time series from a given 1-D single-sided power spectal density.
    To save computational efforts, the max length of time series is 16384.
    ****If desired length>16384, time series will be repeated. Notice repeat will
    cause PSD of generated time series differs from the reference PSD.****
    Args:
        sxx: 1D single-sided PSD.
        freq: frequency responding to sxx.
        fs: samplling frequency.
        n: samples of the time series.
    Returns:
        status: true if sucess, false if error.
        x: time series
    """
    x = np.zeros((n,))
    ### check input sampling frequency
    if fs < 2.0*freq[-1] or fs < 0.0:
        return False, x
    ### check if interpolation is needed
    repeat_output = False
    N = n
    if n%2 != 0:                                # N should be even
        N = n+1
        repeat_output = True
    if N > 16384:                               # max data length is 16384
        N = 16384
        repeat_output = True
    ### convert psd to time series
    L = freq.shape[0]                           # num of samples in psd
    if L != N//2+1:                             # interp psd instead of output
        L = N//2 + 1
        freq_interp = np.linspace(0, fs/2.0, L)
        sxx = np.interp(freq_interp, freq, sxx)
    sxx[1:L-1] = 0.5 * sxx[1:L-1]               # single-sided psd amplitude to double-sided
    ax = np.sqrt(sxx*N*fs)                      # double-sided frequency spectrum amplitude
    phi = math.pi * np.random.randn(L)          # random phase
    xk = ax * np.exp(1j*phi)                    # single-sided frequency spectrum
    xk = np.hstack([xk, xk[-2:0:-1].conj()])    # double-sided frequency spectrum
    xm = np.fft.ifft(xk)                        # inverse fft
    x_tmp = xm.real                             # real part
    ### repeat x to output time series of desired lenght n
    if repeat_output is True:
        repeat_num = n // N
        repeat_remainder = n % N
        x = np.hstack([np.tile(x_tmp, (repeat_num,)), x_tmp[0:repeat_remainder]])
    else:
        x = x_tmp
    return True, x
