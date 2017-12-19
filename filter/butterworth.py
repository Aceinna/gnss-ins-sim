# -*- coding: utf-8 -*-
# Filename: butterworth.py

"""
Butterworth low pass filter design.
Created on 2017-09-29
@author: dongxiaoguang
"""

# import
import math
import numpy as np

# globals
VERSION = '1.0'

def butter_lpf(fc, fs, n=3):
    '''
    Butterworth low pass filter design.
    Y/U = num/den.
    num = b[0] + b[1]z^(-1) + ... + b[n]z^(-n).
    den = a[0] + a[1]z^(-1) + ... + a[n]z^(-n), a[0]=1.
    Args:
        fc: cut-off frequency, Hz.
        fs: sample frequency, Hz.
        n: filter order.
    Returns:
        state: true means OK, false means wrong fs or fc
        num: numerator of the filter, n+1
        den: denominator of the filter, n+1
    '''
    num = np.zeros((n+1,))
    den = np.zeros((n+1,))
    num[0] = 1.0
    den[0] = 1.0
    # input check
    if fs <= 0.0:                   # sample frequency should be above 0
        return False, num, den
    fn = 0.5 * fs                   # Nyquist frequency
    if (fc <= 0.0) or (fc >= fn):   # cut-off frequency should be above 0 and below fn
        return False, num, den
    # calc lpf parameters
    c = 1.0 / math.tan(math.pi*fc/fs)   # to satisfy the cut-off frequency
    c_sqr = c * c
    # denominator
    n_now = 0                       # highest order of the current result
    tmp = np.zeros((3,))            # coefficients corresponding to k-th pole and its conjugate
    for k in range(1, n//2+1):
        phi = math.pi/2.0 + (2.0*k - 1.0)*math.pi/2.0/n
        cphi = math.cos(phi)
        tmp[0] = c_sqr + 1.0 - 2.0*c*cphi
        tmp[1] = -2.0*c_sqr + 2.0
        tmp[2] = c_sqr + 1.0 + 2.0*c*cphi
        for i in range(0, n_now+1): # use num to store den temporarily
            num[i] = den[i]
        tmp_den = poly_multiply(num, tmp, n_now, 2)
        n_now += 2
        for i in range(0, n_now+1):
            den[i] = tmp_den[i]
    if n%2 != 0:     # filter order is odd, there is a pole=-1 (analog)
        for i in range(0, n_now+1):
            num[i] = den[i]
        tmp[0] = c + 1.0
        tmp[1] = 1.0 - c
        tmp_den = poly_multiply(num, tmp, n_now, 1)
        n_now += 1
        for i in range(0, n_now+1):
            den[i] = tmp_den[i]
    # numerator
    num[0] = 1.0    # x^n
    num[n] = 1.0    # x^0
    for k in range(1, n//2+1):
        num[k] = num[k-1] / k * (n-k+1)
        num[n-k] = num[k]
    # normalization, gain = den[0], num /= gain, den /= gain
    for k in range(n, -1, -1):
        num[k] /= den[0]
        den[k] /= den[0]
    return True, num, den

def poly_multiply(a, b, na, nb):
    '''
    Poly multiplication.
    Args:
        a: (na+1), coefficients of a(0)*x^na + a(1) * x^(na-1) + ... + a(na)
        b: (nb+1), coefficients of b(0)*x^nb + b(1) * x^(na-1) + ... + b(nb)
        na: order of poly a
        nb: order of poly b
    Returns:
        c: (na+nb+1), coefficients of a*b = c(0)*x^(na+nb) + ... + a(na)*b(nb)
    '''
    nc = na + nb        # highest order of c=a*b
    c = np.zeros((nc+1,))
    for i in range(0, nc+1):
        c[i] = 0.0  # fill c with 0
        # c = a*b
        order_c = nc - i
        for j in range(0, na+1):
            order_a = na - j
            order_b = order_c - order_a
            if order_b < 0:
                continue
            k = nb - order_b
            if k < 0:
                continue
            c[i] += a[j]*b[k]
    return c
