# -*- coding: utf-8 -*-
# Filename: geoparams.py

"""
Geo parameters calculation. Include local Earth radius, Earth rotatoin rate,
local gravity.
Created on 2017-09-12
@author: dongxiaoguang
"""

# import
import math
import numpy as np
#import scipy.linalg

# global
VERSION = '1.0'
GM = 3.986004418e14                 # m3/(s2)
Re = 6378137                        # m
FLATTENING = 1/298.257223563        # Earth flattening, f = (a-b)/a
ECCENTRICITY = 0.0818191908426215   # Earth eccentricy, e2 = 2*f-f^2
E_SQR = ECCENTRICITY**2             # squared eccentricity
W_IE = 7292115e-11                  # Earth's rotation rate

def geo_param(pos):
    """
    Calculate local radius and gravity given the [Lat, Lon, Alt]
    Local radius include meridian radius rm and normal radius rn.
    Args:
        pos: [Lat, Lon, Alt], rad, m
    Returns:
        rm: meridian radius, m
        rn: normal radius, m
        g: gravity, m/s/s
        sl: sin(Lat)
        cl: cos(lat)
        w_ie: Earth's rotation rate w.r.t the inertial frame, rad/s
    """
    # some constants
    normal_gravity = 9.7803253359
    k = 0.00193185265241        # WGS-84 gravity model constant. For more details, refer to
                                # https://en.wikipedia.org/wiki/Gravity_of_Earth
    m = 0.00344978650684        # m = w*w*a*a*b/GM
    # calc
    sl = math.sin(pos[0])
    cl = math.cos(pos[0])
    sl_sqr = sl * sl
    h = pos[2]
    rm = (Re*(1 - E_SQR)) / (math.sqrt(1.0 - E_SQR*sl_sqr) * (1.0 - E_SQR*sl_sqr))
    rn = Re / (math.sqrt(1.0 - E_SQR*sl_sqr))
    g1 = normal_gravity * (1 + k*sl_sqr) / math.sqrt(1.0 - E_SQR*sl_sqr)
    g = g1 * (1.0 - (2.0/Re) * (1.0 + FLATTENING + m - 2.0*FLATTENING*sl_sqr)*h + 3.0*h*h/Re/Re)
    return rm, rn, g, sl, cl, W_IE

def earth_radius(lat):
    """
    Calculate Earth meridian radius and normal radius.
    Args:
        lat: Latitude, rad
    Returns:
        rm: meridian radius, m
        rn: normal radius, m
    """
    sl = math.sin(lat)
    sl_sqr = sl * sl
    rm = (Re*(1 - E_SQR)) / (math.sqrt(1.0 - E_SQR*sl_sqr) * (1.0 - E_SQR*sl_sqr))
    rn = Re / (math.sqrt(1.0 - E_SQR*sl_sqr))
    return rm, rn

def lla2ecef(lla):
    '''
    [Lat Lon Alt] position to xyz position
    Args:
        lla: [Lat, Lon, Alt], [rad, rad, meter], numpy array of size (3,)
    return:
        WGS-84 position, [x, y, z], [m, m, m], numpy array of size (3,)
    '''
    sl = math.sin(lla[0])
    cl = math.cos(lla[0])
    sl_sqr = sl * sl

    r = Re / math.sqrt(1.0 - E_SQR*sl_sqr)
    rho = (r + lla[2]) * cl
    x = rho * math.cos(lla[1])
    y = rho * math.sin(lla[1])
    z = (r*(1.0-E_SQR) + lla[2]) * sl
    return np.array([x, y, z])

def lla2ecef_batch(lla):
    '''
    [Lat Lon Alt] position to xyz position
    Args:
        lla: [Lat, Lon, Alt], [rad, rad, meter], numpy array of size (n,3)
    return:
        WGS-84 position, [x, y, z], [m, m, m], numpy array of size (n,3)
    '''
    # only one LLA
    if lla.ndim == 1:
        return lla2ecef(lla)
    # multiple LLA
    n = lla.shape[0]
    xyz = np.zeros((n, 3))
    for i in range(0, n):
        sl = math.sin(lla[i, 0])
        cl = math.cos(lla[i, 0])
        sl_sqr = sl * sl
        r = Re / math.sqrt(1.0 - E_SQR*sl_sqr)
        rho = (r + lla[i, 2]) * cl

        xyz[i, 0] = rho * math.cos(lla[i, 1])
        xyz[i, 1] = rho * math.sin(lla[i, 1])
        xyz[i, 2] = (r*(1.0-E_SQR) + lla[i, 2]) * sl
    return xyz

def ecef2lla(xyz):
    '''
    [x y z] position in ECEF to [Lat Lon Alt]
    Args:
        WGS-84 position, [x, y, z], [m, m, m], numpy array of size (3,)
    return:
        lla: [Lat, Lon, Alt], [rad, rad, meter], numpy array of size (3,)
    '''
    # longitude
    lon = math.atan2(xyz[1], xyz[0])
    # distance from the polar axis
    rho = math.sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1])
    # Spheroid properties
    b = (1.0 - FLATTENING) * Re             # Semiminor axis
    e2 = FLATTENING * (2.0 - FLATTENING)    # Square of (first) eccentricity
    ep2 = e2 / (1.0 - e2)                   # Square of second eccentricity
    # Bowring's formula for initial parametric (beta) and geodetic latitudes
    beta = math.atan2(xyz[2], (1.0 - FLATTENING) * rho)
    lat = math.atan2(xyz[2] + b*ep2*math.sin(beta)**3.0,\
                     rho - Re*e2*math.cos(beta)**3.0)
    # Fixed-point iteration with Bowring's formula
    # (typically converges within two or three iterations)
    beta_new = math.atan2((1.0 - FLATTENING)*math.sin(lat), math.cos(lat))
    count = 0
    while count < 5 and beta != beta_new:
        beta = beta_new
        lat = math.atan2(xyz[2] + b*ep2*math.sin(beta)**3.0,\
                         rho - Re*e2*math.cos(beta)**3.0)
        beta_new = math.atan2((1.0 - FLATTENING)*math.sin(lat), math.cos(lat))
        count += 1
    # Ellipsoidal height from final value for latitude
    slat = math.sin(lat)
    N = Re/math.sqrt(1.0-e2*slat*slat)
    alt = rho*math.cos(lat) + (xyz[2] + e2*N*slat)*slat - N
    return np.array([lat, lon, alt])

