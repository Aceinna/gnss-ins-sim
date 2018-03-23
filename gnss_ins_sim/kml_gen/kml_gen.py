# -*- coding: utf-8 -*-
# Fielname = kml_gen.py

"""
Generate .kml file contents from LLA
Created on 2017-11-29
@author: dongxiaoguang
"""

import os
import math
import numpy as np
from ..geoparams import geoparams
from ..attitude import attitude

R2D = 180.0/math.pi

def kml_gen(pos, template_file='template.kml', name='pathgen', convert_to_lla=False):
    '''
    Generate .kml file contents from position data.
    Args:
        pos: nx3 [lat, lon, alt in rad and m, or [x, y, z] in m
        template_file: template kml file. The line 'REPALCE THIS WITH COORDINATES'
                       will be replaced by coordinates defined in pos.
        name: string name of this trajectory.
        convert_to_lla: true if position data are generated in a virtual inertial frame.
            xyz-form position data will be converted to [Lat Lon Alt] coordinates.
            See imu_sim and pathgen for details about the virtual inertial frame.
    Returns:
        kml_contents: string contents of kml file.
    '''
    # get lla from pos
    n = pos.shape[0]
    if convert_to_lla is False:
        lla = pos
        lla[:, 0] = lla[:, 0] * R2D
        lla[:, 1] = lla[:, 1] * R2D
    else:
        lla = np.zeros(pos.shape)
        # # xyz to lla
        # for i in range(0, n):
        #     lla[i, :] = geoparams.xyz2lla(pos[i, :])
        # # rad to deg
        # lla[:, 0] = lla[:, 0] * R2D
        # lla[:, 1] = lla[:, 1] * R2D
        # ned to lla
        lla[0, :] = geoparams.xyz2lla(pos[0, :])
        ecef_to_ned = attitude.rot_y(-math.pi/2.0-lla[0, 0]).dot(attitude.rot_z(lla[0, 1]))
        for i in range(1, n):
            ned_pos = pos[i, :]-pos[0, :]
            ecef_pos = pos[0, :] + ecef_to_ned.T.dot(ned_pos)
            lla[i, :] = geoparams.xyz2lla(ecef_pos)
        # rad to deg
        lla[:, 0] = lla[:, 0] * R2D
        lla[:, 1] = lla[:, 1] * R2D
    # gen kml according to data and template
    if template_file is None:
        template_file = os.path.join(os.path.dirname(__file__), 'template.kml')
    else:
        if not os.path.isabs(template_file):     # add by DXG
            template_file = os.path.join(os.path.dirname(__file__), template_file)
    fp = open(template_file)
    lines = ''
    for line in fp:
        if line == 'REPALCE THIS WITH COORDINATES\n':
            for i in range(0, n):
                if i == 0:
                    lines = lines + '\t\t\t\t'
                lines = lines + ('%f,%f,%f ' %(lla[i, 1], lla[i, 0], lla[i, 2]))
            lines = lines + '\n'
        elif 'PATHGEN' in line:
            line = line.replace('PATHGEN', name)
            lines = lines + line
        else:
            lines = lines + line
    # close template file
    fp.close()
    return lines
    