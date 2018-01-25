# -*- coding: utf-8 -*-
# Fielname = kml_gen.py

"""
Generate .kml file contents from LLA
Created on 2017-11-29
@author: dongxiaoguang
"""

import math
import numpy as np

R2D = 180.0/math.pi

def kml_gen_from_lla(lla, template_file):
    '''
    Generate .kml file contents from lla.
    Args:
        lla: nx3 [lat, lon, alt], rad, m.
        template_file: template kml file. The line 'REPALCE THIS WITH COORDINATES'
                       will be replaced by coordinates defined in lla.
    Returns:
        kml_contents: string contents of kml file.
    '''
    # data conversion
    lla[:, 0] = lla[:, 0]*R2D   # rad to deg
    lla[:, 1] = lla[:, 1]*R2D
    # gen kml according to data and template
    fp = open(template_file)
    lines = ''
    for line in fp:
        if line == 'REPALCE THIS WITH COORDINATES\n':
            n = lla.shape[0]
            for i in range(0, n):
                if i == 0:
                    lines = lines + '\t\t\t\t'
                lines = lines + ('%f,%f,%f ' %(lla[i, 1], lla[i, 0], lla[i, 2]))
            lines = lines + '\n'
        else:
            lines = lines + line
    # close template file
    fp.close()
    return lines
    