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

kmlstr_header = '''<?xml version = "1.0" encoding = "UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"
     xmlns:gx = "http://www.google.com/kml/ext/2.2" > 
<Document>
      <Style id="track">
         <IconStyle>
            <color>%s</color>
            <colorMode>normal</colorMode>
            <scale> 0.50</scale>
            <Icon>
               <href>http://maps.google.com/mapfiles/kml/shapes/track.png</href>
            </Icon>
         </IconStyle>
         <LabelStyle>
            <color>%s</color>
            <scale>7.000000e-01</scale>
         </LabelStyle>
      </Style>'''
kmlstr_body = '''
   <Placemark>
      <styleUrl>#track</styleUrl>
      <Style> <IconStyle>  <heading>%f</heading> </IconStyle>  </Style>
      <Point>
         <coordinates>%.9f,%.9f,%f</coordinates>
      </Point>
      <ExtendedData>
         <Data name="Index">
         <value><TD>%d</TD> <TD>%f</TD> <TD>%f</TD></value>
         </Data>
      </ExtendedData>
   </Placemark>'''
kmlstr_end = '''
</Document>
</kml>
'''

def kml_gen(data_dir, pos, heading=None, time_stamp=None, name='pathgen', convert_to_lla=False, 
            color='ffff0000', max_points=8000):
    '''
    Generate .kml file contents from position data.
    Args:
        data_dir: directory to save the .kml files.
        pos: nx3 [lat, lon, alt in rad and m, or [x, y, z] in m
        heading: in unit of deg
        time_stamp: 
            'week': week number
            'tow': time of week in seconds
        name: string name of this trajectory.
        convert_to_lla: true if position data are generated in a virtual inertial frame.
            xyz-form position data will be converted to [Lat Lon Alt] coordinates.
            See ins_sim and pathgen for details about the virtual inertial frame.
        color: Color and opacity (alpha) values are expressed in hexadecimal notation.
            The range of values for any one color is 0 to 255 (00 to ff). For alpha,
            00 is fully transparent and ff is fully opaque. The order of expression is
            aabbggrr, where aa=alpha (00 to ff); bb=blue (00 to ff); gg=green (00 to ff);
            rr=red (00 to ff). For example, if you want to apply a blue color with 50 percent
            opacity to an overlay, you would specify the following: <color>7fff0000</color>,
            where alpha=0x7f, blue=0xff, green=0x00, and red=0x00.
        max_points: the number of points in the generated kml file. The default value is 8000.
            If it is -1, that means only integer seconds are used. In this case, time_stamp is required.
    Returns:
        None.
    '''
    # get lla from pos
    pos = pos.copy()
    n = pos.shape[0]
    if convert_to_lla is False:
        lla = pos
        lla[:, 0] = lla[:, 0] * R2D
        lla[:, 1] = lla[:, 1] * R2D
    else:
        # ned to lla
        lla = np.zeros(pos.shape)
        # virtual inertial frame is defined by initial position
        lla[0, :] = geoparams.ecef2lla(pos[0, :])
        c_ne = attitude.ecef_to_ned(lla[0, 0], lla[0, 1])
        for i in range(1, n):
            ned_pos = pos[i, :]-pos[0, :]
            ecef_pos = pos[0, :] + c_ne.T.dot(ned_pos)
            lla[i, :] = geoparams.ecef2lla(ecef_pos)
        # rad to deg
        lla[:, 0] = lla[:, 0] * R2D
        lla[:, 1] = lla[:, 1] * R2D
        # save lla to .csv file. ** This is needed by the web version.
        header_line = 'Latitude (deg), Longitude (deg), Altitude (deg)'
        file_name = data_dir + '//' + name + '_LLA.csv'
        np.savetxt(file_name, lla, header=header_line, delimiter=',', comments='')
    # write data
    ndim = lla.ndim
    nsample = lla.shape[0]
    if heading is None:
        heading = np.zeros((nsample, 1))
    if ndim == 1:
        coordinate_lines = (kmlstr_body)% (heading[0], lla[1], lla[0], lla[2], 0)
    else:
        coordinate_lines = ''
        if max_points < 0:
            # use only integer seconds.
            idx = np.where(time_stamp['tow'] % 1 == 0)[0]
        else:
            step = int(math.ceil(nsample / max_points))
            idx = range(0, nsample, step)
        for i in idx:
            if lla[i][2] < 0:
                lla[i][2] = 0
            if time_stamp is None:
                coordinate_lines += (kmlstr_body)% (heading[i], lla[i][1], lla[i][0], lla[i][2],\
                                                    i, 0, 0)
            else:
                coordinate_lines += (kmlstr_body)% (heading[i], lla[i][1], lla[i][0], lla[i][2],\
                                                    i, time_stamp['week'][i], time_stamp['tow'][i])
    # write to file
    # gen kml according to data and template
    kml_file = data_dir + '//' + name + '.kml'
    f = open(kml_file, 'w+')
    f.truncate()
    # write header
    lines = (kmlstr_header)% (color, color)
    f.write(lines)
    # write data
    f.write(coordinate_lines)
    # write end
    f.write(kmlstr_end)
    f.close()
    