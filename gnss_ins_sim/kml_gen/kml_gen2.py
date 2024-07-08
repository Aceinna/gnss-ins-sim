# -*- coding: utf-8 -*-
# Fielname = kml_gen2.py

"""
Generate .kml file contents from LLA
Created on 2024-07-08
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
      <Style id="track_spp">
         <IconStyle>
            <color>ff0000ff</color><colorMode>normal</colorMode>
            <scale>0.50</scale>
            <Icon><href>http://maps.google.com/mapfiles/kml/shapes/%s.png</href></Icon>
         </IconStyle>
         <LabelStyle>
            <color>ffffff00</color>
            <scale>1</scale>
         </LabelStyle>
      </Style>
      <Style id="track_spp_err">
         <IconStyle>
            <color>ff0000ff</color><colorMode>normal</colorMode>
            <scale>1</scale>
            <Icon><href>http://maps.google.com/mapfiles/kml/shapes/%s.png</href></Icon>
         </IconStyle>
         <LabelStyle>
            <color>ffffff00</color>
            <scale>1</scale>
         </LabelStyle>
      </Style>
      <Style id="track_rtd">
         <IconStyle>
            <color>ff00ffff</color><colorMode>normal</colorMode>
            <scale>0.50</scale>
            <Icon><href>http://maps.google.com/mapfiles/kml/shapes/%s.png</href></Icon>
         </IconStyle>
         <LabelStyle>
            <color>ffffff00</color>
            <scale>1</scale>
         </LabelStyle>
      </Style>
      <Style id="track_rtd_err">
         <IconStyle>
            <color>ff00ffff</color><colorMode>normal</colorMode>
            <scale>1</scale>
            <Icon><href>http://maps.google.com/mapfiles/kml/shapes/%s.png</href></Icon>
         </IconStyle>
         <LabelStyle>
            <color>ffffff00</color>
            <scale>1</scale>
         </LabelStyle>
      </Style>
      <Style id="track_float">
         <IconStyle>
            <color>ffff0000</color><colorMode>normal</colorMode>
            <scale>0.50</scale>
            <Icon><href>http://maps.google.com/mapfiles/kml/shapes/%s.png</href></Icon>
         </IconStyle>
         <LabelStyle>
            <color>ffffff00</color>
            <scale>1</scale>
         </LabelStyle>
      </Style>
      <Style id="track_float_err">
         <IconStyle>
            <color>ffff0000</color><colorMode>normal</colorMode>
            <scale>1</scale>
            <Icon><href>http://maps.google.com/mapfiles/kml/shapes/%s.png</href></Icon>
         </IconStyle>
         <LabelStyle>
            <color>ffffff00</color>
            <scale>1</scale>
         </LabelStyle>
      </Style>
      <Style id="track_fixed">
         <IconStyle>
            <color>ff00ff00</color><colorMode>normal</colorMode>
            <scale>0.50</scale>
            <Icon><href>http://maps.google.com/mapfiles/kml/shapes/%s.png</href></Icon>
         </IconStyle>
         <LabelStyle>
            <color>ffffff00</color>
            <scale>1</scale>
         </LabelStyle>
      </Style>
      <Style id="track_fixed_err">
         <IconStyle>
            <color>ff00ff00</color><colorMode>normal</colorMode>
            <scale>1</scale>
            <Icon><href>http://maps.google.com/mapfiles/kml/shapes/%s.png</href></Icon>
         </IconStyle>
         <LabelStyle>
            <color>ffffff00</color>
            <scale>1</scale>
         </LabelStyle>
      </Style>'''

kml_line = '''
      <Placemark>
        <name>Track</name>
        <Style>
          <LineStyle>
            <color>ffffffff</color>
          </LineStyle>
        </Style>
        <LineString>
          <coordinates>
%s
          </coordinates>
        </LineString>
      </Placemark>'''

kml_description = '''
      <description><![CDATA[
      <TABLE border="1" width="100 " Align="center">
      <TR ALIGN=RIGHT>
      <TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>%d</TD><TD>%.0f</TD><TD>%.3f</TD><TD>(s)</TD></TR>
      <TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>%f</TD><TD>%f</TD><TD>%f</TD><TD>(deg,m)</TD></TR>
      <TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>%.3f</TD><TD>%.3f</TD><TD>%.3f</TD><TD>(deg)</TD></TR>
      </TABLE>
      ]]></description>'''

kmlstr_body = '''
   <Placemark>
      <styleUrl>#%s</styleUrl>
      <Style> <IconStyle>  <heading>%f</heading> </IconStyle>  </Style>
      <Point>
         <coordinates>%.9f,%.9f,%f</coordinates>
      </Point>''' + kml_description + '''
   </Placemark>'''

kmlstr_body_start = '''
   <Placemark>
      <name>Start</name>
      <styleUrl>#%s</styleUrl>
      <Style> <IconStyle>  <heading>%f</heading> </IconStyle>  </Style>
      <Point>
         <coordinates>%.9f,%.9f,%f</coordinates>
      </Point>'''  + kml_description + '''
   </Placemark>'''

kmlstr_body_end = '''
   <Placemark>
      <name>End</name>
      <styleUrl>#%s</styleUrl>
      <Style> <IconStyle>  <heading>%f</heading> </IconStyle>  </Style>
      <Point>
         <coordinates>%.9f,%.9f,%f</coordinates>
      </Point>'''  + kml_description + '''
   </Placemark>'''

kmlstr_end = '''
</Document>
</kml>
'''
SPP = 1
RTD = 2
FIXED = 4
FLOAT = 5
INS = 6
map_err_limit = {SPP:200,\
                 RTD:200,\
                 FIXED:0.5,\
                 FLOAT:1,\
                 INS:2}
map_fix_type = {SPP:'track_spp',\
                RTD:'track_rtd',\
                FIXED:'track_fixed',\
                FLOAT:'track_float',\
                INS:'track_spp',\
                SPP+100:'track_spp_err',\
                RTD+100:'track_rtd_err',\
                FIXED+100:'track_fixed_err',\
                FLOAT+100:'track_float_err',\
                INS+100:'track_spp_err'}
map_track_icon = {'ins':'track',\
                  'gnss':'square',\
                  'gps':'square',\
                  'ref':'triangle'}

def kml_gen(data_dir, pos, rpy=None, time_stamp=None, name='pathgen', convert_to_lla=False,
            track_icon='track', dt=1):
    '''
    Generate .kml file contents from position data.
    Args:
        data_dir: directory to save the .kml files.
        pos: nx3 [lat, lon, alt] in rad and m, or [x, y, z] in m. Two additioanl columns can
             be supplied with fix_type and pos_err. For pos_err > 0.5m, the icon size is larger.
             fix_type is used to specify different icon color for different solution types.
        rpy: nx3, [roll, pitch, heading] in unit of deg. If nx1, there is only heading
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
        track_icon: icon for the track. see the definitions of "map_track_icon" for details.
        dt: Time interval in unit of second to resample the input data. Default is 1s.
            If it is 0, that means all the input LLA will be included in the generated kml file.
    Returns:
        None.
    '''
    #### get lla from pos
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
    #### write data
    ## write header
    kml_file = data_dir + '//' + name + '.kml'
    f = open(kml_file, 'w+')
    f.truncate()
    # write header
    track_icon = map_track_icon[track_icon]
    lines = (kmlstr_header)% (track_icon, track_icon, track_icon, track_icon,\
                              track_icon, track_icon, track_icon, track_icon)
    f.write(lines)
    ## write coordinates
    ndim = lla.ndim
    nsample = lla.shape[0]
    # check if there is roll and pitch
    if rpy is None:
        rpy = np.zeros((nsample, 3))
    elif rpy.ndim == 1: # an array for only heading
        rpy = np.array(rpy)
        rpy = np.reshape(rpy, (rpy.shape[0], 1))    # convert to nx1 matrix
    if rpy.shape[1] == 1: # only heading
        rpy = rpy * np.array([[0, 0, 1]])
    # start writing the line connecting each point
    idx = []    # this is used to select samples from the input to be written in kml
    if ndim == 1:
        tmplines = '            %s,%s,%s\n' % (lla[1], lla[0], lla[2])
    else:
        tmplines = ''
        dt = abs(dt)
        if dt == 0:
            # use all samples
            step = 1
            idx = range(0, nsample, step)
        else:
            # use only samples with time_stamp match dt.
            tmp_time_stamp = time_stamp['tow'] * (1.0/dt)
            rem = abs(np.round(tmp_time_stamp) - tmp_time_stamp) / (1.0/dt)
            idx = np.where(rem < 0.001)[0] # 0.001 is 1ms
        for i in idx:
            tmplines += '            %s,%s,%s\n' % (lla[i, 1], lla[i, 0], lla[i, 2])
    coordinate_lines = (kml_line) % (tmplines)
    f.write(coordinate_lines)
    # start writing each point
    if ndim == 1:
        coordinate_lines = (kmlstr_body)% (rpy[2], lla[1], lla[0], lla[2], 0)
        f.write(coordinate_lines)
    else:
        coordinate_lines = ''
        for i in idx:
            if i == idx[0]:
                tmp_kmlstr_body = kmlstr_body_start
            elif i == idx[-1]:
                tmp_kmlstr_body = kmlstr_body_end
            else:
                tmp_kmlstr_body = kmlstr_body
            if lla[i, 2] < 0:
                lla[i, 2] = 0
            # fix type
            fix_type = 1
            nCol = lla.shape[1]
            if nCol >= 4:
                fix_type = lla[i, 3]
                if nCol >= 5:
                    if lla[i, 4] > map_err_limit[fix_type]:
                        fix_type += 100
            track_type = map_fix_type[fix_type]
            # gen lines
            if time_stamp is None:
                this_week = 0
                this_tow = 0
            else:
                this_week = time_stamp['week'][i]
                this_tow = time_stamp['tow'][i]
            coordinate_lines = (tmp_kmlstr_body)% (track_type, rpy[i, 2],\
                                                   lla[i, 1], lla[i, 0], lla[i, 2],\
                                                   i, this_week, this_tow,\
                                                   lla[i, 1], lla[i, 0], lla[i, 2],\
                                                   rpy[i, 0], rpy[i, 1], rpy[i, 2])
            f.write(coordinate_lines)
    ## write end
    f.write(kmlstr_end)
    f.close()
    