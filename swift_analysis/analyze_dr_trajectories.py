# Copyright (C) 2018 Swift Navigation Inc.
# Contact: <dev@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

import sys
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

resultsdir = sys.argv[1]

# I don't know how to use dataframes so I do this the stupid way.
COL_TIME = 'time (sec)'
COL_DUT_LAT = 'pos_lat (deg)'
COL_DUT_LON = 'pos_lon (deg)'
COL_DUT_ALT = 'pos_alt (m)'
COL_REF_LAT = 'ref_pos_lat (deg)'
COL_REF_LON = 'ref_pos_lon (deg)'
COL_REF_ALT = 'ref_pos_alt (m)'
# Earth radius at 32deg Lat.
EARTHRAD = 6372168.
D2R = np.pi / 180.

times = []
hori_errors = []
vert_errors = []
atrack_errors = []
ctrack_errors = []
for f in glob.glob("{}/dr_*.csv".format(resultsdir)):
    df = pd.read_csv(f) 
    t = df[COL_TIME].values    
    dut_lat = df[COL_DUT_LAT].values
    dut_lon = df[COL_DUT_LON].values
    dut_alt = df[COL_DUT_ALT].values
    ref_lat = df[COL_REF_LAT].values
    ref_lon = df[COL_REF_LON].values
    ref_alt = df[COL_REF_ALT].values

    times.append(t)
    dut_y = EARTHRAD * D2R * dut_lat 
    dut_x = EARTHRAD * D2R * dut_lon * np.cos(D2R * 32.)
    ref_y = EARTHRAD * D2R * ref_lat
    ref_x = EARTHRAD * D2R * ref_lon * np.cos(D2R * 32.)
    vert_errors.append(np.abs(ref_alt - dut_alt))
    hori_errors.append(np.linalg.norm([ref_x - dut_x, ref_y - dut_y], axis=0))
    atrack_errors.append((ref_y - dut_y)[-1])
    ctrack_errors.append((ref_x - dut_x)[-1])

plt.subplot(221)
plt.scatter(np.ravel(times)[::10], np.ravel(hori_errors)[::10], color=(0., 0., 1., 0.05))
plt.title("Horizontal Error Magnitude")
plt.xlabel("t (s)")
plt.ylabel("error (m)")
plt.subplot(223)
plt.scatter(np.ravel(times)[::10], np.ravel(vert_errors)[::10], color=(0., 0., 1., 0.05))
plt.title("Vertical Error Magnitude")
plt.xlabel("t (s)")
plt.ylabel("error (m)")
plt.subplot(122)
plt.scatter(np.ravel(ctrack_errors), np.ravel(atrack_errors), color=(0., 0., 1., 1.0))
plt.title("Error Scatter")
plt.xlabel("Horizontal Cross-Track Error (m)")
plt.ylabel("Horizontal Along-Track Error (m)")
plt.gca().set_aspect('equal', 'datalim')

plt.show()
