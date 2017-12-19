# GNSS-IMU-SIM

GNSS-IMU-SIM is an IMU simulation project, which generates reference trajectories, IMU sensor output, 
GPS output, odometer output and magnetometer output.

The project contains the following modules.

### pathgen

pathgen is designed based on [MRepo](http://www.instk.org/). The attitude generation part is modified and a PD controller
is added to get smooth attitude and angular velocity.

See test_path_gen() for usage.

### filter

- A low-pass Butterworth filter design procedure. See test_filter_design() for usage.
- A general digital filter. See test_filter() for usage.

### fusion

- A Mahony filter based dynamic inclinometer algorithm. See test_inclinometer_ecf() for usage.
- An EKF based dynamic inclinometer algorithm with linear acceleration estimation. See test_inclinometer_kf() for usage.
- An EKF based dynamic inclinometer algorithm without linear acceleration estimation. See test_inclinometer_kf() for usage.

### allan

- See test_allan() for usage.

## other modules

- "attitude" contains DCM, quaternion and Euler angles representation and transformation between them.
- "buffer" is a ring buffer class.
- "geoparms" implements Earth radius model, gravity model, Earth rotation model, and a geomagnetic field model.
  The geomagnetic field model is from [https://github.com/cmweiss/geomag/tree/master/geomag](https://github.com/cmweiss/geomag/tree/master/geomag)
- "kml_gen" generates a .kml file from GPS trajectories.
- "google_earth" can load .kml file into Google Earth. This only works in windows.
- "psd" can generate times series from a given power spectral density.

### changelog

20170922:   Data generation for GPS, odometer, acc, gyro and mag is completed.

20171027:   Add maneuver capability to PathGen. Bug fixes.

20171102:   Add a vibration model to acc_gen.

20171110:   Add an odometer model to pathgen. Remove all file operations in pathgen.

20171115:   Run profilint on pathgen, improve computational efficiency.

20171129:   Add .kml file generatation function.

20171208    Add two versions of EKF based inclinometer algorithms.
            Add a function to launch Google Earth and visualize generated trajectories.

20171211    Bug fixes for allan.py in Python2;
            Bug fixes for template.kml.

20171212    OO design of filter.py and google_earth.py;
            Bug fixes for filter.py. Var tmp is not set correctly when failed to get data from buffer (len of tmp is m instead of 3).