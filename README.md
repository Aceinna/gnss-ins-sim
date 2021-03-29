# GNSS-INS-SIM

**GNSS-INS-SIM** is an GNSS/INS simulation project, which generates reference trajectories, IMU sensor output, GPS output, odometer output and magnetometer output. Users choose/set up the sensor model, define the waypoints and provide algorithms, and **gnss-ins-sim** can generate required data for the algorithms, run the algorithms, plot simulation results, save simulations results, and generate a brief summary.

# Contents

------

* [Requirements](#requirements)
* [Demos](#demos)
* [Get started](#get-started)
    * [Step 1 Define the IMU model](#step-1-define-the-imu-model)
    * [Step 2 Create a motion profile](#step-2-create-a-motion-profile)
    * [Step 3 Create your algorithm](#step-3-create-your-algorithm)
    * [Step 4 Run the simulation](#step-4-run-the-simulation)
    * [Step 5 Show results](#step-5-show-results)
* [Acknowledgement](#Acknowledgement)

# Requirements

- Numpy ( version>1.10 )
- Matplotlib

# Demos

We provide the following demos to show how to use this tool:

| file name | description |
|---|---|
| demo_no_algo.py | A demo of generating data, saving generated data to files and plotting (2D/3D)interested data, no user specified algorithm. |
| demo_allan.py | A demo of Allan analysis of gyroscope and accelerometer data. The generated Allan deviation is shown in figures.|
| demo_free_integration.py | A demo of a simple strapdown system. The simulation runs for 1000 times. The statistics of the INS results of the 1000 simulations are generated.|
| demo_inclinometer_mahony.py | A demo of an dynamic inclinometer algorithm based on Mahony's theory. This demo shows how to generate error plot of interested data.|
| demo_aceinna_vg.py | A demo of DMU380 dynamic tilt algorithm. The algorithm is first compiled as a shared library. This demo shows how to call the shared library. This is the algorithm inside Aceinna's VG/MTLT products.|
| demo_aceinna_ins.py | A demo of DMU380 GNSS/INS fusion algorithm. The algorithm is first compiled as a shared library. This demo shows how to call the shared library. This is the algorithm inside Aceinna's INS products.|
| demo_multiple_algorithms.py | A demo of multiple algorithms in a simulation. This demo shows how to compare resutls of multiple algorithm.|
| demo_gen_data_from_files.py | This demo shows how to do simulation from logged data files.|

# Get started

## Step 1 Define the IMU model

### Step 1.1 Define the IMU error model

IMU error model can be specified in two ways:

#### Choose a built-in model

There are three built-in IMU models: 'low-accuracy', 'mid-accuracy' and 'high accuracy'.

#### Manually define the model

```python
imu_err = {
            # gyro bias, deg/hr
            'gyro_b': np.array([0.0, 0.0, 0.0]),
            # gyro angle random walk, deg/rt-hr
            'gyro_arw': np.array([0.25, 0.25, 0.25]),
            # gyro bias instability, deg/hr
            'gyro_b_stability': np.array([3.5, 3.5, 3.5]),
            # gyro bias instability correlation, sec.
            # set this to 'inf' to use a random walk model
            # set this to a positive real number to use a first-order Gauss-Markkov model
            'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
            # accelerometer bias, m/s^2
            'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
            # accelerometer velocity random walk, m/s/rt-hr
            'accel_vrw': np.array([0.03119, 0.03009, 0.04779]),
            # accelerometer bias instability, m/s^2
            'accel_b_stability': np.array([4.29e-5, 5.72e-5, 8.02e-5]),
            # accelerometer bias instability correlation, sec. Similar to gyro_b_corr
            'accel_b_corr': np.array([200.0, 200.0, 200.0]),
            # magnetometer noise std, uT
            'mag_std': np.array([0.2, 0.2, 0.2])
          }
```

### Step 1.2 Create an IMU object

```python
imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=False)
imu = imu_model.IMU(accuracy='low accuracy', axis=9, gps=True)
```

axis = 6 to generate only gyro and accelerometer data.

axis = 9 to generate magnetometer data besides gyro and accelerometer data.

gps = True to generate GPS data, gps = False not.

## Step 2 Create a motion profile

A motion profile specifies the initial states of the vehicle and motion command that drives the vehicle to move, as shown in the following table.

| Ini lat (deg) | ini lon (deg) | ini alt (m) | ini vx_body (m/s) | ini vy_body (m/s) | ini vz_body (m/s) | ini yaw (deg) | ini pitch (deg) | ini roll (deg) |
|---|---|---|---|---|---|---|---|---|
| 32 | 120 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| command type | yaw (deg) | pitch (deg) | roll (deg) | vx_body (m/s) | vy_body (m/s) | vz_body (m/s) | command duration (s) |	GPS visibility |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 200 | 1 |
| ... | ... | ... | ... | ... | ... | ... | ... | ... |

The initial position should be given in the LLA (latitude, longitude and altitude) form. The initial velocity is specified in the vehicle body frame. The initial attitude is represented by Euler angles of ZYX rotation sequence.

Motion commands define how the vehicle moves from its initial state. The simulation will generate true angular velocity, acceleration, magnetic field, position, velocity and attitude according to the commands. Combined with sensor error models, these true values are used to generate gyroscope, accelerometer, magnetometer and GPS output.
There is only one motion command in the above table. Indeed, you can add more motion commands to specify the attitude and velocity of the vehicle. You can also define GPS visibility of the vehicle for each command.

Five command types are supported, as listed below.

| Command type | Comment |
|---|---|
| 1 | Directly define the Euler angles change rate and body frame velocity change rate. The change rates are given by column 2~7. The units are deg/s and m/s/s. Column 8 gives how long the command will last. If you want to fully control execution time of each command by your own, you should always choose the motion type to be 1 |
| 2 | Define the absolute attitude and absolute velocity to reach. The target attitude and velocity are given by column 2~7. The units are deg and m/s. Column 8 defines the maximum time to execute the command. If actual executing time is less than max time, the remaining time will not be used and the next command will be executed immediately. If the command cannot be finished within max time, the next command will be executed after max time. |
| 3 | Define attitude change and velocity change. The attitude and velocity changes are given by column 2~7. The units are deg and m/s. Column 8 defines the maximum time to execute the command. |
| 4 | Define absolute attitude and velocity change. The absolute attitude and velocity change are given by column 2~7. The units are deg and m/s. Column 8 defines the maximum time to execute the command. |
| 5 | Define attitude change and absolute velocity. The attitude change and absolute velocity are given by column 2~7. The units are deg and m/s. Column 8 defines the maximum time to execute the command. |

### An example of motion profile

| Ini lat (deg) | ini lon (deg) | ini alt (m) | ini vx_body (m/s) | ini vy_body (m/s) | ini vz_body (m/s) | ini yaw (deg) | ini pitch (deg) | ini roll (deg) |
|---|---|---|---|---|---|---|---|---|
| 32 | 120 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| command type | yaw (deg) | pitch (deg) | roll (deg) | vx_body (m/s) | vy_body (m/s) | vz_body (m/s) | command duration (s) |	GPS visibility |
| 1 | 0| 0 | 0 | 0 | 0 | 0 | 200 | 1 |
| 5 | 0 | 45 | 0 | 10 | 0 | 0 | 250 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 10 | 1 |
| 3 | 90 | -45 | 0 | 0 | 0 | 0 | 25 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 50 | 1 |
| 3 | 180 | 0 | 0 | 0 | 0 | 0 | 25 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 50 | 1 |
| 3 | -180 | 0 | 0 | 0 | 0 | 0 | 25 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 50 | 1 |
| 3 | 180 | 0 | 0 | 0 | 0 | 0 | 25 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 50 | 1 |
| 3 | -180 | 0 | 0 | 0 | 0 | 0 | 25 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 50 | 1 |
| 3 | 180 | 0 | 0 | 0 | 0 | 0 | 25 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 50 | 1 |
| 3 | -180 | 0 | 0 | 0 | 0 | 0 | 25 | 1 |
| 1 | 0 | 0 | 0 | 0 | 0 | 0 | 50 | 1 |
| 5 | 0 | 0 | 0 | 0 | 0 | 0 | 10 | 1 |

The initial latitude, longitude and altitude of the vehicle are 32deg, 120deg and 0 meter, respectively. The initial velocity of the vehicle is 0. The initial Euler angles are 0deg pitch, 0deg roll and 0deg yaw, which means the vehicle is level and its x axis points to the north.

| command type | yaw (deg) | pitch (deg) | roll (deg) | vx_body (m/s) | vy_body (m/s) | vz_body (m/s) | command duration (s) |	GPS visibility |
|---|---|---|---|---|---|---|---|---|
| 1 | 0| 0 | 0 | 0 | 0 | 0 | 200 | 1 |

This command is of type 1. Command type1 directly gives Euler angle change rate and velocity change rate. In this case, they are zeros. That means keep the current state (being static) of the vehicle for 200sec. During this period, GPS is visible.

| command type | yaw (deg) | pitch (deg) | roll (deg) | vx_body (m/s) | vy_body (m/s) | vz_body (m/s) | command duration (s) |	GPS visibility |
|---|---|---|---|---|---|---|---|---|
| 5 | 0 | 45 | 0 | 10 | 0 | 0 | 250 | 1 |

This command is of type 5. Command type 5 defines attitude change and absolute velocity. In this case, the pitch angle will be increased by 45deg, and the velocity along the x axis of the body frame will be accelerated to 10m/s. This command should be executed within 250sec.

| command type | yaw (deg) | pitch (deg) | roll (deg) | vx_body (m/s) | vy_body (m/s) | vz_body (m/s) | command duration (s) |	GPS visibility |
|---|---|---|---|---|---|---|---|---|
| 3 | 90 | -45 | 0 | 0 | 0 | 0 | 25 | 1 |

This command is of type 3. Command type 3 defines attitude change and velocity change. In this case, the yaw angle will be increased by 90deg, which is a right turn. The pitch angle is decreased by 45deg. The velocity of the vehicle does not change. This command should be executed within 25sec.

The following figure shows the trajectory generated from the motion commands in the above table. The trajectory sections corresponding to the above three commands are marked by command types 1, 5 and 3.
<div align=center>
<img width="500"  src="https://github.com/Aceinna/gnss-ins-sim/blob/master/gnss_ins_sim/docs/images/motion_profile_demo.png"/>
</div>

## Step 3 Create your algorithm

```python
algo = allan_analysis.Allan() # an Allan analysis demo algorithm
```

An algorithm is an object of a Python class. It should at least include the following members:

### self.input and self.output

The member variable 'input' tells **gnss-ins-sim** what data the algorithm need. 'input' is a tuple or list of strings.

The member variable 'output' tells **gnss-ins-sim** what data the algorithm returns. 'output' is a tuple or list of strings.

Each string in 'input' and 'output' corresponds to a set of data supported by **gnss-ins-sim**. The following is a list of supported data by **gnss-ins-sim**.

| name | description |
|-|-|
| 'ref_frame' | Reference frame used as the navigation frame and the attitude reference. <br> 0: NED (default), with x axis pointing along geographic north, y axis pointing eastward, z axis pointing downward. Position will be expressed in LLA form, and the velocity of the vehicle relative to the ECEF frame will be expressed in local NED frame. <br> 1: a virtual inertial frame with constant g, x axis pointing along geographic or magnetic north, z axis pointing along g, y axis completing a right-handed coordinate system. Position and velocity will both be in the [x y z] form in this frame. <br> **Notice: For this virtual inertial frame, position is indeed the sum of the initial position in ecef and the relative position in the virutal inertial frame. Indeed, two vectors expressed in different frames should not be added. This is done in this way here just to preserve all useful information to generate .kml files. Keep this in mind if you use this result.|
| 'fs' | Sample frequency of IMU, units: Hz |
| 'fs_gps' | Sample frequency of GNSS, units: Hz |
| 'fs_mag' | Sample frequency of magnetometer, units: Hz |
| 'time' | Time series corresponds to IMU samples, units: sec. |
| 'gps_time' | Time series corresponds to GNSS samples, units: sec. |
| 'algo_time' | Time series corresponding to algorithm output, units: ['s']. If your algorithm output data rate is different from the input data rate, you should include 'algo_time' in the algorithm output. |
| 'gps_visibility' | Indicate if GPS is available. 1 means yes, and 0 means no. |
| 'ref_pos' | True position in the navigation frame. When users choose NED (ref_frame=0) as the navigation frame, positions will be given in the form of [Latitude, Longitude, Altitude], units: ['rad', 'rad', 'm']. When users choose the virtual inertial frame, positions (initial position + positions relative to the  origin of the frame) will be given in the form of [x, y, z], units:  ['m', 'm', 'm']. |
| 'ref_vel' | True velocity w.r.t the navigation/reference frame expressed in the NED frame, units: ['m/s', 'm/s', 'm/s']. |
| 'ref_att_euler' | True attitude (Euler angles, ZYX rotation sequency), units: ['rad', 'rad', 'rad'] |
| 'ref_att_quat' | True attitude (quaternions) |
| 'ref_gyro' | True angular velocity in the body frame, units: ['rad/s', 'rad/s', 'rad/s'] |
| 'ref_accel' | True acceleration in the body frame, units: ['m/s^2', 'm/s^2', 'm/s^2'] |
| 'ref_mag' | True geomagnetic field in the body frame, units: ['uT', 'uT', 'uT'] (only available when axis=9 in IMU object) |
| 'ref_gps' | True GPS position/velocity, ['rad', 'rad', 'm', 'm/s', 'm/s', 'm/s'] for NED (LLA), ['m', 'm', 'm', 'm/s', 'm/s', 'm/s'] for virtual inertial frame (xyz) (only available when gps=True in IMU object) |
| 'gyro' | Gyroscope measurements, 'ref_gyro' with errors |
| 'accel' | Accelerometer measurements, 'ref_accel' with errors |
| 'mag' | Magnetometer measurements, 'ref_mag' with errors |
| 'gps' | GPS measurements, 'ref_gps' with errors |
| 'ad_gyro' | Allan std of gyro, units: ['rad/s', 'rad/s', 'rad/s'] |
| 'ad_accel' | Allan std of accel, units: ['m/s2', 'm/s2', 'm/s2'] |
| 'pos' | Simulation position from algo, units: ['rad', 'rad', 'm'] for NED (LLA), ['m', 'm', 'm'] for virtual inertial frame (xyz). |
| 'vel' | Simulation velocity from algo, units: ['m/s', 'm/s', 'm/s'] |
| 'att_euler' | Simulation attitude (Euler, ZYX)  from algo, units: ['rad', 'rad', 'rad'] |
| 'att_quat' | Simulation attitude (quaternion)  from algo |
| 'wb' | Gyroscope bias estimation, units: ['rad/s', 'rad/s', 'rad/s'] |
| 'ab' | Accelerometer bias estimation, units: ['m/s^2', 'm/s^2', 'm/s^2'] |
| 'gyro_cal' | Calibrated gyro output, units: ['rad/s', 'rad/s', 'rad/s'] |
| 'accel_cal' | Calibrated acceleromter output, units: ['m/s^2', 'm/s^2', 'm/s^2'] |
| 'mag_cal' | Calibrated magnetometer output, units: ['uT', 'uT', 'uT'] |
| 'soft_iron' | 3x3 soft iron calibration matrix |
| 'hard_iron' | Hard iron calibration, units: ['uT', 'uT', 'uT'] |

### self.run(self, set_of_input)

This is the main procedure of the algorithm. **gnss-ins-sim** will call this procedure to run the algorithm.
'set_of_input' is a list of data that is consistent with self.input.
For example, if you set self.input = ['fs', 'accel', 'gyro'], you should get the corresponding data this way:

```python
  def run(self, set_of_input):
      # get input
      fs = set_of_input[0]
      accel = set_of_input[1]
      gyro = set_of_input[2]
```

### self.get_results(self)

**gnss-ins-sim** will call this procedure to get resutls from the algorithm. The return should be consistent with self.output.
For example, if you set self.output = ['allan_t', 'allan_std_accel', 'allan_std_gyro'], you should return the results this way:

```python
  def get_results(self):
      self.results = [tau,
                      np.array([avar_ax, avar_ay, avar_az]).T,
                      np.array([avar_wx, avar_wy, avar_wz]).T]
      return self.results
```

### self.reset(self)

**gnss-ins-sim** will call this procedure after run the algorithm. This is necessary when you want to run the algorithm more than one time and some states of the algorithm should be reinitialized.

## Step 4 Run the simulation

### step 4.1 Create the simulation object

```python
  sim = ins_sim.Sim(
        # sample rate of imu (gyro and accel), GPS and magnetometer
        [fs, fs_gps, fs_mag],
        # initial conditions and motion definition,
        # see IMU in ins_sim.py for details
        data_path+"//motion_def-90deg_turn.csv",
        # reference frame
        ref_frame=1,
        # the imu object created at step 1
        imu,
        # vehicle maneuver capability
        # [max accel, max angular accel, max angular rate]
        mode=np.array([1.0, 0.5, 2.0]),
        # specifies the vibration model for IMU
        env=None,
        #env=np.genfromtxt(data_path+'//vib_psd.csv', delimiter=',', skip_header=1),
        # the algorithm object created at step 2
        algorithm=algo)
```

**gnss-ins-sim** supports running multiple algorithms in one simulation. You can refer to demo_multiple_algorihtms.py for example.

There are three kinds of vibration models: random, sinusoidal and PSD.

| Acceleration vibration model | description |
|-|-|
| '[nx ny nz]g-random' | normal-distribution random vibration, rms is n*9.8 m/s^2 |
| '[nx ny nz]-random' | normal-distribution random vibration, rms is n m/s^2 |
| '[nx ny nz]g-mHz-sinusoidal' | sinusoidal vibration of m Hz, amplitude is n*9.8 m/s^2 |
| '[nx ny nz]-mHz-sinusoidal' | sinusoidal vibration of m Hz, amplitude is n m/s^2 |
| numpy array of size (n,4) | single-sided PSD. The four columns are [freqency, x, y, z], in units of (m/s^2)^2/Hz |
|

For the vibratoin model of the gyro, it is similar as that of the acceleration except that the unit is default to rad/s and 'd' is used to change the unit to be deg/s.

The following example sets random vibration to accel with RMS for x/y/z axis being 1, 2 and 3 m/s^2, respectively, and sets sinusoidal vibration to gyro with frequency being 0.5Hz and amplitude for x/y/z axis being 6, 5 and 4 deg/s, respectively.
```
    env = {'acc': '[1 2 3]-random',
           'gyro': '[6 5 4]d-0.5Hz-sinusoidal'}
```
### Step 4.2 Run the simulation

```python
sim.run()     # run for 1 time
sim.run(1)    # run for 1 time
sim.run(100)  # run for 100 times
```

## Step 5 Show results

```python
# generate a simulation summary,
# and save the summary and all data in directory './data'.
# You can specify the directory.
sim.results('./data/')

# generate a simulation summary, do not save any file
sim.results()

# plot interested data
sim.plot(['ref_pos', 'gyro'], opt={'ref_pos': '3d'})
```

# Acknowledgement

- Geomagnetic field model [https://github.com/cmweiss/geomag/tree/master/geomag](https://github.com/cmweiss/geomag/tree/master/geomag)
- MRepo [http://www.instk.org](http://www.instk.org/)