# GNSS-IMU-SIM

**GNSS-IMU-SIM** is an IMU simulation project, which generates reference trajectories, IMU sensor output, GPS output, odometer output and magnetometer output. Users choose/set up the sensor model, define the waypoints and provide algorithms, and **gnss-imu-sim** can generated required data for the algorithms, run the algorithms, plot simulation results, save simulations results, and generate a brief summary.

## Requirements

- Numpy ( version>1.10 )
- Matplotlib

## Step 1 Define the IMU model

### Step 1.1 Define the IMU error model

IMU error model can be specified in two ways:

#### Choose a built-in model

There are three built-in IMU models: 'low-accuracy', 'mid-accuracy' and 'high accuracy'.

#### Manually define the model
```
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
```
imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=False)
imu = imu_model.IMU(accuracy='low accuracy', axis=9, gps=True)
```
axis = 6 to generate only gyro and accelerometer data.

axis = 9 to generate magnetometer data besides gyro and accelerometer data.

gps = True to generate GPS data, gps = False not.

## Step 2 Create your algorithm

```
algo = allan_analysis.Allan() # an Allan analysis demo algorithm
```
An algorithm is an object of a Python class. It should at least include the following members:

### self.input

The member variable 'input' tells **gnss-imu-sim** what data the algorithm need. 'input' is a tuple or list of strings.
Each string in 'input' corresponds to a set of data generated and provided by **gnss-imu-sim**.

**Supported input**

| name | description |
|-|-|
| 'ref_frame' | Reference frame. 0 to use the NED frame as the navigation frame, 1 to use a virtual inertial frame. For the NED frame, its origin is located at the vehicle center, its x axis is in the local horizontal plane and points northwards, its y axis is in the local horizontal plane and points eastwards, and its z axis points downwards. For the virtual inertial frame, the Earth rotation will be ignored. This frame can be considered as an NED frame fixed at the initial position.|
| 'time' | Time series corresponds to IMU samples, units: sec. |
| 'fs' | Sample frequency of IMU, units: Hz |
| 'ref_pos' | True position in the navigation frame. When users choose NED (ref_frame=0) as the navigation frame, positions will be given in the form of [Latitude, Longitude, Altitude], units: ['rad', 'rad', 'm']. When users choose the virtual inertial frame, positions (initial position + positions relative to the  origin of the frame) will be given in the form of [x, y, z], units:  ['m', 'm', 'm']. |
| 'ref_vel' | True velocity w.r.t the navigation/reference frame expressed in the body frame, units: ['m/s', 'm/s', 'm/s']. |
| 'ref_att_euler' | True attitude (Euler angles, ZYX rotation sequency), units: ['rad', 'rad', 'rad'] |
| 'ref_att_quat' | True attitude (quaternions) |
| 'ref_gyro' | True angular velocity in the body frame, units: ['rad/s', 'rad/s', 'rad/s'] |
| 'ref_accel' | True acceleration in the body frame, units: ['m/s^2', 'm/s^2', 'm/s^2'] |
| 'ref_mag' | True geomagnetic field in the body frame, units: ['uT', 'uT', 'uT'] (only available when axis=9 in IMU object) |
| 'ref_gps' | True GPS position/velocity, ['rad', 'rad', 'm', 'm/s', 'm/s', 'm/s'] for NED (LLA), ['m', 'm', 'm', 'm/s', 'm/s', 'm/s'] for virtual inertial frame (xyz) (only available when gps=True in IMU object) |
| 'gps_time' | Time series correspond to GPS samples, units: sec |
| 'gyro' | Gyroscope measurements, 'ref_gyro' with errors |
| 'accel' | Accelerometer measurements, 'ref_accel' with errors |
| 'mag' | Magnetometer measurements, 'ref_mag' with errors |
| 'gps' | GPS measurements, 'ref_gps' with errors |

### self.output
The member variable 'output' tells **gnss-imu-sim** what data the algorithm returns. 'output' is a tuple or list of strings.
Each element in 'output' corresponds to a set of data that can be understood by **gnss-imu-sim**.

**Supported output**

| name | description |
|-|-|
| 'allan_t' | Time series of Allan analysis, units: ['s'] |
| 'allan_std_gyro' | Allan std of gyro, units: ['rad/s', 'rad/s', 'rad/s'] |
| 'allan_std_accel' | Allan std of accel, units: ['m/s2', 'm/s2', 'm/s2'] |
| 'pos' | Simulation position from algo, units: ['rad', 'rad', 'm'] for NED (LLA), ['m', 'm', 'm'] for virtual inertial frame (xyz). |
| 'vel' | Simulation velocity from algo, units: ['m/s', 'm/s', 'm/s'] |
| 'att_euler' | Simulation attitude (Euler, ZYX)  from algo, units: ['rad', 'rad', 'rad'] |
| 'att_quat' | Simulation attitude (quaternion)  from algo |
| 'wb' | Gyroscope bias estimation, units: ['rad/s', 'rad/s', 'rad/s'] |
| 'ab' | Accelerometer bias estimation, units: ['m/s^2', 'm/s^2', 'm/s^2'] |

### self.batch

| value | description |
| - | - |
| batch=True | Put all data from t0 to tf (default) |
| batch=False | Sequentially put data from t0 to tf (not supported yet) |

### self.run(self, set_of_input)

This is the main procedure of the algorithm. **gnss-imu-sim** will call this procedure to run the algorithm.
'set_of_input' is a list of data that is consistent with self.input.
For example, if you set self.input = ['fs', 'accel', 'gyro'], you should get the corresponding data this way:
```
  def run(self, set_of_input):
      # get input
      fs = set_of_input[0]
      accel = set_of_input[1]
      gyro = set_of_input[2]
```
### self.get_results(self)

**gnss-imu-sim** will call this procedure to get resutls from the algorithm. The return should be consistent with self.output.
For example, if you set self.output = ['allan_t', 'allan_std_accel', 'allan_std_gyro'], you should return the results this way:
```
  def get_results(self):
      self.results = [tau,
                      np.array([avar_ax, avar_ay, avar_az]).T,
                      np.array([avar_wx, avar_wy, avar_wz]).T]
      return self.results
```

### self.reset(self)

**gnss-imu-sim** will call this procedure after run the algorithm. This is necessary when you want to run the algorithm more than one time and some states of the algorithm should be reinitialized.

## Step 3 Run the simulation

### step 3.1 Create the simulation object:
```
  sim = imu_sim.Sim(
        # sample rate of imu (gyro and accel), GPS and magnetometer
        [fs, fs_gps, fs_mag],
        # the imu object created at step 1
        imu,
        # initial conditions and motion definition,
        # see IMU in imu_sim.py for details
        data_path+"//motion_def-90deg_turn.csv",
        # reference frame
        ref_frame=1,
        # vehicle maneuver capability
        # [max accel, max angular accel, max angular rate]
        mode=np.array([1.0, 0.5, 2.0]),
        # specifies the vibration model for IMU
        env=None,
        #env=np.genfromtxt(data_path+'//vib_psd.csv', delimiter=',', skip_header=1),
        # the algorithm object created at step 2
        algorithm=algo)
```
There are three kinds of vibration models:

| vibration model | description |
|-|-|
| 'ng-random' | normal-distribution random vibration, rms is n*9.8 m/s^2 |
| 'n-random' | normal-distribution random vibration, rms is n m/s^2 |
| 'ng-mHz-sinusoidal' | sinusoidal vibration of m Hz, amplitude is n*9.8 m/s^2 |
| 'n-mHz-sinusoidal' | sinusoidal vibration of m Hz, amplitude is n m/s^2 |
| numpy array of size (n,4) | single-sided PSD. [freqency, x, y, z], m^2/s^4/Hz |

### Step 3.2 Run the simulation:
```
sim.run()     # run for 1 time
sim.run(1)    # run for 1 time
sim.run(100)  # run for 100 times
```

### Step 3.3 Show results
```
# generate a simulation summary,
# and save the summary and all data in directory './data'.
# You can specify the directory.
sim.results('./data/')

# generate a simulation summary, do not save any file
sim.results()

# plot interested data
sim.plot(['ref_pos', 'gyro'], opt={'ref_pos': '3d'})
```

## Acknowledgement
- Geomagnetic field model [https://github.com/cmweiss/geomag/tree/master/geomag](https://github.com/cmweiss/geomag/tree/master/geomag)
- MRepo [http://www.instk.org](http://www.instk.org/)