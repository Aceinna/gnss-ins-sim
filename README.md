# GNSS-IMU-SIM

GNSS-IMU-SIM is an IMU simulation project, which generates reference trajectories, IMU sensor output, 
GPS output, odometer output and magnetometer output. Trajectory generation is designed based on [MRepo](http://www.instk.org/). Users can provide algorithms, and gnss-imu-sim can generated required data for the algorithms, run the algorithms, plot simulation results, save simulations results, and generate a brief summary.

## Requirements

- Numpy (version>1.10)
- Matplotlib

## Step 1 Define the IMU model

### Step 1.1 Define the IMU error model

IMU error model can be specified in two ways:

#### Choose a built-in model

There are three built-in IMU models: 'low-accuracy', 'mid-accuracy' and 'high accuracy'.

#### Manually define the model

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

### Step 1.2 Create an IMU object

imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=False)

imu = imu_model.IMU(accuracy='low accuracy', axis=9, gps=True)

axis = 6 to generate only gyro and accelerometer data.

axis = 9 to generate magnetometer data besides gyro and accelerometer data.

gps=True to generate GPS data, gps=False not.

## Step 2 Create you algorithm

algo = allan_analysis.Allan() # an Allan analysis demo algorithm

An algorithm is an object of a Python class. It should at least include the following members:

### self.input

The member variable 'input' tells gnss-imu-sim what data the algorithm need. 'input' is a tuple or list of strings.
Each element in 'input' corresponds to a set of data generated and provided by gnss-imu-sim.

Supported input:

- 'ref_frame': reference frame, 0 to use the NED frame as the navigation frame, 1 to use a virtual inertial frame.

- 'time': sample time, units: sec

- 'fs': Sample frequency of imu, units: Hz

- 'ref_pos': true pos in the navigation frame, units: ['rad', 'rad', 'm'] for NED (LLA), ['m', 'm', 'm'] for virtual inertial frame (xyz)

- 'ref_vel': true vel in the body frame, units: ['m/s', 'm/s', 'm/s']

- 'ref_att_euler': true attitude (Euler angles, ZYX rotation sequency), units: ['rad', 'rad', 'rad']

- 'ref_gyro': true angular velocity, units: ['rad/s', 'rad/s', 'rad/s']

- 'ref_accel': True accel, units: ['m/s^2', 'm/s^2', 'm/s^2']

- 'ref_mag': true magnetic field, units: ['uT', 'uT', 'uT'] (only available when axis=9 in IMU object)

- 'ref_gps': true GPS pos/vel, ['m', 'm', 'm', 'm/s', 'm/s', 'm/s'] for NED (LLA), ['m', 'm', 'm', 'm/s', 'm/s', 'm/s'] for virtual inertial frame (xyz) (only available when gps=True in IMU object)

- 'gps_time': GPS sample time, units: sec

- 'gyro': gyro measurements, 'ref_gyro' with errors

- 'accel': accel measurements, 'ref_accel' with errors

- 'mag': magnetometer measurements, 'ref_mag' with errors

- 'gps': GPS measurements, 'ref_gps' with errors

### self.output
The member variable 'output' tells gnss-imu-sim what data the algorithm retuns. 'output' is a tuple or list of strings.

Each element in 'output' corresponds to a set of data that can be understood by gnss-imu-sim.

Supported output:

- 'av_t': Allan var time, units: ['s']

- 'av_gyro': Allan var of gyro, units: ['rad/s', 'rad/s', 'rad/s']

- 'av_accel': Allan var of accel, units: ['m/s2', 'm/s2', 'm/s2']

- 'pos': simulation position from algo, units: ['rad', 'rad', 'm'] for NED (LLA), ['m', 'm', 'm'] for virtual inertial frame (xyz).

- 'vel': simulation velocity from algo, units: ['m/s', 'm/s', 'm/s']

- 'att_euler': simulation attitude (Euler, ZYX)  from algo, units: ['rad', 'rad', 'rad']

- 'att_quat': simulation attitude (quaternion)  from algo

- 'wb': gyro bias estimation, units: ['rad/s', 'rad/s', 'rad/s']

- 'ab': accel bias estimation, units: ['m/s^2', 'm/s^2', 'm/s^2']

### self.batch

batch=True: Put all data from t0 to tf if True (default)

batch=False: Sequentially put data from t0 to tf if False (not supported yet)

### self.run(self, set_of_input)

This should be the main procedure of the algorithm. gnss-imu-sim will call this procedure to run the algorithm.
'set_of_input' is a list of data that is consistent with self.input.

For example, if you set self.input = ['fs', 'accel', 'gyro'], you should get the corresponding data this way:

  def run(self, set_of_input):

          # get input
          fs = set_of_input[0]
          accel = set_of_input[1]
          gyro = set_of_input[2]

### self.get_results(self)

gnss-imu-sim will call this procedure to get resutls from the algorithm. The return should be consistent with self.output.

For example, if you set self.output = ['av_t', 'av_accel', 'av_gyro'], you should return the results this way:

  def get_results(self):

          self.results = [tau,
                          np.array([avar_ax, avar_ay, avar_az]).T,
                          np.array([avar_wx, avar_wy, avar_wz]).T]
          return self.results


### self.reset(self)

gnss-imu-sim will call this procedure after run the algorithm. This is necessary when you want to run the algorithm more than one time and some states of the algorithm should be reinitialized.

## Step 3 Run the simulation

### step 3.1 Create the simulation object:

  sim = imu_sim.Sim(

                    [fs, fs_gps, fs_mag],       # sample rate of imu (gyro and accel), GPS and magnetometer
                    imu,                        # the imu object created at step 1
                    data_path+"//motion_def-90deg_turn.csv",  # initial conditions and motion definition, see IMU in imu_sim.py for details
                    ref_frame=1,                # reference frame
                    mode=np.array([1.0, 0.5, 2.0]), # vehicle maneuver capability [max accel, max angular accel, max angular rate]
                    env=None,
                    #env=np.genfromtxt(data_path+'//vib_psd.csv', delimiter=',', skip_header=1),
                    algorithm=algo)             # the algorithm object created at step 2

  'env' specifies the vibration model for IMU, there are three vibration models:

  - 'ng-random': normal-distribution random vibration, rms is n*9.8 m/s^2
  - 'n-random': normal-distribution random vibration, rms is n m/s^2
  - 'ng-mHz-sinusoidal': sinusoidal vibration of m Hz, amplitude is n*9.8 m/s^2
  - 'n-mHz-sinusoidal': sinusoidal vibration of m Hz, amplitude is n m/s^2
  - numpy array of size (n,4): single-sided PSD. [freqency, x, y, z], m/s^2/rt-Hz

### Step 3.2 Run the simulation:
  - sim.run()     # run for 1 time
  - sim.run(1)    # run for 1 time
  - sim.run(100)  # run for 100 times

### Step 3.3 Show results
  - sim.results('./data/')  # generate a simulation summary, and save the summary and all data in directory './data'. You can specify the directory.
  - sim.results()           # generate a simulation summary, do not save any file
  - sim.plot(['ref_pos', 'gyro'], opt={'ref_pos': '3d'})  # plot interested data
