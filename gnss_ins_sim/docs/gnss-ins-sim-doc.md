# **gnss-ins-sim**

------

**gnss-ins-sim** is an open-source GNSS + inertial navigation, sensor fusion simulator. It consists of a trajectory generatro, sensor (gyroscope, accelerometer, magnetometer and GPS) data generator and a fusion algorithm simulator. 

## Contents

------

* [1 Introduction](#1-introduction)
* [2 Preliminary](#2-preliminary)
    * [2.1 Cordinate systems](#21-coordinate-systems)
    * [2.2 Attitude representation](#22-attitude-representation)
* [3 Sensor model](#3-sensor-model)
    * [3.1 Gyroscope](#31-gyroscope)
    * [3.2 Accelerometer](#32-accelerometer)
    * [3.3 GPS](#33-gps)
    * [3.4 Magnetometer](#34-magnetometer)
* [4 Algorithm](#4-algorithm)
    * [4.1 Allan analysis](#41-allan-analysis)
    * [4.2 Strapdown inertial system](#42-strapdown-inertial-system)
* [5 Simulation design and implementation](#5-simulation-design-and-implementation)
* [6 References](#6-references)

## 1 Introduction

------
This doc is for better understanding and usage of **gnss-ins_sim**.

## 2 Preliminary

------

### 2.1 Coordinate systems

This section gives the definitions of the coordinate systems used in **gnss-ins-sim**.

#### 2.1.1 ECEF frame

The ECEF frame is an Earth-Centered, Earth-Fixed reference system. Its origin is the Earth’s CoM. Its z axis points to the true north. Its x axis points to the intersection of the Greenwich meridian and the equator, and its y axis complete the right-handed coordinate frame. We use the WGS-84 reference ellipsoid in the simulation since it is used in GPS.
Obviously, the ECEF is not an inertial reference system.

<div align=center>
<img width="500"  src="https://github.com/Aceinna/gnss-ins-sim/blob/master/gnss_ins_sim/docs/images/ECEF.png"/>
</div>

#### 2.1.2 NED frame

For a device near or on the surface of the Earth, it is more convenient to describe its motion (especially rotations) in the NED (North-East-Downward) or ENU (East-North-Upward) frame. We choose the NED frame in gnss-ins-imu.
Its origin is fixed at the device. Its x axis is in the local horizontal plane and point to the true north. Its z axis is perpendicular to the local horizontal plane and point downwards, and its y axis completes the right-handed coordinate frame.

<div align=center>
<img width="400"  src="https://github.com/Aceinna/gnss-ins-sim/blob/master/gnss_ins_sim/docs/images/NED.png"/>
</div>

#### 2.1.3 Virtual Inertial frame

For local-area or short-time applications, it may be more convenient to choose a virtual inertial frame as the reference frame especially when the sensor is of low accuracy. In these cases, the shape and rotation of the Earth will be ignored.
The virtual inertial frame can be considered as an NED frame fixed at a known point.

#### 2.1.4 Body frame

The body coordinate system is fixed at the device. Its origin is located at the center of device. Its x axis points forward, lying in the symmetric plane of the device. Its y axis points to the right side of the device. Its z axis points downward to complete the right-hand coordinate system.

## 2.2 Attitude representation

Two representations of the device attitude are adopted in **gnss-ins-sim**:

* quaternions (scalar first);
* Euler angles corresponding to the ZYX rotation sequence.

## 3 Sensor model

------

### 3.1 Gyroscope

The mathematical model of a gyro in **gnss-ins-sim** is
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\omega_m=\omega&plus;b_{\omega}&plus;n_{\omega}" title="\omega_m=\omega+b_{\omega}+n_{\omega}" />
</div>

<div align=center>
<img src="https://latex.codecogs.com/gif.latex?b_{\omega}=b_{\omega&space;c}&plus;b_{\omega&space;v}" title="b_{\omega}=b_{\omega c}+b_{\omega v}" />
</div>

where <img src="https://latex.codecogs.com/gif.latex?\omega" title="\omega" /> is the true angular velocity, 
the subscript "m" means measurement, 
<img src="https://latex.codecogs.com/gif.latex?n_\omega" title="n_\omega" /> is the white noise of the gyroscope output,
and <img src="https://latex.codecogs.com/gif.latex?b_\omega" title="b_\omega" /> is the gyro bias which includes a constant bias <img src="https://latex.codecogs.com/gif.latex?b_{\omega&space;c}" title="b_{\omega c}" /> and a varying bias <img src="https://latex.codecogs.com/gif.latex?b_{\omega&space;v}" title="b_{\omega v}" />.

#### 3.1.1 Constant Bias

For a rate gyro, its constant bias is the average output from the gyroscope when it is not undergoing any rotation. When the rate gyro output is integrated to get angles, the constant bias causes the angles to grow linearly with time.

#### 3.1.2 White Noise / Angle Random Walk

Besides the bias, the gyro measurements are also perturbed by a white noise sequence, which can be modelled as a sequence of zero-mean uncorrelated random variables. All the random variables are iid (independent and identically distributed) and have the same finite variance <img src="https://latex.codecogs.com/gif.latex?{\sigma}^2" title="{\sigma}^2" />.

When integrating the angular rate measurements from the gyro, the white noise introduces a zero-mean random walk error into the angles, whose standard deviation is:
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\sigma&space;_\theta(t)=\sigma\cdot\sqrt{dt\cdot&space;t}" title="\sigma _\theta(t)=\sigma\cdot\sqrt{dt\cdot t}" />
</div>

where <img src="https://latex.codecogs.com/gif.latex?dt" title="dt" /> is the sample interval, and <img src="https://latex.codecogs.com/gif.latex?t" title="t" /> is the integration duration. For <img src="https://latex.codecogs.com/gif.latex?n" title="n" /> samples, <img src="https://latex.codecogs.com/gif.latex?t=n\cdot&space;dt" title="t=n\cdot dt" /> .

Since we usually integrate angular rate to get angles, it is common to specify the white noise using an angle random walk (ARW):
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\textup{ARW}=\sigma&space;_\theta(1)" title="\textup{ARW}=\sigma _\theta(1)" />
</div>

The units of ARW are usually <img src="https://latex.codecogs.com/gif.latex?^\circ/\sqrt{h}" title="^\circ/\sqrt{h}" /> .

#### 3.1.3 Bias Stability

The bias stability describes how the bias of a gyro may change over a specified period of time. It reflects the best possible estimate of the gyro bias.
There are two models for the varying bias:
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\dot{b}_{\omega&space;v}=n_{\omega&space;v}" title="\dot{b}_{\omega v}=n_{\omega v}" />
</div>

<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\dot{b}_{\omega&space;v}=-\frac{1}{\tau}b_{\omega&space;v}&plus;n_{\omega&space;v}" title="\dot{b}_{\omega v}=\frac{1}{\tau}b_{\omega v}+n_{\omega v}" />
</div>
The first is a random walk model, and the second is a first-order Gauss-Markov model.
In reality, bias fluctuations do not really behave as a random walk or a first-order Gauss-Markov model, and therefore the random walk model and the Gauss-Markov model are only good approximations to the true process of bias fluctuations.

#### 3.1.4 Calibration Errors

Calibration errors refer to errors in the scale factors, alignments, and linearities of the gyros. Such errors tend to produce bias errors that are only observed when the device is undergoing rotations. Since theses errors can be accurately calibrated during factory calibration, we ignore such erros in **gnss-ins-sim**.

### 3.2 Accelerometer

The mathematical model of an accelerometer in **gnss-ins-sim** is
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?a_m=a&plus;b_a&plus;n_a" title="a_m=a+b_a+n_a" />
</div>

where, <img src="https://latex.codecogs.com/gif.latex?a_m" title="a_m" /> is the accelerometer measurement, <img src="https://latex.codecogs.com/gif.latex?a" title="a" /> is the true  acceleration which inludes linear acceleration and gravitational acceleration, <img src="https://latex.codecogs.com/gif.latex?b_a" title="b_a" /> is the accelerometer bias which includes a constant bias and a varying bias, and <img src="https://latex.codecogs.com/gif.latex?n_a" title="n_a" /> is the accelerometer noise.

#### 3.2.1 Constant Bias

If the accelerometer measurements are used to determine the tilt angle of the device w.r.t the local horizontal plane, a constant bias error of <img src="https://latex.codecogs.com/gif.latex?\epsilon" /> will cause errors in pitch and roll angles. When double integrated to get positions, a constant bias error causes an error in position which grows quadratically with time. The accumulated error in position is
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?r(t)=\frac{\epsilon}{2}&space;\cdot&space;t^2" />
</div>

#### 3.2.2 White Noise / Velocity Random Walk

Similar to a gyro ARW, the white noise sequence in the accelerometer measurements introduce a velocity random walk (VRW) that is usually specified with units of <img src="https://latex.codecogs.com/gif.latex?m/s/\sqrt{h}" />.

#### 3.2.3 Bias Stability

Similar to a gyro, the bias stability of an accelerometer can also be modelled by a random walk model or a frist-order Gauss-Markov model.

#### 3.2.4 Calibration Errors

Calibration errors (errors in the scale factors, alignments and linearities of the accelerometers) tend to produce bias errors that are only observed when the device is undergoing accelerations (including gravitational acceleration).

### 3.3 GPS

A GPS receiver can provide device position and velocity in the ECEF frame (WGS-84). Its accuracy is usually specified as horizontal accuracy and vertical accuracy.

There are various statistical methods of describing specifications for GPS receivers.

### 3.3.1 2D accuracy

| Accuracy measures | Formula | Probability | Definition |
|------------|--------|-----------|-----------|
| CEP | <img src="https://latex.codecogs.com/gif.latex?0.56\sigma_x&plus;0.62\sigma_y" /> <img src="https://latex.codecogs.com/gif.latex?\textup{(Accurate&space;when&space;}&space;\sigma_y/\sigma_x>0.3)" /> | 50% | The radius of circle centered at the true position, containing the position estimate with probability of 50%. |
| DRMS | <img src="https://latex.codecogs.com/gif.latex?\sqrt{\sigma_x^2&plus;\sigma_y^2}" /> | 65% | The square root of the average of the squared horizontal position errors. |
| 2DRMS | <img src="https://latex.codecogs.com/gif.latex?2\sqrt{\sigma_x^2&plus;\sigma_y^2}" /> | 95% | Twice the DRMS of the horizontal position errors. |

### 3.3.2 3D accuracy

| Accuracy measures | Formula | Probability | Definition |
|------------|--------|-----------|-----------|
| <div align=center> SEP <br>(Spherical Error Probable) </div>| <img src="https://latex.codecogs.com/gif.latex?0.51(\sigma_x&plus;\sigam_y&plus;\sigma_z)" /> | 50% | The radius of sphere centered at the true position, containing the position estimate in 3D with probability of 50%. |
| <div align=center> DRMS </div> | <img src="https://latex.codecogs.com/gif.latex?\sqrt{\sigma_x^2&plus;\sigma_y^2&plus;\sigma_z^2}" /> | 61% | The radius of sphere centered at the true position, containing the position estimate in 3D with probability of 61%. |
| <div align=center> 2DRMS </div>| <img src="https://latex.codecogs.com/gif.latex?0.833(\sigma_x&plus;\sigam_y&plus;\sigma_z)" /> | 90% | The radius of sphere centered at the true position, containing the position estimate in 3D with probability of 90%. |

We can see that GPS error characteristis are complicaetd<sup>[1]</sup>. To simplify the simulation, we use <img src="https://latex.codecogs.com/gif.latex?[\sigma_{NorthPos},\sigma_{EastPos},\sigma_{DownPos},\sigma_{NorthVel},\sigma_{EastVel},\sigma_{DownVel}]"> to specify the position and velocity error statistics of a GPS receiver.

### 3.4 Magnetometer

Unlike IMU, when we talk about the errors of a magnetometer, we also include environmental inteferences around the sensor, besides inherent errors of the magnetometer.
There are two kinds of interferences: hard iron error and soft iron error. The former introduces in the sensor output a constant offset from the true value, which has the same effect as the sensor bias. The latter is caused by soft iron materials around the sensor and distort the environmental magnetic field to be measured.
The model of the magnetometer can be given by
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?m_m=C_s_i\cdot&space;m&plus;b_m&plus;n_m">
</div>

where, <img src="https://latex.codecogs.com/gif.latex?m"> is the true magnetic field, <img src="https://latex.codecogs.com/gif.latex?m_m"> is the sensor output, <img src="https://latex.codecogs.com/gif.latex?C_s_i"> is the soft iron matrix, <img src="https://latex.codecogs.com/gif.latex?b_m"> is the hard iron, <img src="https://latex.codecogs.com/gif.latex?n_m"> is the white noise. Indeed, the hard iron is composed of "true" hard iron and sensor bias, and the soft iron represents the combined effects of "true" soft iron, misalignment, scale factor error and installation error.

If we take measurements of an ideal magnetometer while rotating the sensor arbitrarily and make a 3D plot of the measurement, all data points will be located on the surface of a sphere. The center of the spehre will be at [0, 0, 0] and the radius of the sphere is the amplitude of the local geomagnetic field. The hard iron will shift the sphere center and the linear soft iron will distor the sphere into an ellipsoid.
<div align=center>
<img width="500"  src="https://github.com/Aceinna/gnss-ins-sim/blob/master/gnss_ins_sim/docs/images/mag_si_hi.png"/>
</div>

## 4 Algorithm

------

## 4.1 Allan analysis

In the previous sections we described a number of noise processes which arise in accelerometer and gyroscope signals. In this section we describe a technique known as Allan Variance, which can be used to detect and determine the properties of such processes.
The Allan Variance of a signal is a function of averaging time. For an averaging time t, the Allan Variance is computed as follows<sup>[2]</sup>:

* 1\. Take a long sequence of data and divide it into bins of length <img src="https://latex.codecogs.com/gif.latex?t">. There must be enough data for at least 9 bins (otherwise the results obtained begin to lose their significance).
* 2\. Average the data in each bin to obtain a list of averages <img src="https://latex.codecogs.com/gif.latex?(a(t)_1,a(t)_2,...,at(t)_n)">, where <img src="https://latex.codecogs.com/gif.latex?n"> is the
number of bins.
* 3\. The Allan Variance is then given by
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\textup{AVAR}(t)=\frac{1}{2(n-1)}\cdot&space;\sum_{i}^{&space;}(a(t)_{i&plus;1}-a(t)_i)^2">
</div>

To determine the characteristics of the underlying noise processes, Allan Deviation
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\textup{AD}(t)=\sqrt{\textup{AVAR}(t)}">
</div>
is plotted as a function of t on a log-log scale. Different types of random process cause slopes with
different gradients to appear on the plot.
<div align=center>
<img width="800"  src="https://github.com/Aceinna/gnss-ins-sim/blob/master/gnss_ins_sim/docs/images/allan_deviation.png"/>
</div>

## 4.2 Strapdown inertial system

### 4.2.1 Rotation

We adotp two attitude representations in **gnss-ins-sim**: Euler angles and quaternions. Since Euler angles are more straightforward, they are more suitable for input and output. In motion definitions files, Euler angles are used to specify device attitude.

In **gnss-ins-sim**, Euler angles correspon to the ZYX rotation sequence (yaw, pitch and roll), denoted by <img src="https://latex.codecogs.com/gif.latex?\psi,\theta,&space;&space;\phi"> respectively. 

The transformation matrix from the navigation frame to the body frame is
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?C_n^b=\begin{bmatrix}cos(\theta)cos(\psi)&space;&cos(\theta)sin(\psi)&space;&-sin(\theta)&space;\\cos(\psi)sin(\phi)sin(\theta)-cos(\phi)sin(\psi)&space;&cos(\phi)cos(\psi)&plus;sin(\phi)sin(\theta)sin(\psi)&space;&cos(\theta)sin(\phi)&space;\\sin(\phi)sin(\psi)&plus;cos(\phi)cos(\psi)sin(\theta)&space;&cos(\phi)sin(\theta)sin(\psi)-cos(\psi)sin(\phi)&space;&cos(\phi)cos(\theta)&space;\end{bmatrix}" />
</div>

We can calculate the angular rate from the Euler angles derivatives:

<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\begin{align*}&&space;\omega_x=\dot{\phi}-\dot{\psi}sin(\theta)&space;\\&space;&&space;\omega_y=\dot{\theta}cos(\phi)+\dot{\psi}cos(\theta)sin(\phi)\\&space;&&space;\omega_z=\dot{\psi}cos(\phi)cos(\theta)-\dot{\theta}sin(\phi)&space;\end{align*}" />
</div>

or calculate Euler angles derivatives from the angular rate:
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\begin{align*}&&space;\dot{\psi}=(\omega_zcos(\phi)+\omega_ysin(\phi))\cdot&space;sec(\theta)&space;\\&space;&&space;\dot{\theta}=\omega_ycos(\phi)-\omega_zsin(\phi)\\&space;&&space;\dot{\phi}=\omega_x+(\omega_zcos(phi)+\omega_ysin(phi))\cdot&space;tan(\theta)&space;\end{align*}" />
</div>

The Euler angles derivatives are used to propagate attitude in the path generation module in **gnss-ins-sim**.

Besides the coordinate transfromation matrix (the Direction Cosine Matrix, DCM) and the Euler angles, quaternions can also be used to represent attitude.

### 4.2.2 Translation

According to Newton's second law of motion, positions and velocities can be calculated by integrating acceleration with known initial states:
<div align=center>
<img src="https://latex.codecogs.com/gif.latex?\begin{align*}&&space;a^n(t)=C_b^na^b(t)&space;\\&space;&&space;\dot{v}^n(t)=a^n(t)\\&space;&&space;\dot{r}^n(t)=v^n(t)&space;\end{align*}" />
</div>

where, <img src="https://latex.codecogs.com/gif.latex?C_b^n" /> is the coordinate transformation matrix from the body frame to the navigation frame, which can be calculated according to formula in [4.2.1 Rotation](#421-rotation)

## 5 Simulation design and implementation

------
**gnss-ins-sim** is composed of the following modules:

| name | description |
|---|---|
| allan | This is a built-in Allan variance module. |
| psd | The psd module implements a function to generate time series from a given single-sided PSD. It is used to generate accelerations when users specify a vibration model by PSD. |
| geoparams | The geoparams module contains two parts: Geomagnetic field and WGS-84 Earth model. |
| attitude | The attitude model provides functions to manipulate different representations of attitude: Euler angles, DCMs and quaternions. |
| pathgen | The pathgen module generates true postions, velocities, Euler angles, angular rates and accelerations accroding to motion definition files. It also generate sensor measurement according to given IMU, magnetometer and GPS error profiles. |
| kml_gen | The kml_gen module generates .kml files with given series of latitude, longitude and altitude. The generated .kml files can be imported into Google Earth to visualize the trajectories. |
| sim | The sim module provides interfaces to the users. Most users would just use sim to do simulations. If you want more control, please refer to the above modules. |

 A diagram of **gnss-ins-sim** is shown below.

 <div align=center>
<img width="500"  src="https://github.com/Aceinna/gnss-ins-sim/blob/master/gnss_ins_sim/docs/images/simulator.png"/>
</div>

Please refer to doc strings of the functions in each module.

## 6 References

------

* [1] [http://www.navipedia.net/index.php/Positioning_Error](http://www.navipedia.net/index.php/Positioning_Error)
* [2] Woodman, Oliver J. An introduction to inertial navigation. No. UCAM-CL-TR-696. University of Cambridge, Computer Laboratory, 2007.