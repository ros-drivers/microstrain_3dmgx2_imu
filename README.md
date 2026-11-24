![attachment:imu.png](doc/imu.png)

(original 3DM-GX2 coordinate system)

## Who Should Use this Package?

- If you are running on the PR2/Texas robot this driver should be
  running by default when you bring up the robot. You do not need to
  launch it yourself.

<!-- -->

- If you want to use 3DM-GX2 compatible IMU with ROS, use the
  [imu_node](#RosAPI).

<!-- -->

- For advanced users who want lower-level access to a 3DB-GX2 compatible
  IMU, the unsupported
  [C++ API](http://www.ros.org/doc/api/microstrain_3dmgx2_imu/html/classmicrostrain__3dmgx2__imu_1_1IMU.html)
  may be used.

## Supported Hardware

This driver should work with IMUs that use the Microstrain 3DM-GX2
protocol.

## API Stability

The ROS API of this node should be considered stable.

While a
[C++ API](http://www.ros.org/doc/api/microstrain_3dmgx2_imu/html/classmicrostrain__3dmgx2__imu_1_1IMU.html)
exists, it has not been reviewed and should be considered unstable.

## ROS API

<a id="RosAPI"></a>

### imu_node

`imu_node` is a driver for the 3DM-GX2 Inertial Measurement Unit. It
publishes orientation, angular velocity and linear acceleration as well
as their covariances, and complies with the
[IMU API](imu_drivers#RosAPI). This node only provides streaming access
to message 0xC8 (Acceleration, Angular Rate and Orientation), the
unsupported
[C++ API](http://www.ros.org/doc/api/microstrain_3dmgx2_imu/html/classmicrostrain__3dmgx2__imu_1_1IMU.html)
gives access to a broader range of message types.

#### Published Topics

- `imu/data`
  ([sensor_msgs/msg/Imu](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Imu.html)):
  Streaming inertial data from the IMU. Includes acceleration, angular
  rates and orientation. The orientation is produced by the IMU
  firmware, which uses gravity, and in some cases the Earth's magnetic
  field, to remove integration drift. The orientation matrix is the
  transpose of the orientation matrix returned by the hardware, rotated
  180 degrees around the y axis. This corresponds to a transformation
  from the IMU frame to the world frame, with the z axis point up.
- `diagnostics`
  ([diagnostic_msgs/msg/DiagnosticArray](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticArray.html)):
  Diagnostic status information.
- `imu/is_calibrated`
  ([std_msgs/msg/Bool](https://docs.ros.org/en/rolling/p/std_msgs/msg/Bool.html)):
  Latched topic indicating if the gyroscope biases have been calibrated.

#### Services

- `~self_test`
  ([diagnostic_msgs/srv/SelfTest](https://docs.ros.org/en/rolling/p/diagnostic_msgs/srv/SelfTest.html)):
  Starts the imu self test.
- `imu/calibrate`
  ([std_srvs/srv/Empty](https://docs.ros.org/en/rolling/p/std_srvs/srv/Empty.html)):
  Tells the IMU to (re)calibrate its gyroscope biases. The service call
  returns once calibration is complete. Calibration failure can be
  detected by subscribing to the `imu/is_calibrated` topic. A message
  will published to `imu/is_calibrated` for each call to
  `imu/calibrate`.

#### Parameters

- `~port` (`string`, default: `/dev/ttyUSB0`): The port the imu is
  running on.
- `~frame_id` (`string`, default: `imu`): The frame in which imu
  readings will be returned.
- `~autocalibrate` (`bool`, default: `true`): Whether the imu
  automatically computes its biases at startup (set to `false` if you
  intend to calibrate via the calibrate service).
- `~orientation_stdev` (`double`, default: `0.035`): Square root of the
  orientation_covariance diagonal elements in rad. The default value,
  derived from the manufacturer datasheet, is very conservative.
- `~angular_velocity_stdev` (`double`, default: `0.012`): Square root of
  the angular_velocity_covariance diagonal elements in rad/s. The
  default value, derived from the manufacturer datasheet, is very
  conservative.
- `~linear_acceleration_stdev` (`double`, default: `0.098`): Square root
  of the linear_acceleration_covariance diagonal elements in m/s\^2. The
  default value, derived from the manufacturer datasheet, is very
  conservative.
- `~max_drift_rate` (`double`, default: `0.0001`): Just after performing
  a calibration, if the observed gyro rates are greater than this
  parameter, the calibration is assumed to have failed, and the imu
  starts reporting itself as uncalibrated.
- `~assume_calibrated` (`bool`, default: `false`): Indicates whether the
  IMU should assume that it is calibrated at startup.

## Communication Protocol Manual

The 3DM-GX2 protocol can be found
[here](http://files.microstrain.com/dcp/Inertia-Link-3DM-GX2-data-communications-protocol.pdf).

## Tutorials

- [How to Use Inertial-Link IMUs with the imu_node](#how-to-use-inertial-link-imus-with-the-imu_node):
  This tutorial is an introduction to using the !MicroStrain
  Inertial-Link IMU connected to a desktop. After reading the tutorial,
  you should be able to bring up the imu_node and display the IMU data.

## Troubleshooting

### Errors when running imu_node

#### Error: Unable to open serial port



```
[ERROR] 1263326790.987557000: Unable to open serial port [/dev/ttyUSB0]. No such file or directory. The requested port does not exist. Is the IMU connected? Was the port name misspelled? (in microstrain_3dmgx2_imu::IMU:openPort) You may find further details at http://www.ros.org/wiki/microstrain_3dmgx2_imu/Troubleshooting
```



**Solution:**

- Is the IMU connected?
- Was the port name misspelled?

#### Error: Permission denied



```
[ERROR] 1263327271.083389000: Unable to open serial port [/u/blaise/bar]. Permission denied. You probably don't have premission to open the port for reading and writing. (in microstrain_3dmgx2_imu::IMU:openPort) You may find further details at http://www.ros.org/wiki/microstrain_3dmgx2_imu/Troubleshooting
```

 **Solution:**

- Use **chmod** to change the permissions on the port You can use
  **chmod** as illustrated in the
  [tutorial](http://www.ros.org/wiki/microstrain_3dmgx2_imu/Tutorials/UsingTheIMUNode).
  You will also need e**x**ecute access on the directories containing
  the device.

<!-- -->

- Use **udev** For more permanent use, you should use **udev** to
  automatically set the correct permissions on the device.

#### Error: IMU keeps restarting with an incoherent sequence of errors



```
[ WARN] 1263339155.975117000: Exception thrown while trying to get the IMU reading. This sometimes happens if another process is trying to access the IMU port. You may try 'lsof|grep /dev/ttyUSB1' to find processes that have the port open. invalid checksum.
 Make sure the IMU sensor is connected to this computer. (in microstrain_3dmgx2_imu::IMU:receive)
[ INFO] 1263339158.037993000: Connected to IMU [    Inertia-Link] model [            4200] s/n [            3046] options [       5g 300d/s]
[ INFO] 1263339158.038080000: Not calibrating the IMU sensor. Use the calibrate service to calibrate it before use.
[ INFO] 1263339158.038120000: Initializing IMU time with offset 0.000000.
[ INFO] 1263339158.049985000: IMU sensor initialized.
[ WARN] 1263339161.050045000: Exception thrown while trying to get the IMU reading. This sometimes happens if another process is trying to access the IMU port. You may try 'lsof|grep /dev/ttyUSB1' to find processes that have the port open. timeout reached (in microstrain_3dmgx2_imu::IMU:read_with_timeout)
[ INFO] 1263339163.107080000: Connected to IMU [       5g 300d/s] model [    Inertia-Link] s/n [            3046] options [       5g 300d/s]
[ INFO] 1263339163.107172000: Not calibrating the IMU sensor. Use the calibrate service to calibrate it before use.
[ INFO] 1263339163.107219000: Initializing IMU time with offset 0.000000.
[ INFO] 1263339163.122005000: IMU sensor initialized.
[ERROR] 1263339163.251135000: Exception thrown while starting IMU.
 invalid checksum.
 Make sure the IMU sensor is connected to this computer. (in microstrain_3dmgx2_imu::IMU:receive) You may find further details at http://www.ros.org/wiki/microstrain_3dmgx2_imu/Troubleshooting
```

 **Solution:**

- More than one process is trying to access the IMU. Try running
  

```
lsof|grep /dev/ttyUSB0
```

 to find all the processes that might be
  accessing the IMU. Kill the ones that shouldn't be running.
- That serial port is not connected to an IMU. Try unplugging and
  replugging devices to figure out which port corresponds to the IMU.
  For a permanent setup use **udev** to assign consistent names to the
  ports.

#### Error: Invalid checksum



```
user@prX:rosrun imu_node imu_node
 [ INFO] 1236966577.039701000: Starting node [/imu]
 [ INFO] 1236966577.039923000: Started XML-RPC server on port [59905]
 [ INFO] 1236966577.039987000: Started TCPROS server on port [43106]
 [ INFO] 1236966578.049387000: Initializing IMU sensor.
 [ INFO] 1236966580.016606000: Exception thrown while starting IMU.
 MS_3DMGX2::IMU::receive: invalid checksum.
 Make sure the IMU sensor is connected to this computer.
```



**Solution:**

- Is the IMU connected? A device is connected but not the IMU.

### Errors when bringing up the PR2

#### Error: Gyro Not Calibrated

The **Calibration Status** in the diagnostics indicates


```
Gyro not calibrated
```

 The mechanism calibration scripts in
mechanism_bringup output 

```
Mechanism calibration complete, but IMU calibration failed.
```



**Solution:**

- Short term solution: Use rosservice to manually trigger calibration
  and get on with your work: 

```
rosservice call imu_node/calibrate
```


- Permanent solution: In most cases, you will want to set the IMU's
  autocalibrate option to true so that the IMU calibrates at startup.
  Beware, the IMU should not move during calibration, or the calibration
  will be incorrect. The only case in which you do not want to do this
  is if you know that the IMU calibration is being triggered later in
  the bringup process.

## How to Use Inertial-Link IMUs with the imu_node

- **Note:** This tutorial assumes that you have completed the previous
  tutorials: [ROS tutorials](ROS/Tutorials).
- **Description:** This tutorial is an introduction to using the
  !MicroStrain Inertial-Link IMU connected to a desktop. After reading
  the tutorial, you should be able to bring up the imu_node and display
  the IMU data.
- **Keywords:** IMU, driver
- **Tutorial Level:** BEGINNER

### Compiling

Start by getting the dependencies and compiling the driver.



```
$ rosdep install microstrain_3dmgx2_imu
$ rosmake microstrain_3dmgx2_imu
```



### Powered On and Plugged In

Make sure that your USB imu is plugged in and the power light is on.

### Configuring the IMU

Make sure that the imu_node will be able to access the imu.

Start by listing the permissions of the IMU:



```
ls -l /dev/ttyUSB0
```

 You will see something similar to:

. 

```
crw-rw-XX- 1 root dialout 188, 0 2009-08-14 12:04 /dev/ttyUSB0
```



If XX is rw: the usb device is configured properly.

If XX is \--: the usb device is no configured properly and you need to:



```
sudo chmod a+rw /dev/ttyUSB0
```



### Starting a roscore

For the imu_node to work properly, a ros core must be running. In a
**new terminal**:



```
$ roscore
```



### Running the imu_node

In a **new terminal**, run the imu_node:



```
$ rosrun microstrain_3dmgx2_imu imu_node
```

 You will see something similar to:

. 

```
[ INFO] 1250276678.466059000: Started node [/imu], pid [17125], bound on [aqy], xmlrpc port [49515], tcpros port [44773], logging to [/u/mwise/ros/ros/log/imu_17125.log], using [real] time
[ INFO] 1250276679.540144000: Connected to IMU [    Inertia-Link] model [            4200] s/n [            3046] options [       5g 300d/s]
[ INFO] 1250276679.540247000: Calibrating IMU gyros.
[ INFO] 1250276689.650297000: IMU gyro calibration completed.
[ INFO] 1250276689.650416000: Initializing IMU time with offset 0.000000.
[ INFO] 1250276689.664184000: IMU sensor initialized.
```



Note: If your IMU is at a port other than /dev/ttyUSB0, you could
explicitly specify the port parameter, for example:


```
$ rosrun microstrain_3dmgx2_imu imu_node _port:=/dev/ttyUSB1
```



### Viewing the data

To see that everything is working and data is being published to ros, in
a **new terminal**:



```
$ rostopic echo imu/data
```

 You will see something similar to:

. 

```
header: 
  seq: 1791
  stamp: 1263496141218770057
  frame_id: imu
orientation: 
  x: -0.00113008373396
  y: 0.00770460351395
  z: 0.379699384935
  w: 0.925077162566
orientation_covariance: [0.0012250000000000002, 0.0, 0.0, 0.0, 0.0012250000000000002, 0.0, 0.0, 0.0, 0.0012250000000000002]
angular_velocity: 
  x: 0.005341142416
  y: -0.00330266845413
  z: -7.2395623647e-06
angular_velocity_covariance: [2.8900000000000004e-08, 0.0, 0.0, 0.0, 2.8900000000000004e-08, 0.0, 0.0, 0.0, 2.8900000000000004e-08]
linear_acceleration: 
  x: 0.438507700588
  y: 0.012467633853
  z: -9.82538275752
linear_acceleration_covariance: [0.0096040000000000014, 0.0, 0.0, 0.0, 0.0096040000000000014, 0.0, 0.0, 0.0, 0.0096040000000000014]
---
```



Move the IMU around to see the data change.

Note that the covariances have been set to values consistent with the
manufacturer datasheet. These covariances are very pessimistic and you
will most likely be able to use smaller values in your application.

------------------------------------------------------------------------
