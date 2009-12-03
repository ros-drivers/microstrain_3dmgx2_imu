/*
 * hokuyourg_player
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**

@mainpage

@htmlinclude manifest.html

@b The imu_node is designed to make use of the microstrain inertialink
or 3dmgx2 IMUs and makes use of the 3dmgx2_driver.

<hr>

@section information Information

The IMU provides a single message Imu messaged at 100Hz
which is taken from the 3DMGX2 ACCEL_ANGRATE_ORIENTATION message.

<hr>

@section usage Usage

@par Example

@verbatim
$ imu_node
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- @b "imu_data"/<a href="../../sensor_msgs/html/classstd__msgs_1_1Imu.html">sensor_msgs/Imu</a> : the imu data
- @b "/diagnostics"/<a href="../../diagnostic_msgs/html/classrobot__msgs_1_1DiagnosticMessage.html">diagnostic_msgs/DiagnosticMessage</a> : diagnostic status information.
- @b "~is_calibrated" :  Latched topic indicating if the gyro is calibrated. 

<hr>

@section services
 - @b "~self_test"     :  SelfTest service provided by SelfTest helper class
 - @b "~calibrate"     :  Calibrate the gyro's biases. The gyro must not move during calibration

<hr>

@section parameters ROS parameters

Reads the following parameters from the parameter server

 - @b "~port"          : @b [string] the port the imu is running on
 - @b "~frame_id"      : @b [string] the frame in which imu readings will be returned (Default: "imu")
 - @b "~autocalibrate" : @b [bool] whether the imu automatically computes its biases at startup (not useful if you intend to calibrate via the calibrate service)
 - @b "~orientation_stdev" : @b [double] square root of the orientation_covariance diagonal elements in rad (Default: 0.035)
 - @b "~angular_velocity_stdev" : @b [double] square root of the angular_velocity_covariance diagonal elements in rad/s (Default: 0.012)
 - @b "~linear_acceleration_stdev" : @b [double] square root of the linear_acceleration_covariance diagonal elements in m/s^2 (Default: 0.098)

 **/

#include <assert.h>
#include <math.h>
#include <iostream>

#include "3dmgx2_driver/3dmgx2.h"

#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"

#include "tf/transform_datatypes.h"
#include "imu_node/AddOffset.h"

#include "boost/thread.hpp"
#include "std_msgs/Bool.h"

using namespace std;

class ImuNode 
{
public:
  ms_3dmgx2_driver::IMU imu;
  sensor_msgs::Imu reading;

  string port;

  ms_3dmgx2_driver::IMU::cmd cmd;

  SelfTest<ImuNode> self_test_;
  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher imu_data_pub_;
  ros::ServiceServer add_offset_serv_;
  ros::ServiceServer calibrate_serv_;
  ros::Publisher is_calibrated_pub_;

  bool running;

  bool autocalibrate_;
  bool calibrated_;
  volatile bool calibrate_request_;
  
  int error_count_;

  string frameid_;
  
  double offset_;
    
  double bias_x_;
  double bias_y_;
  double bias_z_;

  double angular_velocity_stdev_, angular_velocity_covariance_;
  double linear_acceleration_covariance_, linear_acceleration_stdev_;
  double orientation_covariance_, orientation_stdev_;

  double desired_freq_;
  diagnostic_updater::FrequencyStatus freq_diag_;

  ImuNode(ros::NodeHandle h) : self_test_(this), diagnostic_(h), 
  node_handle_(h), private_node_handle_("~"), calibrated_(false), 
  calibrate_request_(false), error_count_(0), 
  desired_freq_(100), 
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
  {
    imu_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>("imu_data", 100);

    add_offset_serv_ = private_node_handle_.advertiseService("add_offset", &ImuNode::addOffset, this);
    calibrate_serv_ = private_node_handle_.advertiseService("calibrate", &ImuNode::calibrate, this);
    is_calibrated_pub_ = private_node_handle_.advertise<std_msgs::Bool>("is_calibrated", 1, true);

    publish_is_calibrated();

    private_node_handle_.param("autocalibrate", autocalibrate_, true);
    
    private_node_handle_.param("port", port, string("/dev/ttyUSB0"));

    cmd = ms_3dmgx2_driver::IMU::CMD_ACCEL_ANGRATE_ORIENT;
    
    running = false;

    bias_x_ = bias_y_ = bias_z_ = 0;

    private_node_handle_.param("frameid", frameid_, string("imu"));
    reading.header.frame_id = frameid_;

    private_node_handle_.param("time_offset", offset_, 0.0);

    private_node_handle_.param("linear_acceleration_stdev", linear_acceleration_stdev_, 0.098); 
    private_node_handle_.param("orientation_stdev", orientation_stdev_, 0.035); 
    private_node_handle_.param("angular_velocity_stdev", angular_velocity_stdev_, 0.012); 

    double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    double orientation_covariance = orientation_stdev_ * orientation_stdev_;
    double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    
    reading.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    reading.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    reading.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    reading.angular_velocity_covariance[0] = angular_velocity_covariance;
    reading.angular_velocity_covariance[4] = angular_velocity_covariance;
    reading.angular_velocity_covariance[8] = angular_velocity_covariance;
    
    reading.orientation_covariance[0] = orientation_covariance;
    reading.orientation_covariance[4] = orientation_covariance;
    reading.orientation_covariance[8] = orientation_covariance;
    
    self_test_.setPretest(&ImuNode::pretest);
    self_test_.addTest(&ImuNode::InterruptionTest);
    self_test_.addTest(&ImuNode::ConnectTest);
    self_test_.addTest(&ImuNode::ReadIDTest);
    self_test_.addTest(&ImuNode::GyroBiasTest);
    self_test_.addTest(&ImuNode::StreamedDataTest);
    self_test_.addTest(&ImuNode::GravityTest);
    self_test_.addTest(&ImuNode::DisconnectTest);
    self_test_.addTest(&ImuNode::ResumeTest);

    diagnostic_.add( freq_diag_ );
    diagnostic_.add( "Calibration Status", this, &ImuNode::calibrationStatus );
    diagnostic_.add( "IMU Status", this, &ImuNode::deviceStatus );
  }

  ~ImuNode()
  {
    stop();
  }

  int start()
  {
    stop();

    try
    {
      imu.openPort(port.c_str());

      char dev_name[17];
      char dev_model_num[17];
      char dev_serial_num[17];
      char dev_opt[17];
      imu.getDeviceIdentifierString(ms_3dmgx2_driver::IMU::ID_DEVICE_NAME, dev_name);
      imu.getDeviceIdentifierString(ms_3dmgx2_driver::IMU::ID_MODEL_NUMBER, dev_model_num);
      imu.getDeviceIdentifierString(ms_3dmgx2_driver::IMU::ID_SERIAL_NUMBER, dev_serial_num);
      imu.getDeviceIdentifierString(ms_3dmgx2_driver::IMU::ID_DEVICE_OPTIONS, dev_opt);
      ROS_INFO("Connected to IMU [%s] model [%s] s/n [%s] options [%s]",
          dev_name, dev_model_num, dev_serial_num, dev_opt);

      if (autocalibrate_)
      {
        calibrate_request_ = true;
        calibrate_req_check();
      }
      else
      {
        ROS_INFO("Not calibrating the IMU sensor. Use the calibrate service to calibrate it before use.");
      }

      ROS_INFO("Initializing IMU time with offset %f.", offset_);

      imu.initTime(offset_);

      ROS_INFO("IMU sensor initialized.");

      imu.setContinuous(cmd);

      freq_diag_.clear();

      running = true;

    } catch (ms_3dmgx2_driver::Exception& e) {
      error_count_++;
      ROS_ERROR("Exception thrown while starting IMU.\n %s", e.what());
      diagnostic_.broadcast(2, "Error opening IMU.");
      return -1;
    }

    return(0);
  }
  
  int stop()
  {
    if(running)
    {
      try
      {
        imu.closePort();
      } catch (ms_3dmgx2_driver::Exception& e) {
        error_count_++;
        ROS_INFO("Exception thrown while stopping IMU.\n %s", e.what());
      }
      running = false;
    }

    return(0);
  }

  int publish_datum()
  {
    try
    {
      uint64_t time;

      double accel[3];
      double angrate[3];
      double orientation[9];

      static double prevtime = 0;
      double starttime = ros::Time::now().toSec();
      if (prevtime && prevtime - starttime > 0.025)
        ROS_WARN("Full IMU loop took %f ms. Nominal is 10ms.", 1000 * (prevtime - starttime));
      imu.receiveAccelAngrateOrientation(&time, accel, angrate, orientation);
      double endtime = ros::Time::now().toSec();
      if (endtime - starttime > 0.025)
        ROS_WARN("Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
      prevtime = starttime;

      reading.linear_acceleration.x = accel[0];
      reading.linear_acceleration.y = accel[1];
      reading.linear_acceleration.z = accel[2];
 
      reading.angular_velocity.x = angrate[0];
      reading.angular_velocity.y = angrate[1];
      reading.angular_velocity.z = angrate[2];
      
      btQuaternion quat;
      btMatrix3x3(orientation[0], orientation[1], orientation[2],
                  orientation[3], orientation[4], orientation[5],
                  orientation[6], orientation[7], orientation[8]).getRotation(quat);
      
      tf::quaternionTFToMsg(quat, reading.orientation);
      
      reading.header.stamp = ros::Time::now().fromNSec(time);

      starttime = ros::Time::now().toSec();
      imu_data_pub_.publish(reading);
      endtime = ros::Time::now().toSec();
      if (endtime - starttime > 0.025)
        ROS_WARN("Publishing took %f ms. Nominal is 10 ms.", 1000 * (endtime - starttime));
        
      freq_diag_.tick();
    } catch (ms_3dmgx2_driver::Exception& e) {
      error_count_++;
      ROS_INFO("Exception thrown while trying to get the IMU reading.\n%s", e.what());
      return -1;
    }

    return(0);
  }

  bool spin()
  {
    while (node_handle_.ok())
    {
      if (start() == 0)
      {
        while(node_handle_.ok()) {
          if(publish_datum() < 0)
            break;
          self_test_.checkTest();
          diagnostic_.update();
          calibrate_req_check();
        }
      } else {
        usleep(1000000);
        self_test_.checkTest();
        diagnostic_.update();
        calibrate_req_check();
      }
    }

    stop();

    return true;
  }

  void publish_is_calibrated()
  {
    std_msgs::Bool msg;
    msg.data = calibrated_;
    is_calibrated_pub_.publish(msg);
  }

  void pretest()
  {
    try
    {
      imu.closePort();
    } catch (ms_3dmgx2_driver::Exception& e) {
    }
  }

  void InterruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Interruption Test";

    if (imu_data_pub_.getNumSubscribers() == 0 )
    {
      status.level = 0;
      status.message = "No operation interrupted.";
    }
    else
    {
      status.level = 1;
      status.message = "There were active subscribers.  Running of self test interrupted operations.";
    }
  }

  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Connection Test";

    imu.openPort(port.c_str());

    status.summary(0, "Connected successfully.");
  }

  void ReadIDTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    char id[17];

    status.name = "Read ID Test";

    imu.getDeviceIdentifierString(ms_3dmgx2_driver::IMU::ID_SERIAL_NUMBER, id);

    self_test_.setID(id);
    
    status.summary(0, "Read Successfully");
  }

  void GyroBiasTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Gyro Bias Test";

    imu.initGyros(&bias_x_, &bias_y_, &bias_z_);

    status.summary(0, "Successfully calculated gyro biases.");

    status.add("Bias_X", bias_x_);
    status.add("Bias_Y", bias_y_);
    status.add("Bias_Z", bias_z_);
  }


  void StreamedDataTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Streamed Data Test";

    uint64_t time;
    double accel[3];
    double angrate[3];

    if (!imu.setContinuous(ms_3dmgx2_driver::IMU::CMD_ACCEL_ANGRATE))
    {
      status.summary(2, "Could not start streaming data.");
    } else {

      for (int i = 0; i < 100; i++)
      {
        imu.receiveAccelAngrate(&time, accel, angrate);
      }
      
      imu.stopContinuous();

      status.summary(0, "Data streamed successfully.");
    }
  }

  void GravityTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Streamed Data Test";

    uint64_t time;
    double accel[3];
    double angrate[3];

    double grav = 0.0;

    double grav_x = 0.0;
    double grav_y = 0.0;
    double grav_z = 0.0;

    if (!imu.setContinuous(ms_3dmgx2_driver::IMU::CMD_ACCEL_ANGRATE))
    {
      status.summary(2, "Could not start streaming data.");
    } else {

      int num = 200;

      for (int i = 0; i < num; i++)
      {
        imu.receiveAccelAngrate(&time, accel, angrate);
        
        grav_x += accel[0];
        grav_y += accel[1];
        grav_z += accel[2];

      }
      
      imu.stopContinuous();

      grav += sqrt( pow(grav_x / (double)(num), 2.0) + 
                    pow(grav_y / (double)(num), 2.0) + 
                    pow(grav_z / (double)(num), 2.0));
      
      //      double err = (grav - ms_3dmgx2_driver::G);
      double err = (grav - 9.796);
      
      if (fabs(err) < .05)
      {
        status.summary(0, "Gravity detected correctly.");
      } else {
        status.summaryf(2, "Measured gravity deviates by %f", err);
      }

      status.add("Measured gravity", grav);
      status.add("Gravity error", err);
    }
  }


  void DisconnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Disconnect Test";

    imu.closePort();

    status.summary(0, "Disconnected successfully.");
  }

  void ResumeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Resume Test";

    if (running)
    {

      imu.openPort(port.c_str());
      freq_diag_.clear();

      if (imu.setContinuous(cmd) != true)
      {
        status.summary(2, "Failed to resume previous mode of operation.");
        return;
      }
    }

    status.summary(0, "Previous operation resumed successfully.");    
  }

  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if (running)
      status.summary(0, "IMU is running");
    else
      status.summary(2, "IMU is stopped");

    status.add("Device", port);
    status.add("TF frame", frameid_);
    status.add("Error count", error_count_);
  }

  void calibrationStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (calibrated_)
    {
      status.summary(0, "Gyro is calibrated");
      status.add("X bias", bias_x_);
      status.add("Y bias", bias_y_);
      status.add("Z bias", bias_z_);
    }
    else
      status.summary(2, "Gyro not calibrated");

  }

  bool addOffset(imu_node::AddOffset::Request &req, imu_node::AddOffset::Response &resp)
  {
    double offset = req.add_offset;
    offset_ += offset;

    ROS_INFO("Adding %f to existing IMU time offset.", offset);
    ROS_INFO("Total IMU time offset is now %f.", offset_);

    // send changes to imu driver
    imu.setFixedOffset(offset_);

    // write changes to param server
    private_node_handle_.setParam("time_offset", offset_);

    // set response
    resp.total_offset = offset_;

    return true;
  }

  bool calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    calibrate_request_ = true;

    while (calibrate_request_)
    {
      sleep(1); // The calibration takes 10s, so this is plenty often enough.
    }

    return true;
  }
  
  void calibrate_req_check()
  {
    if (calibrate_request_)
    {
      ROS_INFO("Calibrating IMU gyros.");
      imu.initGyros(&bias_x_, &bias_y_, &bias_z_);
      calibrated_ = true;
      publish_is_calibrated();
      calibrate_request_ = false;
      ROS_INFO("IMU gyro calibration completed.");
      freq_diag_.clear();
    }
  }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_node");

  ros::NodeHandle nh;

  ImuNode in(nh);
  
  boost::thread DriverSpinThread(boost::bind(&ImuNode::spin, &in));

  ros::spin();

  DriverSpinThread.timed_join(boost::posix_time::seconds(2) );

  return(0);
}
