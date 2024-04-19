//
// Created by lsy on 24-4-18.
//

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <chrono>
#include <thread>

#include "diablo_hw/OSDKVehicle.h"
#include "diablo_hw/OnboardSDKUartProtocol.h"
#include "realtime_tools/realtime_buffer.h"

namespace diablo_hw
{
class DiabloRobot
{
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;

public:
  DiabloRobot(DIABLO::OSDK::Vehicle* vehicle, DIABLO::OSDK::Movement_Ctrl* movementCtrl, ros::NodeHandle nodeHandle);
  void read();
  void write();
  void update();

private:
  void commandCB(const geometry_msgs::TwistConstPtr& msg);
  void testCommandCB(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO_STREAM("CB");
  }

  void pubDiabloInfo()
  {
    imuPub_.publish(imu_);
    leftHipJointPub_.publish(leftHipJoint_);
    rightHipJointPub_.publish(rightHipJoint_);
    leftWheelJointPub_.publish(leftWheelJoint_);
    rightWheelJointPub_.publish(rightWheelJoint_);
    leftKneeJointPub_.publish(leftKneeJoint_);
    rightKneeJointPub_.publish(rightKneeJoint_);
  }

  // Timing
  double cycleTimeErrorThreshold_{}, loopHz_{};
  std::thread loopThread_;
  std::atomic_bool loopRunning_{};
  ros::Duration elapsedTime_;
  Clock::time_point lastTime_;

  std::shared_ptr<DIABLO::OSDK::Vehicle> vehicle_;
  std::shared_ptr<DIABLO::OSDK::Movement_Ctrl> movementCtrl_;
  sensor_msgs::Imu imu_;
  sensor_msgs::JointState leftHipJoint_, rightHipJoint_, leftWheelJoint_, rightWheelJoint_, leftKneeJoint_,
      rightKneeJoint_;

  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_rt_buffer_;
  ros::Subscriber diabloCmdSub_, testSub_;
  ros::Publisher leftHipJointPub_, rightHipJointPub_, leftWheelJointPub_, rightWheelJointPub_, leftKneeJointPub_,
      rightKneeJointPub_, imuPub_;
};
}  // namespace diablo_hw