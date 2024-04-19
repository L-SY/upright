//
// Created by lsy on 24-4-18.
//

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <chrono>
#include <thread>

#include "diablo_hw/OSDKVehicle.h"
#include "diablo_hw/OnboardSDKUartProtocol.h"
#include "realtime_tools/realtime_buffer.h"

namespace diablo_hw
{
enum ctrl_mode
{
  JOY = 0,
  TOPIC = 1,
  NONE = 2
};
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
  void commandCB(const geometry_msgs::TwistConstPtr& msg)
  {
    Mode_ = TOPIC;
    cmd_rt_buffer_.writeFromNonRT(*msg);
    lastTopicCmdTime_ = ros::Time::now();
  }
  void joyCommandCB(const sensor_msgs::JoyConstPtr& msg)
  {
    Mode_ = ctrl_mode::JOY;
    movementCtrl_->ctrl_data.forward = msg.get()->axes[1];
    movementCtrl_->ctrl_data.left = msg.get()->axes[3];
    uint8_t result = movementCtrl_->SendMovementCtrlCmd();
    lastJoyCmdTime_ = ros::Time::now();
  }

  void judgeControlMode()
  {
    if (Mode_ == TOPIC)
      if ((ros::Time::now().toSec() - lastTopicCmdTime_.toSec()) >= 1)
      {
        Mode_ = NONE;
        ROS_INFO_STREAM("1s not receive msg form /diablo_cmd, the Mode is change to NONE");
      }
    if (Mode_ == JOY)
      if ((ros::Time::now().toSec() - lastJoyCmdTime_.toSec()) >= 1)
      {
        //        TODO: (4yang) it has bug in the judge but do not make any err
        Mode_ = NONE;
        ROS_INFO_STREAM("1s not receive msg form /joy, the Mode is change to NONE");
      }
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
  sensor_msgs::Joy joy_;
  sensor_msgs::JointState leftHipJoint_, rightHipJoint_, leftWheelJoint_, rightWheelJoint_, leftKneeJoint_,
      rightKneeJoint_;

  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_rt_buffer_;
  ros::Subscriber diabloCmdSub_, joySub_;
  ros::Publisher leftHipJointPub_, rightHipJointPub_, leftWheelJointPub_, rightWheelJointPub_, leftKneeJointPub_,
      rightKneeJointPub_, imuPub_;
  ctrl_mode Mode_ = NONE;
  ros::Time lastJoyCmdTime_, lastTopicCmdTime_;
};
}  // namespace diablo_hw