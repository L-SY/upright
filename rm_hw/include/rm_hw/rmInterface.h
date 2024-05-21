//
// Created by lsy on 24-4-26.
//

#pragma once

#include <chrono>
#include <thread>
#include <memory>

#include "geometry_msgs/Pose.h"
#include "ps5_joy/ps5Joy.h"
#include "rm_msgs/MoveC.h"
#include "rm_msgs/MoveJ.h"
#include "rm_msgs/MoveJ_P.h"
#include "rm_msgs/Plan_State.h"
#include "rm_msgs/MoveL.h"
#include "rm_msgs/JointPos.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <upright_common/filters/lp_filter.h>

namespace rm_interface
{

class rmInterface
{
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;

public:
  rmInterface(ros::NodeHandle& nh)
  {
    // Initialize jointStates_ with empty JointState objects named joint1 to
    // joint6
    for (int i = 1; i <= 6; ++i)
    {
      jointStates_.name.push_back("joint" + std::to_string(i));
      jointStates_.position.push_back(0.);
      jointStates_.velocity.push_back(0.);
      jointStates_.effort.push_back(0.);
      lastJointStates_.position.push_back(0.);
    }

    // Subscribe to /joint_state and /rm_driver/Pose_State
    jointStateSub_ = nh.subscribe("/joint_states", 10, &rmInterface::jointStateCallback, this);
    poseStateSub_ = nh.subscribe("rm_driver/Pose_State", 10, &rmInterface::poseStateCallback, this);
    planStateSub_ = nh.subscribe("/rm_driver/Plan_State", 10, &rmInterface::planStateCallback, this);
    moveJPPub_ = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);
    moveJPub_ = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);
    moveJPosPub_ = nh.advertise<rm_msgs::JointPos>("/rm_driver/JointPos", 10);
    j1LPFilter_ = std::make_shared<LowPassFilter>(nh);
    j2LPFilter_ = std::make_shared<LowPassFilter>(nh);
    j3LPFilter_ = std::make_shared<LowPassFilter>(nh);
    j4LPFilter_ = std::make_shared<LowPassFilter>(nh);
    j5LPFilter_ = std::make_shared<LowPassFilter>(nh);
    j6LPFilter_ = std::make_shared<LowPassFilter>(nh);
    jointsLPFilters_.push_back(j1LPFilter_);
    jointsLPFilters_.push_back(j2LPFilter_);
    jointsLPFilters_.push_back(j3LPFilter_);
    jointsLPFilters_.push_back(j4LPFilter_);
    jointsLPFilters_.push_back(j5LPFilter_);
    jointsLPFilters_.push_back(j6LPFilter_);

    ps5Joy_ = std::make_shared<joy::PS5Joy>(nh);
  }

  ~rmInterface() = default;
  void moveJ(std::vector<double>& joint_angles, double speed);
  void moveJPos(std::vector<double>& joint_angles, double speed);
  void moveToZero()
  {
    std::vector<double> zero_positions = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double default_speed = 0.5;

    ROS_INFO("Moving to zero position with default speed %.1f", default_speed);

    moveJ(zero_positions, default_speed);
  }
  void servoControl(double x, double y, double z, double roll, double pitch, double yaw, double speed);
  void moveToPoseWithQuaternion(double x, double y, double z, double x_orient, double y_orient, double z_orient,
                                double w_orient, double speed);
  void moveToPoseWithEuler(double x, double y, double z, double roll, double pitch, double yaw, double speed);

  std::shared_ptr<joy::PS5Joy> ps5Joy_;
  std::shared_ptr<LowPassFilter> j1LPFilter_, j2LPFilter_, j3LPFilter_, j4LPFilter_, j5LPFilter_, j6LPFilter_;
  std::vector<std::shared_ptr<LowPassFilter>> jointsLPFilters_;
  sensor_msgs::JointState jointStates_, lastJointStates_;

private:
  // Callback function for joint state updates
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (size_t i = 0; i < jointStates_.name.size(); ++i)
    {
      auto it = std::find(msg->name.begin(), msg->name.end(), jointStates_.name[i]);
      if (it != msg->name.end())
      {
        int index = std::distance(msg->name.begin(), it);
        jointStates_.position[i] = msg->position[index];
        jointStates_.velocity[i] = 0.;
        //        jointStates_.effort[i] = 0.;
        //        jointStates_.effort[i] =
        //            (jointStates_.position[i] - lastJointStates_.position[i]) / (ros::Time::now() - lastTime_).toSec();
        //
        //        Duration time_span = std::chrono::duration_cast<Duration>(Clock::now() - lastClockTime_);
        //        double dt = ros::Duration(time_span.count()).toSec();
        //        double computerVel = (jointStates_.position[i] - lastJointStates_.position[i]) / dt;
        //        lowPassFilter_->input(computerVel, ros::Time::now());
        //        jointStates_.velocity[i] = lowPassFilter_->output();
        //        lastTime_ = ros::Time::now();
        //        lastClockTime_ = Clock::now();
        //        lastJointStates_.position[i] = jointStates_.position[i];

        //        jointStates_.velocity[i] = msg->velocity[index];
        //        jointStates_.effort[i] = msg->effort[index];
      }
    }
  }

  // Callback function for pose updates
  void poseStateCallback(const geometry_msgs::Pose::ConstPtr& msg)
  {
    EePose_ = *msg;
  }

  void planStateCallback(const rm_msgs::Plan_State::ConstPtr& msg)
  {
    //    // 将接收到的消息打印出来，显示机械臂是否完成运动
    //    if (msg->state)
    //    {
    //      ROS_INFO("*******Plan State OK");
    //    }
    //    else
    //    {
    //      ROS_INFO("*******Plan State Fail");
    //    }
  }

  // ROS Subscribers
  ros::Subscriber jointStateSub_;
  ros::Subscriber poseStateSub_;
  ros::Subscriber planStateSub_;
  // ROS Publisher
  ros::Publisher moveJPPub_;
  rm_msgs::MoveJ_P moveJPTargetPose_;
  ros::Publisher moveJPub_;
  rm_msgs::JointPos jointPosCmd_;
  ros::Publisher moveJPosPub_;

  // Data members
  geometry_msgs::Pose EePose_;
  ros::Time lastTime_;
  Clock::time_point lastClockTime_;
};
}  // namespace rm_interface
