//
// Created by lsy on 24-4-26.
//

#pragma once

#include "geometry_msgs/Pose.h"
#include "ps5_joy/ps5Joy.h"
#include "rm_msgs/MoveC.h"
#include "rm_msgs/MoveJ.h"
#include "rm_msgs/MoveJ_P.h"
#include "rm_msgs/Plan_State.h"
#include "rm_msgs/MoveL.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
namespace rm_interface
{

class rmInterface
{
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
    }

    // Subscribe to /joint_state and /rm_driver/Pose_State
    jointStateSub_ = nh.subscribe("/joint_states", 10, &rmInterface::jointStateCallback, this);
    poseStateSub_ = nh.subscribe("rm_driver/Pose_State", 10, &rmInterface::poseStateCallback, this);
    planStateSub_ = nh.subscribe("/rm_driver/Plan_State", 10, &rmInterface::planStateCallback, this);
    moveJPPub_ = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);
    moveJPub_ = nh.advertise<rm_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);

    ps5Joy_ = std::make_shared<joy::PS5Joy>(nh);
  }

  ~rmInterface() = default;
  void moveJ(std::vector<double>& joint_angles, double speed);
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
  sensor_msgs::JointState jointStates_;

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
        jointStates_.effort[i] = 0.;
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

  // Data members
  geometry_msgs::Pose EePose_;
};
}  // namespace rm_interface
