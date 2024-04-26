//
// Created by lsy on 24-4-26.
//

#pragma once

#include "geometry_msgs/Pose.h"
#include "ps5_joy/ps5Joy.h"
#include "rm_msgs/MoveC.h"
#include "rm_msgs/MoveJ.h"
#include "rm_msgs/MoveJ_P.h"
#include "rm_msgs/MoveL.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"

namespace rm_interface {

class rmInterface {
public:
  rmInterface(ros::NodeHandle &nh) {
    // Initialize jointStates_ with empty JointState objects named joint1 to
    // joint6
    for (int i = 1; i <= 6; ++i) {
      jointStates_.name.push_back("joint" + std::to_string(i));
      jointStates_.position.push_back(0.);
      jointStates_.velocity.push_back(0.);
      jointStates_.effort.push_back(0.);
    }

    // Subscribe to /joint_state and /rm_driver/Pose_State
    joint_state_sub_ =
        nh.subscribe("joint_state", 10, &rmInterface::jointStateCallback, this);
    pose_state_sub_ = nh.subscribe("rm_driver/Pose_State", 10,
                                   &rmInterface::poseStateCallback, this);
    ps5Joy_ = std::make_shared<joy::PS5Joy>(nh);
  }

  ~rmInterface() = default;

private:
  // Callback function for joint state updates
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    for (size_t i = 0; i < jointStates_.name.size(); ++i) {
      auto it =
          std::find(msg->name.begin(), msg->name.end(), jointStates_.name[i]);
      if (it != msg->name.end()) {
        int index = std::distance(msg->name.begin(), it);
        jointStates_.position[index] = msg->position[index];
        jointStates_.velocity[index] = msg->velocity[index];
        jointStates_.effort[index] = msg->effort[index];
      }
    }
  }

  // Callback function for pose updates
  void poseStateCallback(const geometry_msgs::Pose::ConstPtr &msg) {
    pose_ = *msg;
  }

  std::shared_ptr<joy::PS5Joy> ps5Joy_;
  // ROS Subscribers
  ros::Subscriber joint_state_sub_;
  ros::Subscriber pose_state_sub_;

  // Data members
  geometry_msgs::Pose pose_;
  sensor_msgs::JointState jointStates_;
};
} // namespace rm_interface
