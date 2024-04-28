//
// Created by lsy on 24-3-7.
//

#include "upright_hw/UprightHW.h"

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

namespace generally
{
bool UprightHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!GenerallyHW::init(root_nh, robot_hw_nh))
  {
    return false;
  }
  setupJoints();
  setupTopic(robot_hw_nh);
  ROS_INFO_STREAM("upright hw Init Finish!");
  return true;
}

void UprightHW::read(const ros::Time& time, const ros::Duration& /*period*/)
{
  is_reading_ = true;
}

void UprightHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  is_writing_ = true;
  geometry_msgs::Twist diabloCmd;
  diabloCmd.linear.x = velocityJointInterface_.getHandle("diabloX").getCommand();
  diabloCmd.angular.z = velocityJointInterface_.getHandle("diabloYaw").getCommand();
  diabloMotorPub_.publish(diabloCmd);

  std_msgs::Float64MultiArray command;
  command.data.resize(effort_joint_handles_.size());
  int i = 0;
  for (const auto& joint : effort_joint_handles_)
  {
    command.data[i] = joint.getCommand();
    i++;
  }
  is_writing_ = false;
}

bool UprightHW::setupTopic(ros::NodeHandle& nh)
{
  diabloMotorPub_ = nh.advertise<geometry_msgs::Twist>("/diablo", 1);
  diabloOdomSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/diablo_odom", 1, &UprightHW::diabloOdomCallBack, this);
  return true;
}

bool UprightHW::setupJoints()
{
  // Two vel-joint for diablo x-yaw
  diabloMotorData[0].name_ = "diabloX";
  diabloMotorData[1].name_ = "diabloY";
  diabloMotorData[2].name_ = "diabloYaw";
  for (auto& joint : diabloMotorData)
  {
    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_, &joint.tau_);
    jointStateInterface_.registerHandle(state_handle);
    hardware_interface::JointHandle joint_handle(state_handle, &joint.velDes_);
    velocityJointInterface_.registerHandle(joint_handle);
    velocity_joint_handles_.push_back(velocityJointInterface_.getHandle(joint.name_));
  }

  // Six hybrid-joint for rm arm
  for (int i = 0; i < 6; ++i)
    rmMotorData[i].name_ = "joint" + std::to_string(i + 1);
  for (auto& joint : rmMotorData)
  {
    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_, &joint.tau_);
    jointStateInterface_.registerHandle(state_handle);
    positionJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &joint.posDes_));
    position_joint_handles_.push_back(positionJointInterface_.getHandle(joint.name_));
  }

  return true;
}

}  // namespace generally