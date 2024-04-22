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
  ROS_INFO_STREAM("HW Init Finish!");
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
  qz_hw::hybrid_force qzCmdMsg;
  qzCmdMsg.header.stamp = ros::Time::now();
  int j = 0;
  for (auto& qzJoint : hybrid_joint_handles_)
  {
    qzCmdMsg.joint_names.push_back(hybrid_joint_handles_[i].getName());
    qzCmdMsg.effort.push_back(hybrid_joint_handles_[i].getFeedforward());
    qzCmdMsg.positions.push_back(hybrid_joint_handles_[i].getPositionDesired());
    qzCmdMsg.velocities.push_back(hybrid_joint_handles_[i].getVelocityDesired());
    qzCmdMsg.kds.push_back(hybrid_joint_handles_[i].getKd());
    qzCmdMsg.kps.push_back(hybrid_joint_handles_[i].getKp());
    j++;
  }
  qzMotorPub_.publish(qzCmdMsg);
  is_writing_ = false;
}

bool UprightHW::setupTopic(ros::NodeHandle& nh)
{
  qzMotorPub_ = nh.advertise<qz_hw::hybrid_force>("/airbot_play/joint_command", 1);
  qzJointSub_ = nh.subscribe<sensor_msgs::JointState>("/airbot_play/joint_states", 1, &UprightHW::qzHWCallBack, this);
  diabloMotorPub_ = nh.advertise<geometry_msgs::Twist>("/diablo", 1);
  diabloOdomSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/diablo_odom", 1, &UprightHW::diabloOdomCallBack, this);
  return true;
}

bool UprightHW::setupJoints()
{
  // Two vel-joint for diablo x-yaw
  diabloMotorData[0].name_ = "diabloX";
  diabloMotorData[1].name_ = "diabloYaw";
  for (auto& joint : diabloMotorData)
  {
    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_, &joint.tau_);
    jointStateInterface_.registerHandle(state_handle);
    hardware_interface::JointHandle joint_handle(state_handle, &joint.velDes_);
    velocityJointInterface_.registerHandle(joint_handle);
    velocity_joint_handles_.push_back(effortJointInterface_.getHandle(joint.name_));
  }

  // Six hybrid-joint for qz arm
  for (int i = 0; i < 6; ++i)
  {
    qzMotorData[i].name_ = "joint" + std::to_string(i);
    ROS_INFO_STREAM(qzMotorData[i].name_);
  }
  for (auto& joint : qzMotorData)
  {
    hardware_interface::JointStateHandle state_handle(joint.name_, &joint.pos_, &joint.vel_, &joint.tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(hardware_interface::HybridJointHandle(
        state_handle, &joint.posDes_, &joint.velDes_, &joint.kp_, &joint.kd_, &joint.ff_));
    hybrid_joint_handles_.push_back(hybridJointInterface_.getHandle(joint.name_));
  }

  return true;
}

}  // namespace generally