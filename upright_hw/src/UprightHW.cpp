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
  swingboyHwPtr_ = std::make_shared<can_hw::CanHW>();
  swingboyHwPtr_->setCanBusThreadPriority(95);
  swingboyHwPtr_->init(root_nh, robot_hw_nh);
  setupJoints();
  setupTopic(robot_hw_nh);

  for (auto name : swingboyHwPtr_->get<hardware_interface::JointStateInterface>()->getNames())
  {
    ROS_INFO_STREAM(name);
    auto jointHandle = swingboyHwPtr_->get<hardware_interface::JointStateInterface>()->getHandle(name);
    auto effJointHandle = swingboyHwPtr_->get<hardware_interface::EffortJointInterface>()->getHandle(name);
    this->get<hardware_interface::JointStateInterface>()->registerHandle(jointHandle);
    this->get<hardware_interface::EffortJointInterface>()->registerHandle(effJointHandle);
  }
  auto effort_joint_interface = this->get<hardware_interface::JointStateInterface>();
  std::vector<std::string> names = effort_joint_interface->getNames();
  for (const auto& name : names)
    ROS_INFO_STREAM(name);
  ROS_INFO_STREAM("upright hw Init Finish!");
  return true;
}

void UprightHW::read(const ros::Time& time, const ros::Duration& period)
{
  is_reading_ = true;
  swingboyHwPtr_->read(time, period);
}

void UprightHW::write(const ros::Time& time, const ros::Duration& period)
{
  is_writing_ = true;
  geometry_msgs::Twist diabloCmd;
  diabloCmd.linear.x = velocityJointInterface_.getHandle("diabloX").getCommand();
  diabloCmd.angular.z = velocityJointInterface_.getHandle("diabloYaw").getCommand();
  diabloMotorPub_.publish(diabloCmd);
  swingboyHwPtr_->write(time, period);
  is_writing_ = false;
}

bool UprightHW::setupTopic(ros::NodeHandle& nh)
{
  diabloMotorPub_ = nh.advertise<geometry_msgs::Twist>("/diablo_cmd", 1);
  diabloOdomSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/diablo_odom", 1, &UprightHW::diabloOdomCallBack, this);
  optimizedStateTrajectorySub_ = nh.subscribe<ocs2_msgs::mpc_flattened_controller>(
      "/mobile_manipulator_mpc_policy", 1, &UprightHW::optimizedStateTrajectoryCallBack, this);
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

  return true;
}

}  // namespace generally