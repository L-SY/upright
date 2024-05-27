//
// Created by lsy on 24-5-27.
//

#include "diablo_hw/DiabloHW.h"

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

namespace diablo_hw
{
bool DiabloHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!GenerallyHW::init(root_nh, robot_hw_nh))
  {
    return false;
  }
  setupJoints();
  setupTopic(robot_hw_nh);
  ROS_INFO_STREAM("Diablo hw Init Finish!");
  return true;
}

void DiabloHW::read(const ros::Time& time, const ros::Duration& period)
{
  is_reading_ = true;
}

void DiabloHW::write(const ros::Time& time, const ros::Duration& period)
{
  is_writing_ = true;
  geometry_msgs::Twist diabloCmd;
  diabloCmd.linear.x = velocityJointInterface_.getHandle("diabloX").getCommand();
  diabloCmd.angular.z = velocityJointInterface_.getHandle("diabloYaw").getCommand();
  diabloMotorPub_.publish(diabloCmd);
  is_writing_ = false;
}

bool DiabloHW::setupTopic(ros::NodeHandle& nh)
{
  diabloMotorPub_ = nh.advertise<geometry_msgs::Twist>("/diablo_cmd", 1);
  diabloOdomSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/diablo_odom", 1, &DiabloHW::diabloOdomCallBack, this);
  optimizedStateTrajectorySub_ = nh.subscribe<ocs2_msgs::mpc_flattened_controller>(
      "/mobile_manipulator_mpc_policy", 1, &DiabloHW::optimizedStateTrajectoryCallBack, this);
  return true;
}

bool DiabloHW::setupJoints()
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

}  // namespace diablo_hw

PLUGINLIB_EXPORT_CLASS(diablo_hw::DiabloHW, hardware_interface::RobotHW)