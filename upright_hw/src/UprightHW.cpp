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
  XmlRpc::XmlRpcValue motorsNameList;
  if (root_nh.getParam("/robot_def/motors_name", motorsNameList))
  {
    if (motorsNameList.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < motorsNameList.size(); ++i)
      {
        if (motorsNameList[i].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          robotMotorName_.push_back(static_cast<std::string>(motorsNameList[i]));
        }
        else
        {
          ROS_ERROR("Motor name is not a string.");
        }
      }
    }
    else
    {
      ROS_ERROR("Motors name list is not an array.");
    }
  }
  else
  {
    ROS_ERROR("Failed to get motors name from parameter server.");
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
  std_msgs::Float64MultiArray command;
  command.data.resize(effort_joint_handles_.size());
  int i = 0;
  for (const auto& joint : effort_joint_handles_)
  {
    command.data[i] = joint.getCommand();
    i++;
  }

  rosMotorCmdPub_.publish(command);
  is_writing_ = false;
}

bool UprightHW::setupTopic(ros::NodeHandle& nh)
{
  rosMotorCmdPub_ = nh.advertise<std_msgs::Float64MultiArray>("/ros_motor_cmd", 1);
  webotsMotorPosSub_ =
      nh.subscribe<std_msgs::Float64MultiArray>("/webots_motor_pos", 1, &UprightHW::webotMotorPosCallback, this);
  webotsMotorVelSub_ =
      nh.subscribe<std_msgs::Float64MultiArray>("/webots_motor_vel", 1, &UprightHW::webotMotorVelCallback, this);
  webotsMotorTorSub_ =
      nh.subscribe<std_msgs::Float64MultiArray>("/webots_motor_tor", 1, &UprightHW::webotMotorTorCallback, this);
}

bool UprightHW::setupJoints()
{
  int joint_index = 0;
  for (const auto& joint : robotMotorName_)
  {
    hardware_interface::JointStateHandle state_handle(joint, &jointData_[joint_index].pos_,
                                                      &jointData_[joint_index].vel_, &jointData_[joint_index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hardware_interface::JointHandle joint_handle(state_handle, &jointData_[joint_index].tauDes_);
    effortJointInterface_.registerHandle(joint_handle);
    effort_joint_handles_.push_back(effortJointInterface_.getHandle(joint));

    joint_index++;
  }
  return true;
}

}  // namespace generally