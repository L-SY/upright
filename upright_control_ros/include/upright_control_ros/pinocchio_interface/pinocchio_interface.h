//
// Created by lsy on 23-10-30.
//

#pragma once

#include <pinocchio/parsers/sample-models.hpp>
// must add fwd.hpp before other .h for solve the conflict between ROS and
// pinocchio https://github.com/wxmerkt/pinocchio_ros_example
#include <hardware_interface/joint_state_interface.h>
#include <pinocchio/algorithm/dynamics.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "generally_hw/hardware_interface/HybridJointInterface.h"
#include "ros_param.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

namespace pinocchio_interface {
class PinocchioInterface {
public:
  inline bool init(ros::NodeHandle &controller_nh);
  inline void pubDynamics();
  template <class JointStateHandle>
  inline void computerInverseDynamics(std::vector<JointStateHandle> jnt_states);
  inline void computerInverseDynamics(std::vector<double> jnt_pos,
                                      std::vector<double> jnt_vel,
                                      std::vector<double> jnt_tau);
  bool send_tau_ = false;
  double ff_effort_scale_;
  Eigen::VectorXd tau_{}, tau_without_a_{}, tau_without_a_v_{};
  Eigen::VectorXd q_{}, v_{}, last_v_{}, a_{}, zero_{};

private:
  pinocchio::Model model_;
  std::string urdf_filename_;
  pinocchio::Data pinocchio_data_;

  ros::Time last_time_{};
  ros::NodeHandle node_;
  ros::Publisher error_pub_, tau_pub_, tau_without_a_pub_, tau_without_a_v_pub_,
      tau_exe_pub_, a_pub_;
  std_msgs::Float64MultiArray tau_error_msg_{}, tau_msg_{},
      tau_without_a_msg_{}, tau_without_a_v_msg_{}, tau_exe_msg_{}, a_msg_{};

  int num_hw_joints_;
};

bool PinocchioInterface::init(ros::NodeHandle &dynamics_nh) {
  if (!dynamics_nh.hasParam("urdf_filename")) {
    ROS_ERROR_STREAM("NO URDF FILE PATH");
    return false;
  } else {
    dynamics_nh.getParam("urdf_filename", urdf_filename_);
    pinocchio::urdf::buildModel(urdf_filename_, model_);
    pinocchio_data_ = pinocchio::Data(model_);
    num_hw_joints_ =
        (int)model_.joints.size() - 1; // -1 is for remove the "universe"
  }
  if (!dynamics_nh.hasParam("send_tau"))
    send_tau_ = false;
  else
    dynamics_nh.getParam("send_tau", send_tau_);
  if (!dynamics_nh.hasParam("ff_effort_scale")) {
    ff_effort_scale_ = 0.;
    ROS_INFO_STREAM("The ff_effort_scale is set to zero");
  } else {
    dynamics_nh.getParam("ff_effort_scale", ff_effort_scale_);
    ROS_INFO_STREAM("The ff_effort_scale is set to " << ff_effort_scale_);
  }

  //  Init the dynamics relative var.
  zero_.resize(num_hw_joints_);
  tau_.resize(num_hw_joints_);
  tau_without_a_.resize(num_hw_joints_);
  tau_without_a_v_.resize(num_hw_joints_);
  q_.resize(num_hw_joints_);
  v_.resize(num_hw_joints_);
  last_v_.resize(num_hw_joints_);
  a_.resize(num_hw_joints_);

  //  Init the publisher relative var.
  tau_error_msg_.data.resize(num_hw_joints_);
  tau_msg_.data.resize(num_hw_joints_);
  tau_exe_msg_.data.resize(num_hw_joints_);
  tau_without_a_msg_.data.resize(num_hw_joints_);
  tau_without_a_v_msg_.data.resize(num_hw_joints_);
  a_msg_.data.resize(num_hw_joints_);
  a_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("a", 1);
  error_pub_ =
      dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_error", 1);
  tau_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau", 1);
  tau_exe_pub_ =
      dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_exe", 1);
  tau_without_a_pub_ =
      dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_without_a", 1);
  tau_without_a_v_pub_ =
      dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_without_a_v", 1);
  return true;
}
void PinocchioInterface::pubDynamics() {
  //  Pub all data
  error_pub_.publish(tau_error_msg_);
  tau_pub_.publish(tau_msg_);
  tau_exe_pub_.publish(tau_exe_msg_);
  tau_without_a_pub_.publish(tau_without_a_msg_);
  tau_without_a_v_pub_.publish(tau_without_a_v_msg_);
  a_pub_.publish(a_msg_);
}

template <class JointStateHandle>
void PinocchioInterface::computerInverseDynamics(
    std::vector<JointStateHandle> jnt_states) {
  for (int i = 0; i < (int)tau_.size(); ++i) {
    q_(i) = jnt_states[i].getPosition();
    v_(i) = jnt_states[i].getVelocity();
    double a_ = (v_(i) - last_v_[i]) / (ros::Time::now() - last_time_).toSec();
    last_v_[i] = v_(i);
    zero_(i) = 0.;
  }
  last_time_ = ros::Time::now();
  //  Use RNEA to computer inverse dynamics
  tau_ = pinocchio::rnea(model_, pinocchio_data_, q_, v_, a_);
  tau_without_a_ = pinocchio::rnea(model_, pinocchio_data_, q_, v_, zero_);
  tau_without_a_v_ = pinocchio::rnea(model_, pinocchio_data_, q_, zero_, zero_);
  for (int i = 0; i < (int)tau_.size(); ++i) {
    tau_[i] *= ff_effort_scale_;
    tau_without_a_[i] *= ff_effort_scale_;
    tau_without_a_v_[i] *= ff_effort_scale_;
    tau_error_msg_.data[i] = tau_(i) - jnt_states[i].getEffort();
    tau_msg_.data[i] = tau_(i);
    tau_without_a_msg_.data[i] = tau_without_a_(i);
    tau_without_a_v_msg_.data[i] = tau_without_a_v_(i);
    a_msg_.data[i] = a_(i);
  }
}

void PinocchioInterface::computerInverseDynamics(std::vector<double> jnt_pos,
                                                 std::vector<double> jnt_vel,
                                                 std::vector<double> jnt_tau) {
  for (int i = 0; i < (int)tau_.size(); ++i) {
    q_(i) = jnt_pos[i];
    v_(i) = jnt_vel[i];
    double a_ = (v_(i) - last_v_[i]) / (ros::Time::now() - last_time_).toSec();
    last_v_[i] = v_(i);
    zero_(i) = 0.;
  }
  last_time_ = ros::Time::now();
  //  Use RNEA to computer inverse dynamics
  tau_ = pinocchio::rnea(model_, pinocchio_data_, q_, v_, a_);
  tau_without_a_ = pinocchio::rnea(model_, pinocchio_data_, q_, v_, zero_);
  tau_without_a_v_ = pinocchio::rnea(model_, pinocchio_data_, q_, zero_, zero_);
  for (int i = 0; i < (int)tau_.size(); ++i) {
    tau_[i] *= ff_effort_scale_;
    tau_without_a_[i] *= ff_effort_scale_;
    tau_without_a_v_[i] *= ff_effort_scale_;
    tau_error_msg_.data[i] = tau_(i) - jnt_tau[i];
    tau_msg_.data[i] = tau_(i);
    tau_without_a_msg_.data[i] = tau_without_a_(i);
    tau_without_a_v_msg_.data[i] = tau_without_a_v_(i);
    a_msg_.data[i] = a_(i);
  }
}

} // namespace pinocchio_interface