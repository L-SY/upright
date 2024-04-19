#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include <iostream>
#include <mutex>
#include <stdint.h>

#include <qz_hw/hybrid_force.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

class Robot {
public:
  Robot(ros::NodeHandle &nh, bool arm = false, bool gripper = false);
  ~Robot();

  void Init();
  void Handle();

private:
  bool withArm_;
  bool withGripper_;

  uint32_t leg_cmd_timeout_cnt_, arm_cmd_timeout_cnt_;

  bool leg_high_level_offline_, arm_high_level_offline_;

  std::mutex cmd_leg_mutex_, cmd_arm_mutex_;

  ros::Publisher joint_state_pub_, imu_pub_, arm_state_pub_;
  ros::Subscriber joint_cmd_sub_, arm_cmd_sub_;

  sensor_msgs::JointState joint_state_msg_, arm_state_msg_;
  sensor_msgs::Imu imu_state_sim_msg_;

  struct {
    float t_p[6], t_v[6], f_tau[6];
    float kp[6], kd[6];
    int exchange_num = 0;
    void resetCommand() {
      std::fill(t_p, t_p + 6, 0.0f);
      std::fill(t_v, t_v + 6, 0.0f);
      std::fill(f_tau, f_tau + 6, 0.0f);
      std::fill(kp, kp + 6, 0.0f);
      std::fill(kd, kd + 6, 0.5f);
    }
  } leg_cmd_;

  struct {
    float arm_t_p[7], arm_t_v[7], arm_f_tau[7];
    float arm_kp[7], arm_kd[7];
    int arm_exchange_num = 0;
    void resetCommand() {
      std::fill(arm_t_p, arm_t_p + 7, 0.0f);
      std::fill(arm_t_v, arm_t_v + 7, 0.0f);
      std::fill(arm_f_tau, arm_f_tau + 7, 0.0f);
      std::fill(arm_kp, arm_kp + 7, 0.0f);
      std::fill(arm_kd, arm_kd + 7, 0.5f);
    }
  } arm_cmd_;

  float realYaw_, lastYawRead_, yawRead_;

  void imu_yaw_update_();
  void leg_update_();
  void arm_update_();

  void leg_cmd_handler_();
  void arm_cmd_handler_();
  void set_leg_zero_();

  void legCmdCallback_(const qz_hw::hybrid_force::ConstPtr &joint_cmd);
  void armCmdCallback_(const qz_hw::hybrid_force::ConstPtr &joint_cmd);
};

#endif