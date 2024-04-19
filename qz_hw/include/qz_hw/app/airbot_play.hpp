#ifndef _AIRBOTPLAY_HPP_
#define _AIRBOTPLAY_HPP_

#include <iostream>
#include <mutex>
#include <stdint.h>

#include <qz_hw/hybrid_force.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <drivers/can.hpp>

class AirbotPlay {
public:
  AirbotPlay(ros::NodeHandle &nh, CanChannel_e can_channel = CAN0,
             bool is_teacher = false, bool with_gripper = false);
  ~AirbotPlay();

  void Handle();

private:
  bool teacher_arm_;

  bool with_gripper_;

  uint32_t arm_cmd_timeout_cnt_;

  bool arm_high_level_offline_;

  std::mutex cmd_arm_mutex_;

  ros::Publisher arm_state_pub_;
  ros::Subscriber arm_cmd_sub_;

  sensor_msgs::JointState arm_state_msg_;

  struct {
    float arm_t_p[7], arm_t_v[7], arm_f_tau[7];
    float arm_kp[7], arm_kd[7];
    int arm_exchange_num = 0;
    void resetCommand() {
      std::fill(arm_t_p, arm_t_p + 7, 0.0f);
      std::fill(arm_t_v, arm_t_v + 7, 0.0f);
      std::fill(arm_f_tau, arm_f_tau + 7, 0.0f);
      std::fill(arm_kp, arm_kp + 7, 0.0f);
      std::fill(arm_kd, arm_kd + 7, 1.5f);
    }
  } arm_cmd_;

  void arm_update_();

  void arm_cmd_handler_();

  void armCmdCallback_(const qz_hw::hybrid_force::ConstPtr &joint_cmd);
};

#endif