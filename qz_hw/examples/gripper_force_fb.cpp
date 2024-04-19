#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <airbot/motor_driver.hpp>
#include <drivers/can.hpp>
#include <qz_hw/hybrid_force.h>

#define max_timeout_ms (100)

struct JointCmd {
  int32_t id;
  float kp;
  float kd;
  float t_p;
  float t_v;
  float f_tau;
  void setzero() {
    kp = 0.0f;
    kd = 0.0f;
    t_p = 0.0f;
    t_v = 0.0f;
    f_tau = 0.0f;
  }
};

std::unique_ptr<arm::MotorDriver> control_driver;
std::unique_ptr<arm::MotorDriver> gripper_driver;

int main(int argc, char **argv) {
  ros::init(argc, argv, "gripper_force_fb");

  ros::NodeHandle node;

  int can_device;
  ros::param::get("~can_device", can_device);
  ROS_INFO("can_device: %d", can_device);
  CanChannel_e can_channel = static_cast<CanChannel_e>(can_device);

  std::mutex cmd_mutex;

  JointCmd handle_cmd, gripper_cmd;
  handle_cmd.id = 8;
  gripper_cmd.id = 7;

  control_driver =
      arm::MotorDriver::MotorCreate(can_channel, handle_cmd.id, "newteacher");
  gripper_driver =
      arm::MotorDriver::MotorCreate(can_channel, gripper_cmd.id, "gripper");

  auto can_callback = [](can_frame_t &frame) {
    if ((frame.can_id == gripper_driver->get_motor_id()) ||
        (frame.data[0] == gripper_driver->get_motor_id() &&
         (frame.can_id == 0 || frame.can_id == 0x700 || frame.can_id == 0x701 ||
          frame.can_id == 0x702 || frame.can_id == 0x703))) {
      gripper_driver->CanRxMsgCallback(frame, 0);
    } else {
      int ret_cmd = control_driver->get_RET_CMD();
      int ret_cmd_id = control_driver->get_motor_id() | ret_cmd << 7;
      if ((frame.can_id == control_driver->get_motor_id()) ||
          (frame.can_id == 0X7FF &&
           frame.data[1] == control_driver->get_motor_id()) ||
          (frame.can_id == ret_cmd_id)) {
        control_driver->CanRxMsgCallback(frame, 0);
      }
    }
  };
  Can::can_handle.SetCanCallback(can_channel, can_callback);

  control_driver->MotorInit();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  control_driver->set_motor_control_mode(arm::MotorDriver::MIT);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  gripper_driver->MotorInit();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  gripper_driver->set_motor_control_mode(arm::MotorDriver::MIT);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  ros::Publisher arm_state_pub;

  arm_state_pub =
      node.advertise<sensor_msgs::JointState>("/gripper/joint_states", 3);
  sensor_msgs::JointState joint_states_msg;

  joint_states_msg.name = {"handle", "gripper"};
  joint_states_msg.position.resize(2);
  joint_states_msg.velocity.resize(2);
  joint_states_msg.effort.resize(2);

  auto cmd_recv_time = std::chrono::high_resolution_clock::now();
  std::this_thread::sleep_for(std::chrono::milliseconds(max_timeout_ms));

  ros::Subscriber arm_cmd_sub;
  arm_cmd_sub = node.subscribe<qz_hw::hybrid_force>(
      "/gripper/joint_command", 3,
      [&](const qz_hw::hybrid_force::ConstPtr &joint_cmd) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        cmd_recv_time = std::chrono::high_resolution_clock::now();
        handle_cmd.kp = joint_cmd->kps[0];
        handle_cmd.kd = joint_cmd->kds[0];
        handle_cmd.t_p = joint_cmd->positions[0];
        handle_cmd.t_v = joint_cmd->velocities[0];
        handle_cmd.f_tau = joint_cmd->effort[0];
        gripper_cmd.kp = joint_cmd->kps[1];
        gripper_cmd.kd = joint_cmd->kds[1];
        gripper_cmd.t_p = joint_cmd->positions[1];
        gripper_cmd.t_v = joint_cmd->velocities[1];
        gripper_cmd.f_tau = joint_cmd->effort[1];
      });

  std::thread joint_state_update_thread = std::thread([&]() {
    auto start = std::chrono::high_resolution_clock::now();
    while (ros::ok()) {
      start = std::chrono::high_resolution_clock::now();
      joint_states_msg.header.stamp = ros::Time::now();
      joint_states_msg.position[0] = control_driver->get_motor_pos();
      joint_states_msg.position[1] = gripper_driver->get_motor_pos();
      joint_states_msg.velocity[0] = control_driver->get_motor_spd();
      joint_states_msg.velocity[1] = gripper_driver->get_motor_spd();
      joint_states_msg.effort[0] = control_driver->get_motor_current();
      joint_states_msg.effort[1] = gripper_driver->get_motor_current();
      arm_state_pub.publish(joint_states_msg);

      bool lost_connection = false;
      if (control_driver->get_response_count() > 100) {
        ROS_FATAL(
            "arm control motor [%d] {1-8} lost connection, cmd count [%d]",
            control_driver->get_motor_id(),
            control_driver->get_response_count());
        lost_connection = true;
      }

      if (gripper_driver->get_response_count() > 100) {
        ROS_FATAL(
            "arm grasper motor [%d] {1-8} lost connection, cmd count [%d]",
            gripper_driver->get_motor_id(),
            gripper_driver->get_response_count());
        lost_connection = true;
      }

      if (lost_connection) {
        ROS_FATAL("motor lost connection, exiting");
        ros::shutdown();
        break;
      }

      std::this_thread::sleep_until(start + std::chrono::milliseconds(10));
    }
    ROS_WARN("<1> state update thread destroyed");
  });

  std::thread joint_cmd_update_thread = std::thread([&]() {
    auto start = std::chrono::high_resolution_clock::now();
    while (ros::ok()) {
      start = std::chrono::high_resolution_clock::now();
      {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        control_driver->MotorMitModeCmd(handle_cmd.t_p, handle_cmd.t_v,
                                        handle_cmd.kp, handle_cmd.kd,
                                        handle_cmd.f_tau);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        gripper_driver->MotorMitModeCmd(gripper_cmd.t_p, gripper_cmd.t_v,
                                        gripper_cmd.kp, gripper_cmd.kd,
                                        gripper_cmd.f_tau);
      }
      std::this_thread::sleep_until(start + std::chrono::milliseconds(10));
    }
    ROS_WARN("<2> command update thread destroyed");
  });

  bool high_control_module_online = false;
  auto start_rosspin = std::chrono::high_resolution_clock::now();
  while (ros::ok()) {
    start_rosspin = std::chrono::high_resolution_clock::now();
    if (high_control_module_online) {
      if ((start_rosspin - cmd_recv_time) >
          std::chrono::milliseconds(max_timeout_ms)) {
        ROS_WARN("high control module offline");
        high_control_module_online = false;
        {
          std::lock_guard<std::mutex> lock(cmd_mutex);
          handle_cmd.setzero();
          gripper_cmd.setzero();
        }
      }
    } else if ((start_rosspin - cmd_recv_time) <
               std::chrono::milliseconds(max_timeout_ms)) {
      ROS_INFO("high control module online");
      high_control_module_online = true;
    }
    ros::spinOnce();
    std::this_thread::sleep_until(start_rosspin +
                                  std::chrono::milliseconds(10));
  }

  // std::cout << "cmd_recv_time - start_rosspin = " << (cmd_recv_time -
  // start_rosspin).count() << std::endl; ros::shutdown();

  joint_state_update_thread.join();
  joint_cmd_update_thread.join();

  handle_cmd.setzero();
  gripper_cmd.setzero();

  int tx_cnt = 10;
  while (tx_cnt--) {
    control_driver->MotorMitModeCmd(handle_cmd.t_p, handle_cmd.t_v,
                                    handle_cmd.kp, handle_cmd.kd,
                                    handle_cmd.f_tau);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    gripper_driver->MotorMitModeCmd(gripper_cmd.t_p, gripper_cmd.t_v,
                                    gripper_cmd.kp, gripper_cmd.kd,
                                    gripper_cmd.f_tau);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  control_driver->MotorDeInit();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  gripper_driver->MotorDeInit();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  ROS_WARN("<All> gripper node shutdown");
  return 0;
}
