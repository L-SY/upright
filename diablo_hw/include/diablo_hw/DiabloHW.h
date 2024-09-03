//
// Created by lsy on 24-5-27.
//

#pragma once

#include <geometry_msgs/Twist.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

namespace diablo_hw
{
struct DiabloMotorData
{
  std::string name_;
  double pos_, vel_, tau_;  // state
  double velDes_;           // command
};

class DiabloHW : public hardware_interface::RobotHW
{
public:
  DiabloHW() = default;

  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load
   * urdf of robot. Set up transmission and joint limit. Get configuration of
   * can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  void diabloOdomCallBack(const std_msgs::Float64MultiArrayConstPtr& data)
  {
    //    the msg is build by [ x, y, z, roll, pitch, yaw, vx, vy, vz, v_roll,
    //    v_pitch, v_yaw]
    diabloMotorData[0].pos_ = 0;
    diabloMotorData[0].vel_ = 0;
    diabloMotorData[1].pos_ = 0;
    diabloMotorData[2].pos_ = 0;
    diabloMotorData[2].vel_ = 0;
    //    diabloMotorData[0].pos_ = data.get()->data[0];
    //    diabloMotorData[0].vel_ = data.get()->data[6];
    //    diabloMotorData[1].pos_ = data.get()->data[1];
    //    diabloMotorData[2].pos_ = data.get()->data[5];
    //    diabloMotorData[2].vel_ = data.get()->data[11];
  }

  void optimizedStateTrajectoryCallBack(const ocs2_msgs::mpc_flattened_controllerConstPtr& data)
  {
    receiveOptimizedStateTrajectory_ = true;
    optimizedStateTrajectory_ = *data;
  }

private:
  bool setupJoints(), is_writing_ = false, is_reading_ = false, recv_one_ = false;

  bool setupTopic(ros::NodeHandle& nh);
  DiabloMotorData diabloMotorData[3]{};
  ros::Subscriber diabloOdomSub_;
  ros::Publisher diabloMotorPub_;
  std::vector<std::string> robotMotorName_;

  // Sub optimizedStateTrajectory
  bool receiveOptimizedStateTrajectory_ = false;
  ros::Subscriber optimizedStateTrajectorySub_;
  ocs2_msgs::mpc_flattened_controller optimizedStateTrajectory_;

  hardware_interface::VelocityJointInterface
      velocityJointInterface_;  // NOLINT(misc-non-private-member-variables-in-classes)

  hardware_interface::JointStateInterface jointStateInterface_;  // NOLINT(misc-non-private-member-variables-in-classes)
};

}  // namespace diablo_hw
