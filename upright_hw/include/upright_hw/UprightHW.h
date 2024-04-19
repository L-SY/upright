
//
// Created by lsy on 24-3-7.
//

#pragma once

#include <generally_hw/GenerallyHW.h>
#include <std_msgs/Float64MultiArray.h>

namespace generally {
struct UprightMotorData {
  double pos_, vel_, tau_;          // state
  double posDes_, velDes_, tauDes_; // command
};

class UprightHW : public GenerallyHW {
public:
  UprightHW() = default;

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
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time &time, const ros::Duration &period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time &time, const ros::Duration &period) override;

  void webotMotorPosCallback(const std_msgs::Float64MultiArrayConstPtr &data) {
    if (is_reading_)
      for (int i = 0; i < jointNum; ++i)
        jointData_[i].pos_ = data->data[i];
  }

  void webotMotorVelCallback(const std_msgs::Float64MultiArrayConstPtr &data) {
    if (is_reading_)
      for (int i = 0; i < jointNum; ++i)
        jointData_[i].vel_ = data->data[i];
  }

  void webotMotorTorCallback(const std_msgs::Float64MultiArrayConstPtr &data) {
    if (is_reading_)
      for (int i = 0; i < jointNum; ++i)
        jointData_[i].tau_ = data->data[i];
  }

private:
  bool setupJoints(), is_writing_ = false, is_reading_ = false;

  bool setupTopic(ros::NodeHandle &nh);

  const int jointNum = 8;
  UprightMotorData jointData_[8]{}; // NOLINT(modernize-avoid-c-arrays)
  ros::Subscriber webotsMotorPosSub_;
  ros::Subscriber webotsMotorVelSub_;
  ros::Subscriber webotsMotorTorSub_;
  ros::Publisher rosMotorCmdPub_;
  std::vector<std::string> robotMotorName_;
};

} // namespace generally