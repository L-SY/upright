
//
// Created by lsy on 24-3-7.
//

#pragma once

#include <generally_hw/GenerallyHW.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <qz_hw/hybrid_force.h>

namespace generally
{
struct DiabloMotorData
{
  std::string name_;
  double pos_, vel_, tau_;  // state
  double velDes_;           // command
};

struct QzMotorData
{
  std::string name_;
  double pos_, vel_, tau_;  // state
  double posDes_, velDes_, kp_, kd_, ff_;
};

class UprightHW : public GenerallyHW
{
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

  void qzHWCallBack(const sensor_msgs::JointStateConstPtr& data)
  {
    int i = 0;
    for (auto& qzJoint : hybrid_joint_handles_)
    {
      qzMotorData[i].pos_ = data->position[i];
      qzMotorData[i].vel_ = data->velocity[i];
      qzMotorData[i].tau_ = data->effort[i];
      i++;
    }
  }

  void diabloOdomCallBack(const std_msgs::Float64MultiArrayConstPtr& data)
  {
    // TODO ： Make it right
    diabloMotorData[0].pos_ = data.get()->data[0];
    diabloMotorData[0].vel_ = data.get()->data[0];
    diabloMotorData[1].pos_ = data.get()->data[1];
    diabloMotorData[1].vel_ = data.get()->data[1];
  }

private:
  bool setupJoints(), is_writing_ = false, is_reading_ = false, recv_one_ = false;

  bool setupTopic(ros::NodeHandle& nh);
  std::vector<double> qzJointPos_, qzJointVel_, qzJointEff_;
  const int jointNum = 8;
  DiabloMotorData diabloMotorData[3]{};
  QzMotorData qzMotorData[6]{};  // NOLINT(modernize-avoid-c-arrays)
  ros::Subscriber qzJointSub_, diabloOdomSub_;
  ros::Publisher qzMotorPub_, diabloMotorPub_;
  std::vector<std::string> robotMotorName_;
};

}  // namespace generally