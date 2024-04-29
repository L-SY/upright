//
// Created by lsy on 24-4-26.
//

#include "rm_hw/rmInterface.h"

namespace rm_interface
{
void rmInterface::moveJ(std::vector<double>& joint_angles, double speed)
{
  rm_msgs::MoveJ moveJ_Pose;

  moveJ_Pose.joint.resize(joint_angles.size());
  for (size_t i = 0; i < joint_angles.size(); ++i)
  {
    moveJ_Pose.joint[i] = joint_angles[i];
  }

  moveJ_Pose.speed = speed;
  moveJPub_.publish(moveJ_Pose);
}

void rmInterface::servoControl(double d_x, double d_y, double d_z, double d_roll, double d_pitch, double d_yaw,
                               double speed)
{
  double current_x = EePose_.position.x;
  double current_y = EePose_.position.y;
  double current_z = EePose_.position.z;
  tf2::Quaternion current_quat;
  current_quat.setX(EePose_.orientation.x);
  current_quat.setY(EePose_.orientation.y);
  current_quat.setZ(EePose_.orientation.z);
  current_quat.setW(EePose_.orientation.w);

  tf2::Matrix3x3 mat(current_quat);
  double current_roll, current_pitch, current_yaw;
  mat.getRPY(current_roll, current_pitch, current_yaw);

  double target_x = current_x + d_x;
  double target_y = current_y + d_y;
  double target_z = current_z + d_z;
  double target_roll = current_roll + d_roll;
  double target_pitch = current_pitch + d_pitch;
  double target_yaw = current_yaw + d_yaw;

  moveToPoseWithEuler(target_x, target_y, target_z, target_roll, target_pitch, target_yaw, speed);
}

void rmInterface::moveToPoseWithQuaternion(double x, double y, double z, double x_orient, double y_orient,
                                           double z_orient, double w_orient, double speed)
{
  rm_msgs::MoveJ_P moveJ_P_TargetPose;
  moveJ_P_TargetPose.Pose.position.x = x;
  moveJ_P_TargetPose.Pose.position.y = y;
  moveJ_P_TargetPose.Pose.position.z = z;
  moveJ_P_TargetPose.Pose.orientation.x = x_orient;
  moveJ_P_TargetPose.Pose.orientation.y = y_orient;
  moveJ_P_TargetPose.Pose.orientation.z = z_orient;
  moveJ_P_TargetPose.Pose.orientation.w = w_orient;
  moveJ_P_TargetPose.speed = speed;

  moveJPPub_.publish(moveJ_P_TargetPose);
}

void rmInterface::moveToPoseWithEuler(double x, double y, double z, double roll, double pitch, double yaw, double speed)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  double x_orient = quat.x();
  double y_orient = quat.y();
  double z_orient = quat.z();
  double w_orient = quat.w();
  moveToPoseWithQuaternion(x, y, z, x_orient, y_orient, z_orient, w_orient, speed);
}
}  // namespace rm_interface