//
// Created by lsy on 24-3-7.
//

#include "mobile_manipulator_hw/MobileManipulatorHW.h"

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>

namespace generally {
    bool MobileManipulatorHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
        if (!GenerallyHW::init(root_nh, robot_hw_nh)) {
            return false;
        }
        setupJoints();
        setupTopic(robot_hw_nh);
        ROS_INFO_STREAM("HW Init Finish!");
        return true;
    }

    void MobileManipulatorHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
        is_reading_ = true;
    }

    void MobileManipulatorHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
        is_writing_ = true;
        std_msgs::Float64MultiArray command;
        command.data = {0., 0., 0., 0., 0.,0.};
        for (const auto& joint : effort_joint_handles_) {
            if (joint.getName().find("shoulder_pan") != std::string::npos) {
                command.data[0] = joint.getCommand();
            } else if (joint.getName().find("shoulder_lift") != std::string::npos) {
                command.data[1] = joint.getCommand();
            } else if (joint.getName().find("elbow") != std::string::npos) {
                command.data[2] = joint.getCommand();
            } else if (joint.getName().find("wrist_1") != std::string::npos) {
                command.data[3] = joint.getCommand();
            } else if (joint.getName().find("wrist_2") != std::string::npos) {
                command.data[4] = joint.getCommand();
            } else if (joint.getName().find("wrist_3") != std::string::npos) {
                command.data[5] = joint.getCommand();
            }
        }

        rosMotorCmdPub_.publish(command);
        is_writing_ = false;
    }
    bool MobileManipulatorHW::setupTopic(ros::NodeHandle& nh) {
        rosMotorCmdPub_ = nh.advertise<std_msgs::Float64MultiArray>("/ros_motor_cmd",1);
        webotsMotorPosSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/webots_motor_pos",1,&MobileManipulatorHW::webotMotorPosCallback,
                                                                    this);
        webotsMotorVelSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/webots_motor_vel",1,&MobileManipulatorHW::webotMotorVelCallback,
                                                                    this);
        webotsMotorTorSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/webots_motor_tor",1,&MobileManipulatorHW::webotMotorTorCallback,
                                                                    this);
    }
    bool MobileManipulatorHW::setupJoints() {
        for (const auto& joint : urdfModel_->joints_) {
            int joint_index = 0;
            if (joint.first.find("shoulder_pan_joint") != std::string::npos) {
                joint_index = 0;
            } else if (joint.first.find("shoulder_lift_joint") != std::string::npos) {
                joint_index = 1;
            } else if (joint.first.find("elbow_joint") != std::string::npos) {
                joint_index = 2;
            } else if (joint.first.find("wrist_1_joint") != std::string::npos) {
                joint_index = 3;
            } else if (joint.first.find("wrist_2_joint") != std::string::npos) {
                joint_index = 4;
            } else if (joint.first.find("wrist_3_joint") != std::string::npos) {
                joint_index = 5;
            } else {
                continue;
            }
            hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[joint_index].pos_,
                                                              &jointData_[joint_index].vel_,
                                                              &jointData_[joint_index].tau_);
            jointStateInterface_.registerHandle(state_handle);
            hardware_interface::JointHandle joint_handle(state_handle, &jointData_[joint_index].tauDes_);
            effortJointInterface_.registerHandle(joint_handle);
        }
        effort_joint_handles_.push_back(effortJointInterface_.getHandle("ur_arm_shoulder_pan_joint"));
        effort_joint_handles_.push_back(effortJointInterface_.getHandle("ur_arm_shoulder_lift_joint"));
        effort_joint_handles_.push_back(effortJointInterface_.getHandle("ur_arm_elbow_joint"));
        effort_joint_handles_.push_back(effortJointInterface_.getHandle("ur_arm_wrist_1_joint"));
        effort_joint_handles_.push_back(effortJointInterface_.getHandle("ur_arm_wrist_2_joint"));
        effort_joint_handles_.push_back(effortJointInterface_.getHandle("ur_arm_wrist_3_joint"));
        return true;
    }


}  // namespace ur5_webots