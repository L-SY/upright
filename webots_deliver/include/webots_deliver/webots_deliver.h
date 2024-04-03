//
// Created by lsy on 24-3-1.
//
#pragma once

#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <ros/ros.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <unordered_map>
#include <std_msgs/Float64MultiArray.h>

#include "webots_deliver/def/ur5_def.h"
#include "webots_deliver/def/ddt_def.h"
#include "webots_deliver/def/ridgeback_ur5_def.h"

namespace webots_deliver {
    class RosWebotsDeliver {
    public:
        RosWebotsDeliver(ros::NodeHandle nh) {
            XmlRpc::XmlRpcValue motorsNameList, posSensorNameList;
            if (nh.getParam("/robot_def/ridgeback_ur5/motors_name", motorsNameList) &&
                nh.getParam("/robot_def/ridgeback_ur5/pos_sensor_name", posSensorNameList)) {
                if (motorsNameList.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                    for (int i = 0; i < motorsNameList.size(); ++i) {
                        if (motorsNameList[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
                            robotMotorName_.push_back(static_cast<std::string>(motorsNameList[i]));
                            robotSensorName_.push_back(static_cast<std::string>(posSensorNameList[i]));
                        } else {
                            ROS_ERROR("Motor name is not a string.");
                        }
                    }
                } else {
                    ROS_ERROR("Motors name list is not an array.");
                }
            } else {
                ROS_ERROR("Failed to get motors name from parameter server.");
            }
            motorNum_ = (int) robotMotorName_.size();
            initDeliver(nh);
        };

        ~RosWebotsDeliver() = default;

        void initVelocityMode(webots::Robot *robot) {
            rosMotorCmd_.resize(motorNum_);
//            rosMotorCmd_ = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
            enablePositionSensor(robot, 100);
            setAllVelocityMode(robot);
        }

        void initDeliver(ros::NodeHandle nh) {
            observation_ = nh.subscribe<ocs2_msgs::mpc_flattened_controller>("/mobile_manipulator_mpc_policy", 10,
                                                                             &RosWebotsDeliver::MPCPolicyCallback,
                                                                             this);
            rosMotorCmdSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/ros_motor_cmd", 10,
                                                                        &RosWebotsDeliver::rosMotorCmdCallback, this);
            webotsMotorPosPub_ = nh.advertise<std_msgs::Float64MultiArray>("/webots_motor_pos", 1);
            webotsMotorVelPub_ = nh.advertise<std_msgs::Float64MultiArray>("/webots_motor_vel", 1);
            webotsMotorTorPub_ = nh.advertise<std_msgs::Float64MultiArray>("/webots_motor_tor", 1);
            mpc_policy_.inputTrajectory.clear();
            for (int i = 0; i < motorNum_; ++i)
                rosMotorCmd_.push_back(0.);
        }

        void enablePositionSensor(webots::Robot *robot, int samplingPeriod = 1000) {
            for (int i = 0; i < robotSensorName_.size(); ++i) {
                webots::PositionSensor *ps = robot->getPositionSensor(robotSensorName_[i]);
                ps->enable(samplingPeriod);
            }
        }

        void setSingleVelocityMode(webots::Robot *robot, int index) {
            webots::Motor *motor = robot->getMotor(robotMotorName_[index]);
            motor->setPosition(INFINITY);
        }

        void setSingleVelocityMode(webots::Robot *robot, std::string motorName) {
            webots::Motor *motor = robot->getMotor(motorName);
            motor->setPosition(INFINITY);
        }

        void setAllVelocityMode(webots::Robot *robot) {
            for (int i = 0; i < motorNum_; ++i) {
                webots::Motor *motor = robot->getMotor(robotMotorName_[i]);
                motor->setPosition(INFINITY);
            }
        }

        std::unordered_map<std::string, double> getMotorAllState(webots::Robot *robot, const std::string &motor_name) {
            std::unordered_map<std::string, double> stateMap;

            webots::Motor *motor = robot->getMotor(motor_name);
            if (motor != nullptr) {
                double vel = motor->getVelocity();
                double pos = motor->getPositionSensor()->getValue();
                double tor = motor->getTorqueFeedback();

                stateMap["velocity"] = vel;
                stateMap["position"] = pos;
                stateMap["torque"] = tor;
            }
            return stateMap;
        }

        std::unordered_map<std::string, double> getMotorAllState(webots::Robot *robot, int index) {
            std::unordered_map<std::string, double> stateMap;

            webots::Motor *motor = robot->getMotor(robotMotorName_[index]);
            if (motor != nullptr) {
                double vel = motor->getVelocity();
                double pos = motor->getPositionSensor()->getValue();
                double tor = motor->getTorqueFeedback();

                stateMap["velocity"] = vel;
                stateMap["position"] = pos;
                stateMap["torque"] = tor;
            }
            return stateMap;
        }

        std::vector<double> getMotorPositions(webots::Robot *robot, bool print = false) {
            std::vector<double> motor_positions;
            for (int i = 0; i < motorNum_; ++i) {
                webots::PositionSensor *ps = robot->getPositionSensor(robotSensorName_[i]);
                motor_positions.push_back(ps->getValue());
                if (print)
                    std::cout << robotSensorName_[i] << "  " << ps->getValue() << std::endl;
            }
            return motor_positions;
        }

        int getMotorInfo(webots::Robot *robot) {
            int motorCount = 0;
            int numberOfDevices = robot->getNumberOfDevices();
            for (int i = 0; i < numberOfDevices; i++) {
                webots::Device *device = robot->getDeviceByIndex(i);
                std::string deviceName = device->getName();
                if (device->getNodeType() == webots::Node::ROTATIONAL_MOTOR ||
                    device->getNodeType() == webots::Node::LINEAR_MOTOR) {
                    motorCount++;
                    std::cout << "Motor name: " << deviceName << std::endl;
                }
            }
            std::cout << "Motor count: " << motorCount << std::endl;
            return motorCount;
        }

        double getMotorTorque(webots::Robot *robot, std::string motor_name);

        enum CommandType {
            POSITION, VELOCITY, TORQUE, INVALID
        };

        bool setMotorCommands(webots::Robot *robot, std::vector<double> command_value, CommandType command_type) {
            bool is_command_size_correct = (int) command_value.size() == motorNum_;
            bool is_command_type_valid = command_type == POSITION || command_type == VELOCITY || command_type == TORQUE;
            bool is_valid_command = is_command_size_correct && is_command_type_valid;
            if (!is_valid_command)
                return false;
            for (int i = 0; i < motorNum_; ++i) {
                if (command_type == POSITION)
                    robot->getMotor(robotMotorName_[i])->setPosition(command_value[i]);
                else if (command_type == VELOCITY)
                    robot->getMotor(robotMotorName_[i])->setVelocity(command_value[i]);
                else if (command_type == TORQUE)
                    robot->getMotor(robotMotorName_[i])->setTorque(command_value[i]);
            }
            return true;
        }

        void pubWebotsMotorState(webots::Robot *robot) {
            std_msgs::Float64MultiArray posInfo, velInfo, torInfo;
            for (int i = 0; i < motorNum_; ++i) {
                auto motorAllState = getMotorAllState(robot, robotMotorName_[i]);
                posInfo.data.push_back(motorAllState["position"]);
                velInfo.data.push_back(motorAllState["velocity"]);
                torInfo.data.push_back(motorAllState["torque"]);
            }
            webotsMotorPosPub_.publish(posInfo);
            webotsMotorVelPub_.publish(velInfo);
            webotsMotorTorPub_.publish(torInfo);
        }

        void rosMotorCmdCallback(const std_msgs::Float64MultiArray::ConstPtr &data) {
            for (int i = 0; i < motorNum_; ++i)
                rosMotorCmd_[i] = (double) data->data[i];
        }

        void MPCPolicyCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr &data) {
            if (mpc_policy_.inputTrajectory.empty()) {
                ocs2_msgs::mpc_input zero_input;
                for (int i = 0; i < motorNum_; ++i)
                    zero_input.value.push_back(0.);
                mpc_policy_.inputTrajectory.push_back(zero_input);
            } else
                mpc_policy_.inputTrajectory.begin()->value = data->inputTrajectory.begin()->value;
        }

        void velocityModeUpdate(webots::Robot *robot) {
            pubWebotsMotorState(robot);
            setMotorCommands(robot, rosMotorCmd_, webots_deliver::RosWebotsDeliver::VELOCITY);
        }

    public:
        ros::Subscriber observation_;
        ocs2_msgs::mpc_flattened_controller mpc_policy_;
        std::vector<double> rosMotorCmd_{};
        ros::Subscriber rosMotorCmdSub_;
        ros::Publisher webotsMotorVelPub_, webotsMotorPosPub_, webotsMotorTorPub_;

        int motorNum_;
        std::vector<std::string> robotSensorName_, robotMotorName_;
    };
}