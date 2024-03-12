//
// Created by lsy on 24-3-1.
//
#pragma once

#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots_deliver/ur5_def.h>
#include <ros/ros.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <unordered_map>
#include <std_msgs/Float64MultiArray.h>

namespace webots_deliver {
    void enablePositionSensor(webots::Robot *robot, int samplingPeriod = 1000) {
        for (int i = 0; i < MOTOR_NUM; ++i) {
            webots::PositionSensor *ps = robot->getPositionSensor(DEF::pos_sensor_name[i]);
            ps->enable(samplingPeriod);
        }
    }

    void setVelocityMode(webots::Robot *robot) {
        for (int i = 0; i < MOTOR_NUM; ++i) {
            webots::Motor* motor = robot->getMotor(DEF::motors_name[i]);
            motor->setPosition(INFINITY);
        }
    }

    std::unordered_map<std::string, double> getMotorAllState(webots::Robot *robot, const std::string &motor_name)
    {
        std::unordered_map<std::string, double> stateMap;

        webots::Motor* motor = robot->getMotor(motor_name);
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
        for (int i = 0; i < MOTOR_NUM; ++i) {
            webots::PositionSensor *ps = robot->getPositionSensor(DEF::pos_sensor_name[i]);
            motor_positions.push_back(ps->getValue());
            if (print)
                std::cout << DEF::pos_sensor_name[i] << "  " << ps->getValue() << std::endl;
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
        bool is_command_size_correct = (int) command_value.size() == MOTOR_NUM;
        bool is_command_type_valid = command_type == POSITION || command_type == VELOCITY || command_type == TORQUE;
        bool is_valid_command = is_command_size_correct && is_command_type_valid;
        if (!is_valid_command)
            return false;
        for (int i = 0; i < MOTOR_NUM; ++i) {
            if (command_type == POSITION)
                robot->getMotor(DEF::motors_name[i])->setPosition(command_value[i]);
            else if (command_type == VELOCITY)
                robot->getMotor(DEF::motors_name[i])->setVelocity(command_value[i]);
            else if (command_type == TORQUE)
                robot->getMotor(DEF::motors_name[i])->setTorque(command_value[i]);
        }
        return true;
    }
    class RosWebotsDeliver {
    public:
        RosWebotsDeliver(ros::NodeHandle nh){
            initDeliver(nh);
        };

        ~RosWebotsDeliver() = default;

        void initDeliver(ros::NodeHandle nh)
        {
            observation_ = nh.subscribe<ocs2_msgs::mpc_flattened_controller>("/mobile_manipulator_mpc_policy",10, &RosWebotsDeliver::MPCPolicyCallback,this);
            rosMotorCmdSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/ros_motor_cmd",10, &RosWebotsDeliver::rosMotorCmdCallback,this);
            webotsMotorPosPub_ = nh.advertise<std_msgs::Float64MultiArray>("/webots_motor_pos",1);
            webotsMotorVelPub_ = nh.advertise<std_msgs::Float64MultiArray>("/webots_motor_vel",1);
            webotsMotorTorPub_ = nh.advertise<std_msgs::Float64MultiArray>("/webots_motor_tor",1);
            mpc_policy_.inputTrajectory.clear();
            for (int i = 0; i < INPUT_DIM; ++i)
                rosMotorCmd_.push_back(0.);
        }
        void pubWebotsMotorState(webots::Robot *robot)
        {
            std_msgs::Float64MultiArray posInfo, velInfo, torInfo;
            for (int i = 0; i < MOTOR_NUM; ++i) {
                auto motorAllState = getMotorAllState(robot, DEF::motors_name[i]);
                posInfo.data.push_back(motorAllState["position"]);
                velInfo.data.push_back(motorAllState["velocity"]);
                torInfo.data.push_back(motorAllState["torque"]);
            }
            webotsMotorPosPub_.publish(posInfo);
            webotsMotorVelPub_.publish(velInfo);
            webotsMotorTorPub_.publish(torInfo);
        }
        void rosMotorCmdCallback(const std_msgs::Float64MultiArray ::ConstPtr& data)
        {
            for (int i = 0; i < INPUT_DIM; ++i)
                rosMotorCmd_[i] = (double)data->data[i];
        }
        void MPCPolicyCallback(const ocs2_msgs::mpc_flattened_controller ::ConstPtr& data)
        {
            if (mpc_policy_.inputTrajectory.empty())
            {
                ocs2_msgs::mpc_input zero_input;
                for (int i = 0; i < INPUT_DIM; ++i)
                    zero_input.value.push_back(0.);
                mpc_policy_.inputTrajectory.push_back(zero_input);
            }
            else
                mpc_policy_.inputTrajectory.begin()->value = data->inputTrajectory.begin()->value;
        }
    public:
        int INPUT_DIM = 6;
        ros::Subscriber observation_;
        ocs2_msgs::mpc_flattened_controller mpc_policy_;
        std::vector<double> rosMotorCmd_{};
        ros::Subscriber rosMotorCmdSub_;
        ros::Publisher webotsMotorVelPub_,webotsMotorPosPub_,webotsMotorTorPub_;

    };
}