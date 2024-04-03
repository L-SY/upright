//
// Created by lsy on 24-4-3.
//

#pragma once

#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/PositionSensor.hpp>

#define ROBOT_NAME "ridgeback_ur5"
#define MOTOR_NUM 10
#define CONFIG_NUM 9

namespace OCS2_DEF {
    const double init_position[CONFIG_NUM] = {0.0, 0.0, 0.0, 0.0, -0.785, 1.57, -0.785, 1.57, 0.0};

    const char motors_name[MOTOR_NUM][40] = {
            "front_left_wheel",
            "front_right_wheel",
            "rear_left_wheel",
            "rear_right_wheel",
            "ur_arm_shoulder_pan_joint",
            "ur_arm_shoulder_lift_joint",
            "ur_arm_elbow_joint",
            "ur_arm_wrist_1_joint",
            "ur_arm_wrist_2_joint",
            "ur_arm_wrist_3_joint"
    };

    const char pos_sensor_name[MOTOR_NUM][40] = {
            "front_left_wheel_sensor",
            "front_right_wheel_sensor",
            "rear_left_wheel_sensor",
            "rear_right_wheel_sensor",
            "ur_arm_shoulder_pan_joint_sensor",
            "ur_arm_shoulder_lift_joint_sensor",
            "ur_arm_elbow_joint_sensor",
            "ur_arm_wrist_1_joint_sensor",
            "ur_arm_wrist_2_joint_sensor",
            "ur_arm_wrist_3_joint_sensor"
    };

}
