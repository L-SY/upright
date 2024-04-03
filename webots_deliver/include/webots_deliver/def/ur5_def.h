//
// Created by lsy on 24-3-1.
//

#pragma once

#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/PositionSensor.hpp>

#define ROBOT_NAME "UR5e"
#define MOTOR_NUM 6

namespace DEF {
    const double init_position[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const char motors_name[MOTOR_NUM][40] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    const char pos_sensor_name[MOTOR_NUM][40] = {"shoulder_pan_joint_sensor", "shoulder_lift_joint_sensor",
                                                 "elbow_joint_sensor", "wrist_1_joint_sensor", "wrist_2_joint_sensor",
                                                 "wrist_3_joint_sensor"};
}
