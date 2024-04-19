#pragma once

#include <stdint.h>

#include <cmath>
#include <ctime>
#include <drivers/can.hpp>

typedef struct IMU_Float {
    float imu[3];       // 单位 rad    顺序yaw pitch roll
    float last_imu[3];  // 上一时刻的imu角度数据
    float gyro[3];      // 单位 rad/s  顺序pitch  roll  yaw
    float accel[3];
    bool connected[3] = {false, false, false};
    clock_t last_rev[3];
} IMU_Float_t;

void GetImuAngle(uint8_t *data);
void GetImuGyro(uint8_t *data);
void GetImuAcc(uint8_t *data);
void ImuCallbackFunc(can_frame_t *frame);

extern IMU_Float_t imu_c_board;
