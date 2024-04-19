#ifndef _MIT_MOTOR_HPP_
#define _MIT_MOTOR_HPP_

#include "drivers/can.hpp"

#define HT_MOTOR_LEFT_HIP 0x01
#define HT_MOTOR_LEFT_KNEE 0x02
#define HT_MOTOR_LEFT_ANKLE 0x04
#define HT_MOTOR_RIGHT_HIP 0x01
#define HT_MOTOR_RIGHT_KNEE 0x02
#define HT_MOTOR_RIGHT_ANKLE 0x04

namespace HT {
    constexpr float P_MIN = -95.5f;  // Radians
    constexpr float P_MAX = 95.5f;
    constexpr float V_MIN = -45.0f;  // Rad/s
    constexpr float V_MAX = 45.0f;
    constexpr float KP_MIN = 0.0f;  // N-m/rad
    constexpr float KP_MAX = 500.0f;
    constexpr float KD_MIN = 0.0f;  // N-m/rad/s
    constexpr float KD_MAX = 5.0f;
    constexpr float T_MIN = -18.0f;
    constexpr float T_MAX = 18.0f;
}  // namespace HT

typedef struct Motor_HT {
    float position;
    float velocity;
    float current;

    float position_zero = 0.0f;

    uint8_t id;
    int32_t tx_cnt = 0;
} Motor_HT_t;


void HTMotor_Start(CanChannel_e can, uint16_t id);
void HTMotor_ZeroPosition(CanChannel_e can, uint8_t id);

void HTMotor_MITCommmand(CanChannel_e can_channel, uint8_t set_id, float t_p, float t_v, float f_kp, float f_kd, float f_t);
void HTMotor_SetMode(CanChannel_e can_type, uint8_t id, uint8_t cmd);

void HTMotorLeftCallbackFunc(can_frame_t *frame);
void HTMotorRightCallbackFunc(can_frame_t *frame);

extern Motor_HT_t HT_motor_left[3];
extern Motor_HT_t HT_motor_right[3];

#endif