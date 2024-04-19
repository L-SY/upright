#include <algorithm>

#include <drivers/mit_motor.hpp>
#include <libraries/utils.hpp>

#define CMD_MOTOR_MODE 0xFC
#define CMD_RESET_MODE 0xFD
#define CMD_ZERO_POSITION 0xFE

void HTMotor_ZeroPosition(CanChannel_e can_channel, uint8_t id) {
    // 根据不同的电机模式，发送不同的报文，给第7位，电机，零位校正，reset三种模式
    HTMotor_SetMode(can_channel, id, CMD_MOTOR_MODE);

    // 参数置零
    HTMotor_MITCommmand(can_channel, id, 0, 0, 0, 0, 0);
}

// 启动电机控制
void HTMotor_Start(CanChannel_e can_channel, uint16_t id) {
    // 先进入电机模式，然后把所有信息变0，然后再发零位校正模式，进行校正
    HTMotor_ZeroPosition(can_channel, id);
    HTMotor_SetMode(can_channel, id, CMD_ZERO_POSITION);
}

void HTMotor_MITCommmand(CanChannel_e can_channel, uint8_t set_id, float t_p, float t_v, float f_kp, float f_kd, float f_t) {
    uint16_t p, v, kp, kd, t;

    t_p = std::clamp(t_p, HT::P_MIN, HT::P_MAX);
    t_v = std::clamp(t_v, HT::V_MIN, HT::V_MAX);
    f_kp = std::clamp(f_kp, HT::KP_MIN, HT::KP_MAX);
    f_kd = std::clamp(f_kd, HT::KD_MIN, HT::KD_MAX);
    f_t = std::clamp(f_t, HT::T_MIN, HT::T_MAX);

    p = float_to_uint(t_p, HT::P_MIN, HT::P_MAX, 16);
    v = float_to_uint(t_v, HT::V_MIN, HT::V_MAX, 12);
    kp = float_to_uint(f_kp, HT::KP_MIN, HT::KP_MAX, 12);
    kd = float_to_uint(f_kd, HT::KD_MIN, HT::KD_MAX, 12);
    t = float_to_uint(f_t, HT::T_MIN, HT::T_MAX, 12);

    can_frame_t frame;
    frame.can_id = set_id;
    frame.can_dlc = 8;
    frame.data[0] = p >> 8;
    frame.data[1] = p & 0xFF;
    frame.data[2] = v >> 4;
    frame.data[3] = ((v & 0xF) << 4) | (kp >> 8);
    frame.data[4] = kp & 0xFF;
    frame.data[5] = kd >> 4;
    frame.data[6] = ((kd & 0xF) << 4) | (t >> 8);
    frame.data[7] = t & 0xff;

    Can::can_handle.CanTransmit(can_channel, frame);

}

void HTMotor_SetMode(CanChannel_e can_channel, uint8_t id, uint8_t cmd) {
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};

    can_frame_t frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = buf[0];
    frame.data[1] = buf[1];
    frame.data[2] = buf[2];
    frame.data[3] = buf[3];
    frame.data[4] = buf[4];
    frame.data[5] = buf[5];
    frame.data[6] = buf[6];
    frame.data[7] = buf[7];

    Can::can_handle.CanTransmit(can_channel, frame);
}

void HTMotorLeftCallbackFunc(can_frame_t *frame) {
    if (frame->data[0] == HT_MOTOR_LEFT_HIP || frame->data[0] == HT_MOTOR_LEFT_KNEE || frame->data[0] == HT_MOTOR_LEFT_ANKLE) {
        size_t arg_ht_motor;
        switch (frame->data[0]) {
        case HT_MOTOR_LEFT_HIP:
            arg_ht_motor = 0;
            break;
        case HT_MOTOR_LEFT_KNEE:
            arg_ht_motor = 1;
            break; 
        case HT_MOTOR_LEFT_ANKLE:
            arg_ht_motor = 2;
            break;
        }
        int p_int = (frame->data[1] << 8) | frame->data[2];
        int v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
        int i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];
        float p = uint_to_float(p_int, HT::P_MIN, HT::P_MAX, 16);
        float v = uint_to_float(v_int, HT::V_MIN, HT::V_MAX, 12);
        float i = uint_to_float(i_int, -40, 40, 12);
        HT_motor_left[arg_ht_motor].position = -p - HT_motor_left[arg_ht_motor].position_zero;
        HT_motor_left[arg_ht_motor].velocity = -v;
        HT_motor_left[arg_ht_motor].current = -i;
        HT_motor_left[arg_ht_motor].tx_cnt = 0;
    }
}

void HTMotorRightCallbackFunc(can_frame_t *frame) {
    if (frame->data[0] == HT_MOTOR_RIGHT_HIP || frame->data[0] == HT_MOTOR_RIGHT_KNEE || frame->data[0] == HT_MOTOR_RIGHT_ANKLE) {
        size_t arg_ht_motor;
        switch (frame->data[0]) {
        case HT_MOTOR_RIGHT_HIP:
            arg_ht_motor = 0;
            break;
        case HT_MOTOR_RIGHT_KNEE:
            arg_ht_motor = 1;
            break; 
        case HT_MOTOR_RIGHT_ANKLE:
            arg_ht_motor = 2;
            break;
        }
        int p_int = (frame->data[1] << 8) | frame->data[2];
        int v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
        int i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];
        float p = uint_to_float(p_int, HT::P_MIN, HT::P_MAX, 16);
        float v = uint_to_float(v_int, HT::V_MIN, HT::V_MAX, 12);
        float i = uint_to_float(i_int, -40, 40, 12);
        HT_motor_right[arg_ht_motor].position = p - HT_motor_right[arg_ht_motor].position_zero;
        HT_motor_right[arg_ht_motor].velocity = v;
        HT_motor_right[arg_ht_motor].current = i;
        HT_motor_right[arg_ht_motor].tx_cnt = 0;
    }
}

Motor_HT_t HT_motor_left[3] = {0};
Motor_HT_t HT_motor_right[3] = {0};
