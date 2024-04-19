#ifndef MOTOR_MESSAGE_H
#define MOTOR_MESSAGE_H

#include <stdint.h>
#include <string.h>

namespace arm {
struct MotorMsg {
    uint16_t firmware_version;  // q: dm 10 软件版本
                                // od 0 固件版本
    uint16_t motor_id;          // dm 7 本机ID
                                // od
    float pos;                  // dm
                                // od 1 位置
    float vel;                  // dm
                                // od 2 速度
    float current;              // dm
                                // od 3 电流
    uint8_t error_id;           // dm od
    uint8_t motor_temperature;  // dm od

    uint16_t linkage_kp;  // od 6 todo: change type
    float kp_spd;         // dm 16 速度环比例参数

    uint16_t speed_ki;  // od 7 todo: change type
    float ki_spd;       // dm 17 速度环积分参数

    uint16_t feedback_kp;  // od 8 todo: change type
    float kp_pos;          // dm 18 位置环比例参数

    uint16_t feedback_kd;  // od 9 todo: change type
    float dampen_ratio;    // dm 22 (位置环)阻尼比

    uint16_t torque_factor;    // od 11 todo: change type
    float torque_coefficient;  // dm 1 扭矩系数

    uint16_t position_kd_filter;  // od 10 todo: change type

    float ki_pos;  // dm 19 位置环积分参数

    uint16_t acceleration;  // od 5 加速度 todo: change type
    float acc;              // dm 3 加速度

    bool write_para_res;  // od dm

    float power;                    // od 4 功率
    uint16_t read_para;             // od 配置代码
    uint16_t write_para;            // od 查询代码
    uint16_t master_id;             // q: dm 6 反馈ID
    uint32_t timeout;               // q: dm 8 超时时间
    uint8_t mos_temperature;        // dm
    uint8_t ctrl_mode;              // dm 9 控制模式
    float under_voltage;            // dm 0 欠压保护值
    float over_voltage;             // dm 20 过压保护值
    float current_limit;            // dm 2 限流值
    float dec;                      // dm 4 减速度
    float max_speed;                // dm 5 最大速度
    float gear_ratio;               // dm 11 齿轮减速比
    float pos_max;                  // dm 12 位置映射最大值
    float vel_max;                  // dm 13 速度映射最大值
    float torque_max;               // dm 14 扭矩映射最大值
    float current_bandwidth;        // dm 15 电流环控制带宽
    float gear_torque_coefficient;  // dm 21 齿轮扭矩系数
    float output_angle;
    MotorMsg() {
        memset(this, 0, sizeof(MotorMsg));
        firmware_version = 0;
        motor_id = 0;
        pos = 0.0f;
        vel = 0.0f;
        current = 0.0f;
        error_id = 0;
        motor_temperature = 0;
        linkage_kp = 0;
        kp_spd = 0.0f;
        speed_ki = 0;
        ki_spd = 0.0f;
        feedback_kp = 0;
        kp_pos = 0.0f;
        feedback_kd = 0;
        dampen_ratio = 0.0f;
        torque_factor = 0;
        torque_coefficient = 0.0f;
        position_kd_filter = 0;
        ki_pos = 0.0f;
        acceleration = 0;
        acc = 0.0f;
        write_para_res = false;
        power = 0.0f;
        read_para = 0;
        write_para = 0;
        master_id = 0;
        timeout = 0;
        mos_temperature = 0;
        ctrl_mode = 0;
        under_voltage = 0.0f;
        over_voltage = 0.0f;
        current_limit = 0.0f;
        dec = 0.0f;
        max_speed = 0.0f;
        gear_ratio = 0.0f;
        pos_max = 0.0f;
        vel_max = 0.0f;
        torque_max = 0.0f;
        current_bandwidth = 0.0f;
        gear_torque_coefficient = 0.0f;
        output_angle = 0.0f;
    }
};

}  // namespace arm

#endif  // MOTOR_MESSAGE_H
