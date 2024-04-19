// DmMotorDriver.cpp
#include <airbot/dm_motor_driver.hpp>

#include <chrono>
#include <thread>

#include <algorithm>

namespace arm {

DmMotorDriver::DmMotorDriver(const CanChannel_e& can_channel, uint16_t motor_id)
    : MotorDriver() {
    can_channel_ = can_channel;
    motor_id_ = motor_id;
    can_motor_msg_ = new MotorMsg();
}

DmMotorDriver::~DmMotorDriver() {
    delete can_motor_msg_;
}

void DmMotorDriver::MotorLock() {
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;  // change according to the mode
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFC;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::MotorUnlock() {
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;  // change according to the mode
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFD;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

bool DmMotorDriver::MotorInit() {
    // send disable command to enter read mode
    DmMotorDriver::MotorUnlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    // send get firmware version command
    DmMotorDriver::MotorGetParam(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    // send enable command to enter contorl mode
    DmMotorDriver::MotorLock();
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    DmMotorDriver::DmLed('G', 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    if (get_motor_msg()->firmware_version == 0) {
        printf("motor_id: %d communication error , init failed!\n", motor_id_);
        return false;
    }
    printf("motor_id: %d\tversion: %d\ttarget version: %d\n", motor_id_, get_motor_msg()->firmware_version, kFirmWareVersion);
    if (get_motor_msg()->firmware_version != kFirmWareVersion) {
        printf("firmware version error, init failed!\n");
        return false;
    } else {
        printf("firmware version match, init success!\n");
        return true;
    }
}

void DmMotorDriver::MotorDeInit() {
    DmMotorDriver::MotorUnlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    DmMotorDriver::DmLed('G', 0);
}

bool DmMotorDriver::MotorWriteFlash() { return true; }

bool DmMotorDriver::MotorSetZero() {
    // send set zero command
    DmMotorDriver::DmMotorSetZero();
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for motor to set zero
    DmMotorDriver::MotorUnlock();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        return false;
    } else {
        return false;
    }
    // disable motor
}

void DmMotorDriver::CanRxMsgCallback(const can_frame_t& rx_frame, uint8_t comm_mode) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // response_count--;
        response_count = 0;
    }
    MotorMsg* can_temp_motor;
    can_temp_motor = can_motor_msg_;

    if (rx_frame.can_id == motor_id_ || rx_frame.can_id == 0) {
        uint16_t motor_id_t = 0;
        int pos_int = 0;
        int spd_int = 0;
        int t_int = 0;
        pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
        spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
        t_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
        motor_id_t = (rx_frame.data[0] & 0x0F);
        can_temp_motor->motor_id = motor_id_t;
        can_temp_motor->error_id = rx_frame.data[0] & 0xF0;
        can_temp_motor->pos = uint_to_float(pos_int, kPMin, kPMax, 16);
        can_temp_motor->vel = uint_to_float(spd_int, kSpdMin, kSpdMax, 12);
        can_temp_motor->current = uint_to_float(t_int, kTorqueMin, kTorqueMax, 12);
        can_temp_motor->mos_temperature = rx_frame.data[6];
        can_temp_motor->motor_temperature = rx_frame.data[7];

        heartbeat_detect_counter_ = 0;
    } else if (rx_frame.can_id == 0x700) {
        uint16_t motor_id = 0;
        uint8_t reg_id = 0;
        motor_id = (rx_frame.data[1] << 8 | rx_frame.data[0]);
        can_temp_motor->motor_id = motor_id;
        reg_id = rx_frame.data[3];
        rv_type_convert.buf[0] = rx_frame.data[4];
        rv_type_convert.buf[1] = rx_frame.data[5];
        rv_type_convert.buf[2] = rx_frame.data[6];
        rv_type_convert.buf[3] = rx_frame.data[7];
        heartbeat_detect_counter_ = 0;
        switch (reg_id) {
        case 0:
            can_temp_motor->under_voltage = rv_type_convert.to_float;
            break;
        case 1:
            can_temp_motor->torque_coefficient = rv_type_convert.to_float;
            break;
        case 2:
            can_temp_motor->current_limit = rv_type_convert.to_float;
            break;
        case 3:
            can_temp_motor->acc = rv_type_convert.to_float;
            break;
        case 4:
            can_temp_motor->dec = rv_type_convert.to_float;
            break;
        case 5:
            can_temp_motor->max_speed = rv_type_convert.to_float;
            break;
        case 6:
            can_temp_motor->master_id = rv_type_convert.to_int;
            break;
        case 7:
            can_temp_motor->motor_id = rv_type_convert.to_int;
            break;
        case 8:
            can_temp_motor->timeout = rv_type_convert.to_uint;
            break;
        case 9:
            can_temp_motor->ctrl_mode = rv_type_convert.to_int;
            break;
        case 10:
            // todo: type error
            // can_temp_motor->firmware_version = rv_type_convert.to_float;
            can_temp_motor->firmware_version = (int)(rv_type_convert.buf[0] * 1000 + rv_type_convert.buf[1] * 100 +
                                                     rv_type_convert.buf[2] * 10 + rv_type_convert.buf[3]);
            break;
        case 11:
            can_temp_motor->gear_ratio = rv_type_convert.to_float;
            break;
        case 12:
            can_temp_motor->pos_max = rv_type_convert.to_float;
            break;
        case 13:
            can_temp_motor->vel_max = rv_type_convert.to_float;
            break;
        case 14:
            can_temp_motor->torque_max = rv_type_convert.to_float;
            break;
        case 15:
            can_temp_motor->current_bandwidth = rv_type_convert.to_float;
            break;
        case 16:
            can_temp_motor->kp_spd = rv_type_convert.to_float;
            break;
        case 17:
            can_temp_motor->ki_spd = rv_type_convert.to_float;
            break;
        case 18:
            can_temp_motor->kp_pos = rv_type_convert.to_float;
            break;
        case 19:
            can_temp_motor->ki_pos = rv_type_convert.to_float;
            break;
        case 20:
            can_temp_motor->over_voltage = rv_type_convert.to_float;
            break;
        case 21:
            can_temp_motor->gear_torque_coefficient = rv_type_convert.to_float;
            break;
        case 22:
            can_temp_motor->dampen_ratio = rv_type_convert.to_float;
            break;
        default:
            break;
        }
    } else if (rx_frame.can_id == 0x701) {
        uint16_t motor_id = 0;
        motor_id = (rx_frame.data[1] << 8 | rx_frame.data[0]);
        can_temp_motor->motor_id = motor_id;
        int rid = rx_frame.data[3];
        param_cmd_flag_[rid] = true;
        if (rid == 9) {
            motor_control_mode_ = rx_frame.data[4];
        }
    } else if (rx_frame.can_id == 0x702) {
        uint16_t motor_id = 0;
        can_temp_motor->motor_id = motor_id;
        motor_id = (rx_frame.data[1] << 8 | rx_frame.data[0]);
        can_temp_motor->write_para_res = rx_frame.data[3];
        heartbeat_detect_counter_ = 0;
    }
}

void DmMotorDriver::MotorGetParam(uint8_t param_cmd) {
    can_frame_t tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0x33;
    tx_frame.data[3] = param_cmd;

    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFF;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::MotorPosModeCmd(float pos, float spd) {
    if (motor_control_mode_ != POS) {
        set_motor_control_mode(POS);
        return;
    }
    if (pos < joint_lower_bounder_[motor_id_ - 1] || pos > joint_upper_bounder_[motor_id_ - 1]) {
        return;
    }
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;
    uint8_t *pbuf, *vbuf;

    spd = std::clamp(spd, kSpdMin, kSpdMax);
    pos = std::clamp(pos, kPMin, kPMax);

    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&spd;

    tx_frame.data[0] = *pbuf;
    tx_frame.data[1] = *(pbuf + 1);
    tx_frame.data[2] = *(pbuf + 2);
    tx_frame.data[3] = *(pbuf + 3);
    tx_frame.data[4] = *vbuf;
    tx_frame.data[5] = *(vbuf + 1);
    tx_frame.data[6] = *(vbuf + 2);
    tx_frame.data[7] = *(vbuf + 3);

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::MotorSpdModeCmd(float spd) {
    if (motor_control_mode_ != SPD) {
        set_motor_control_mode(SPD);
        return;
    }
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x04;

    rv_type_convert.to_float = spd;
    tx_frame.data[0] = rv_type_convert.buf[0];
    tx_frame.data[1] = rv_type_convert.buf[1];
    tx_frame.data[2] = rv_type_convert.buf[2];
    tx_frame.data[3] = rv_type_convert.buf[3];

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
void DmMotorDriver::MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    uint16_t p, v, kp, kd, t;
    can_frame_t tx_frame;

    f_p = std::clamp(f_p, kPMin, kPMax);
    f_v = std::clamp(f_v, kSpdMin, kSpdMax);
    f_kp = std::clamp(f_kp, kKpMin, kKpMax);
    f_kd = std::clamp(f_kd, kKdMin, kKdMax);
    f_t = std::clamp(f_t, kTorqueMin, kTorqueMax);

    p = float_to_uint(f_p, kPMin, kPMax, 16);
    v = float_to_uint(f_v, kSpdMin, kSpdMax, 12);
    kp = float_to_uint(f_kp, kKpMin, kKpMax, 12);
    kd = float_to_uint(f_kd, kKdMin, kKdMax, 12);
    t = float_to_uint(f_t, kTorqueMin, kTorqueMax, 12);
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = p >> 8;
    tx_frame.data[1] = p & 0xFF;
    tx_frame.data[2] = v >> 4;
    tx_frame.data[3] = (v & 0x0F) << 4 | kp >> 8;
    tx_frame.data[4] = kp & 0xFF;
    tx_frame.data[5] = kd >> 4;
    tx_frame.data[6] = (kd & 0x0F) << 4 | t >> 8;
    tx_frame.data[7] = t & 0xFF;

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

// todo
void DmMotorDriver::MotorSetPosParam(float kp, float kd) {}

void DmMotorDriver::MotorSetSpdParam(float kp, float ki) {}

void DmMotorDriver::MotorSetFilterParam(float position_kd_filter, float torque_factor) {}

void DmMotorDriver::set_motor_id(uint8_t motor_id) {
    DmWriteRegister(7, motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    motor_id_ = motor_id;
    DmSaveRegister(7);
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    DmWriteRegister(6, motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    DmSaveRegister(6);
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
}

void DmMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) { DmWriteRegister(9, motor_control_mode); }

const MotorMsg* DmMotorDriver::get_motor_msg() {
    return can_motor_msg_;
}

void DmMotorDriver::DmMotorSetZero() {
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;  // change according to the mode
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFE;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmMotorClearError() {
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;  // change according to the mode
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0xFF;
    tx_frame.data[3] = 0xFF;
    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFB;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmWriteRegister(uint8_t rid, float value) {
    param_cmd_flag_[rid] = false;
    can_frame_t tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0x55;
    tx_frame.data[3] = rid;

    tx_frame.data[4] = *vbuf;
    tx_frame.data[5] = *(vbuf + 1);
    tx_frame.data[6] = *(vbuf + 2);
    tx_frame.data[7] = *(vbuf + 3);
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmWriteRegister(uint8_t rid, int32_t value) {
    param_cmd_flag_[rid] = false;
    can_frame_t tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0x55;
    tx_frame.data[3] = rid;

    tx_frame.data[4] = *vbuf;
    tx_frame.data[5] = *(vbuf + 1);
    tx_frame.data[6] = *(vbuf + 2);
    tx_frame.data[7] = *(vbuf + 3);
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmSaveRegister(uint8_t rid) {
    can_frame_t tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0xAA;
    tx_frame.data[3] = rid;

    tx_frame.data[4] = 0xFF;
    tx_frame.data[5] = 0xFF;
    tx_frame.data[6] = 0xFF;
    tx_frame.data[7] = 0xFF;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void DmMotorDriver::DmLed(uint8_t lid, uint8_t freq) {
    can_frame_t tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = motor_id_ & 0xFF;
    tx_frame.data[1] = motor_id_ >> 8;
    tx_frame.data[2] = 0xDD;
    tx_frame.data[3] = 0xFF;

    tx_frame.data[4] = lid;
    tx_frame.data[5] = 0x00;
    tx_frame.data[6] = 0x00;
    tx_frame.data[7] = freq;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

}  // namespace arm
