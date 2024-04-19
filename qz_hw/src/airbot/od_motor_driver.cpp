// OdMotorDriver.cpp
#include <functional>
#include <iostream>
#include <chrono>
#include <thread>

#include <airbot/od_motor_driver.hpp>

namespace arm {

OdMotorDriver::OdMotorDriver(const CanChannel_e& can_channel, uint16_t motor_id)
    : MotorDriver() {
    can_channel_ = can_channel;
    motor_id_ = motor_id;
    board_id_ = motor_id;
    can_motor_msg_ = new MotorMsg();
}

OdMotorDriver::~OdMotorDriver() {
    delete can_motor_msg_;
}

void OdMotorDriver::MotorLock() {
    OdMotorDriver::ConfigOdMotor(0x08);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void OdMotorDriver::MotorUnlock() {
    OdMotorDriver::ConfigOdMotor(0x07);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool OdMotorDriver::MotorInit() { return Init() && OdCheckId(); }

void OdMotorDriver::MotorDeInit() { OdMotorDriver::MotorLock(); }

bool OdMotorDriver::MotorSetZero() {
    // send set zero command
    OdMotorDriver::ConfigOdMotor(0x03);
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for motor to set zero
    // send read position command
    OdMotorDriver::MotorGetParam(0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        return false;
    } else {
        return true;
    }
}

bool OdMotorDriver::MotorWriteFlash() {
    OdMotorDriver::ConfigOdMotor(0x06);
    return true;
}

void OdMotorDriver::CanRxMsgCallback(const can_frame_t& rx_frame, uint8_t comm_mode) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // response_count--;
        response_count = 0;
    }

    MotorMsg* can_temp_motor;
    can_temp_motor = can_motor_msg_;

    uint8_t motor_id_t = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    int outpos_int = 0;
    if (rx_frame.can_id == RET_CMD_ID) {
        can_temp_motor->motor_id = board_id_;
        ret_id_ = board_id_;
        switch (rx_frame.data[0]) {
        case CMD_DEVICE_ID:
            ret_id_ = rx_frame.data[2] | (rx_frame.data[3] << 8) | (rx_frame.data[4] << 16) | (rx_frame.data[5] << 24);
            break;
        case CMD_HARDWARE_VERSION:
            hardware_version_ = std::string("V" + std::to_string(rx_frame.data[3]) + "." +
                                            std::to_string(rx_frame.data[4]) + "." + std::to_string(rx_frame.data[5]));
            break;

        case CMD_FIRMWARE_VERSION:
            firmware_version_ = std::string("V" + std::to_string(rx_frame.data[3]) + "." +
                                            std::to_string(rx_frame.data[4]) + "." + std::to_string(rx_frame.data[5]));
            break;

        case CMD_SN_CODE:
            static char sn_code[16] = {0};
            static uint8_t sn_code_flag = 0;
            switch (rx_frame.data[1]) {
            case FRAME_1:
                for (int i = 0; i < 4; i++) sn_code[i] = rx_frame.data[i + 2];
                sn_code_flag |= 0x01;
                break;
            case FRAME_2:
                for (int i = 0; i < 4; i++) sn_code[i + 4] = rx_frame.data[i + 2];
                sn_code_flag |= 0x02;
                break;
            case FRAME_3:
                for (int i = 0; i < 4; i++) sn_code[i + 8] = rx_frame.data[i + 2];
                sn_code_flag |= 0x04;
                break;
            case FRAME_4:
                for (int i = 0; i < 4; i++) sn_code[i + 12] = rx_frame.data[i + 2];
                sn_code_flag |= 0x08;
                break;
            default:
                break;
            }
            if (sn_code_flag == 0x0F) {
                sn_code_ = std::string(sn_code, 16);
                sn_code_flag = 0;
            }
            break;
        case CMD_ARM_SN_CODE:
            static char arm_sn_code[16] = {0};
            static uint8_t arm_sn_code_flag = 0;
            switch (rx_frame.data[1]) {
            case FRAME_1:
                for (int i = 0; i < 4; i++) arm_sn_code[i] = rx_frame.data[i + 2];
                arm_sn_code_flag |= 0x01;
                break;
            case FRAME_2:
                for (int i = 0; i < 4; i++) arm_sn_code[i + 4] = rx_frame.data[i + 2];
                arm_sn_code_flag |= 0x02;
                break;
            case FRAME_3:
                for (int i = 0; i < 4; i++) arm_sn_code[i + 8] = rx_frame.data[i + 2];
                arm_sn_code_flag |= 0x04;
                break;
            case FRAME_4:
                for (int i = 0; i < 4; i++) arm_sn_code[i + 12] = rx_frame.data[i + 2];
                arm_sn_code_flag |= 0x08;
                break;
            default:
                break;
            }
            if (arm_sn_code_flag == 0x0F) {
                arm_sn_code_ = std::string(arm_sn_code, 16);
                arm_sn_code_flag = 0;
            }
            break;
        default:
            break;
        }
    } else if (rx_frame.can_id == 0x7FF) {
        if (rx_frame.data[2] != 0x01)
            return;
        if ((rx_frame.data[0] == 0xFF) && (rx_frame.data[1] == 0xFF)) {
            motor_id_t = (rx_frame.data[3] << 8 | rx_frame.data[4]);
            communication_mode_ = 0x01;
            heartbeat_detect_counter_ = 0;
            can_temp_motor->motor_id = motor_id_t;
        } else {
            motor_id_t = (rx_frame.data[0] << 8 | rx_frame.data[1]);
            communication_mode_ = rx_frame.data[3];
            heartbeat_detect_counter_ = 0;
            can_temp_motor->motor_id = motor_id_t;
        }
    } else if (comm_mode == 0x00) {
        ack_status = rx_frame.data[0] >> 5;
        motor_id_t = rx_frame.can_id;
        if (can_temp_motor == nullptr)
            return;
        can_temp_motor->motor_id = motor_id_t;
        can_temp_motor->error_id = rx_frame.data[0] & 0x1F;
        heartbeat_detect_counter_ = 0;

        if (ack_status == 1)  // response frame 1
        {
            pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
            spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
            cur_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
            can_temp_motor->pos = uint_to_float(pos_int, kPosMin, kPosMax, 16);
            can_temp_motor->vel = uint_to_float(spd_int, kSpdMin, kSpdMax, 12);
            can_temp_motor->current = uint_to_float(cur_int, kIMin, kIMax, 12);
            can_temp_motor->motor_temperature = (rx_frame.data[6] - 50) / 2;
        } else if (ack_status == 2)  // response frame 2
        {
            rv_type_convert.buf[0] = rx_frame.data[4];
            rv_type_convert.buf[1] = rx_frame.data[3];
            rv_type_convert.buf[2] = rx_frame.data[2];
            rv_type_convert.buf[3] = rx_frame.data[1];

            can_temp_motor->pos = rv_type_convert.to_float * M_PI / 180.0f;
            can_temp_motor->motor_temperature = (rx_frame.data[7] - 50) / 2;
            can_temp_motor->current = (int16_t)(rx_frame.data[5] << 8 | rx_frame.data[6]) / 100.0f;
        } else if (ack_status == 3)  // response frame 3
        {
            rv_type_convert.buf[0] = rx_frame.data[4];
            rv_type_convert.buf[1] = rx_frame.data[3];
            rv_type_convert.buf[2] = rx_frame.data[2];
            rv_type_convert.buf[3] = rx_frame.data[1];

            can_temp_motor->vel = rv_type_convert.to_float * M_PI / 30.0f;
            can_temp_motor->motor_temperature = (rx_frame.data[7] - 50) / 2;
            can_temp_motor->current = (int16_t)(rx_frame.data[5] << 8 | rx_frame.data[6]) / 100.0f;
        } else if (ack_status == 4)  // response frame 4
        {
            if (rx_frame.can_dlc != 3)
                return;
            can_temp_motor->write_para = rx_frame.data[1];
            can_temp_motor->write_para_res = rx_frame.data[2];
        } else if (ack_status == 5)  // response frame 5
        {
            can_temp_motor->read_para = rx_frame.data[1];
            if (can_temp_motor->read_para == 0 & rx_frame.can_dlc == 4) {
                can_temp_motor->firmware_version = rx_frame.data[2] << 8 | rx_frame.data[3];
            }
            if (can_temp_motor->read_para == 1 & rx_frame.can_dlc == 6)  // get position
            {
                rv_type_convert.buf[0] = rx_frame.data[5];
                rv_type_convert.buf[1] = rx_frame.data[4];
                rv_type_convert.buf[2] = rx_frame.data[3];
                rv_type_convert.buf[3] = rx_frame.data[2];
                can_temp_motor->pos = rv_type_convert.to_float * M_PI / 180.0f;
            } else if (can_temp_motor->read_para == 2 & rx_frame.can_dlc == 6)  // get speed
            {
                rv_type_convert.buf[0] = rx_frame.data[5];
                rv_type_convert.buf[1] = rx_frame.data[4];
                rv_type_convert.buf[2] = rx_frame.data[3];
                rv_type_convert.buf[3] = rx_frame.data[2];
                can_temp_motor->vel = rv_type_convert.to_float * M_PI / 30.0f;
            } else if (can_temp_motor->read_para == 3 & rx_frame.can_dlc == 6)  // get current
            {
                rv_type_convert.buf[0] = rx_frame.data[5];
                rv_type_convert.buf[1] = rx_frame.data[4];
                rv_type_convert.buf[2] = rx_frame.data[3];
                rv_type_convert.buf[3] = rx_frame.data[2];
                can_temp_motor->current = rv_type_convert.to_float;
            } else if (can_temp_motor->read_para == 4 & rx_frame.can_dlc == 6)  // get power
            {
                rv_type_convert.buf[0] = rx_frame.data[5];
                rv_type_convert.buf[1] = rx_frame.data[4];
                rv_type_convert.buf[2] = rx_frame.data[3];
                rv_type_convert.buf[3] = rx_frame.data[2];
                can_temp_motor->power = rv_type_convert.to_float;
            } else if (can_temp_motor->read_para == 5 & rx_frame.can_dlc == 4)  // get acceleration
            {
                can_temp_motor->acceleration = rx_frame.data[2] << 8 | rx_frame.data[3];
            } else if (can_temp_motor->read_para == 6 & rx_frame.can_dlc == 4)  // get linkage_kp
            {
                can_temp_motor->speed_ki = rx_frame.data[2] << 8 | rx_frame.data[3];
            } else if (can_temp_motor->read_para == 7 & rx_frame.can_dlc == 4)  // get speed_ki
            {
                can_temp_motor->linkage_kp = rx_frame.data[2] << 8 | rx_frame.data[3];
            } else if (can_temp_motor->read_para == 8 & rx_frame.can_dlc == 4)  // get feedback_kp
            {
                can_temp_motor->feedback_kp = rx_frame.data[2] << 8 | rx_frame.data[3];
            } else if (can_temp_motor->read_para == 9 & rx_frame.can_dlc == 4)  // get feedback_kd
            {
                can_temp_motor->feedback_kd = rx_frame.data[2] << 8 | rx_frame.data[3];
            } else if (can_temp_motor->read_para == 10 & rx_frame.can_dlc == 4)  // get position_kd_filter
            {
                can_temp_motor->position_kd_filter = rx_frame.data[2] << 8 | rx_frame.data[3];
            } else if (can_temp_motor->read_para == 11 & rx_frame.can_dlc == 4)  // get torque_factor
            {
                can_temp_motor->torque_factor = rx_frame.data[2] << 8 | rx_frame.data[3];
            }
        }
    } else if (comm_mode == 0x01)  // automatic feedback mode
    {
        can_temp_motor->pos = (uint16_t)(rx_frame.data[0] << 8 | rx_frame.data[1]) * M_PI / 18000.0f;
        can_temp_motor->vel = (int16_t)(rx_frame.data[2] << 8 | rx_frame.data[3]) * M_PI / 300.0f;
        can_temp_motor->current = (int16_t)(rx_frame.data[4] << 8 | rx_frame.data[5]) / 100.0f;
        can_temp_motor->motor_temperature = rx_frame.data[6];
        can_temp_motor->error_id = rx_frame.data[7];
    }
}

void OdMotorDriver::MotorGetParam(uint8_t param_cmd) {
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x02;
    tx_frame.data[0] = 0xE0;
    tx_frame.data[1] = param_cmd;

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

// Transmit pos control package. Called in canTask.
void OdMotorDriver::MotorPosModeCmd(float pos, float spd) {
    if (pos < joint_lower_bounder_[motor_id_ - 1] || pos > joint_upper_bounder_[motor_id_ - 1]) {
        return;
    }
    float cur = OdMotorDriver::cur_;
    uint8_t ack_status = OdMotorDriver::ack_status_;
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    if (ack_status > 3)
        return;

    spd = std::clamp(spd, -343.1f, 343.1f);
    cur = std::min(fabs(cur), 409.5f);

    static uint16_t tmp_spd = 0, tmp_cur = 0;
    rv_type_convert.to_float = pos * 180.0f / M_PI;
    tmp_spd = (uint16_t)floorf(spd * 60.0f / M_PI / 2.0f * 10.0f);
    tmp_cur = (uint16_t)floorf(cur * 10.0f);

    tx_frame.data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    tx_frame.data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    tx_frame.data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    tx_frame.data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    tx_frame.data[4] = (rv_type_convert.buf[0] << 5) | (tmp_spd >> 10);
    tx_frame.data[5] = (tmp_spd & 0x3FC) >> 2;
    tx_frame.data[6] = (tmp_spd & 0x03) << 6 | (tmp_cur >> 6);
    tx_frame.data[7] = (tmp_cur & 0x3F) << 2 | ack_status;

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void OdMotorDriver::MotorSpdModeCmd(float spd) {
    float cur = OdMotorDriver::cur_;
    uint8_t ack_status = OdMotorDriver::ack_status_;
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x07;

    if (ack_status > 3)
        return;

    spd = std::clamp(spd, -343.1f, 343.1f);
    cur = std::min(fabs(cur), 409.5f);

    static uint16_t tmp_cur = 0;
    rv_type_convert.to_float = spd * 60.0f / M_PI / 2.0f;
    tmp_cur = (uint16_t)floorf(cur * 10.0f);

    tx_frame.data[0] = 0x40 | ack_status;
    tx_frame.data[1] = rv_type_convert.buf[3];
    tx_frame.data[2] = rv_type_convert.buf[2];
    tx_frame.data[3] = rv_type_convert.buf[1];
    tx_frame.data[4] = rv_type_convert.buf[0];
    tx_frame.data[5] = tmp_cur >> 8;
    tx_frame.data[6] = tmp_cur & 0xff;

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}
// Transmit MIT-mode control(hybrid) package. Called in canTask.
void OdMotorDriver::MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    uint16_t p, v, kp, kd, t;
    can_frame_t tx_frame;

    f_p = std::clamp(f_p, kPosMin, kPosMax);
    f_v = std::clamp(f_v, kSpdMin, kSpdMax);
    f_kp = std::clamp(f_kp, kKpMin, kKpMax);
    f_kd = std::clamp(f_kd, kKdMin, kKdMax);
    f_t = std::clamp(f_t, kTorqueMin, kTorqueMax);

    p = float_to_uint(f_p, kPosMin, kPosMax, 16);
    v = float_to_uint(f_v, kSpdMin, kSpdMax, 12);
    kp = float_to_uint(f_kp, kKpMin, kKpMax, 12);
    kd = float_to_uint(f_kd, kKdMin, kKdMax, 9);
    t = float_to_uint(f_t, kTorqueMin, kTorqueMax, 12);

    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    tx_frame.data[0] = kp >> 7;
    tx_frame.data[1] = ((kp & 0x7F) << 1) | ((kd & 0x100) >> 8);
    tx_frame.data[2] = kd & 0xFF;
    tx_frame.data[3] = p >> 8;
    tx_frame.data[4] = p & 0xFF;
    tx_frame.data[5] = v >> 4;
    tx_frame.data[6] = (v & 0x0F) << 4 | (t >> 8);
    tx_frame.data[7] = t & 0xff;

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void OdMotorDriver::MotorSetPosParam(float kp, float kd) {
    uint16_t fdbKP, fdbKD;
    fdbKP = float_to_uint(kp, kKpMin, kKpMax, 12);
    fdbKD = float_to_uint(kd, kKdMin, kKdMax, 9);
    OdMotorDriver::OdSetPosParam(fdbKP, fdbKD, ack_status_);
    std::this_thread::sleep_for(std::chrono::milliseconds(setup_sleep_time));
}

void OdMotorDriver::MotorSetSpdParam(float kp, float ki) {
    uint16_t linkage, speedKI;
    linkage = float_to_uint(kp, kKpMin, kKpMax, 12);
    speedKI = float_to_uint(ki, kKdMin, kKdMax, 9);
    OdMotorDriver::OdSetSpdParam(linkage, speedKI, ack_status_);
    std::this_thread::sleep_for(std::chrono::milliseconds(setup_sleep_time));
}

void OdMotorDriver::MotorSetFilterParam(float position_kd_filter, float torque_factor) {
    uint16_t position_kd_filter_int, torque_factor_int;
    position_kd_filter_int = float_to_uint(position_kd_filter, kKpMin, kKpMax, 12);
    torque_factor_int = float_to_uint(torque_factor, kKdMin, kKdMax, 9);
    OdMotorDriver::OdSetFilterParam(position_kd_filter_int, torque_factor_int, ack_status_);
    std::this_thread::sleep_for(std::chrono::milliseconds(setup_sleep_time));
}

void OdMotorDriver::MotorResetID() {
    can_frame_t tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x06;
    tx_frame.data[0] = 0x7f;
    tx_frame.data[1] = 0x7f;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = 0x05;
    tx_frame.data[4] = 0x7f;
    tx_frame.data[5] = 0x7f;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void OdMotorDriver::set_motor_id(uint8_t motor_id) {
    can_frame_t tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x06;
    tx_frame.data[0] = motor_id_ >> 8;
    tx_frame.data[1] = motor_id_ & 0xff;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = 0x04;
    tx_frame.data[4] = motor_id >> 8;
    tx_frame.data[5] = motor_id & 0xff;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
    motor_id_ = motor_id;
}

void OdMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) { motor_control_mode_ = motor_control_mode; }

const MotorMsg* OdMotorDriver::get_motor_msg() {
    return can_motor_msg_;
}

void OdMotorDriver::ConfigOdMotor(uint8_t cmd) {
    can_frame_t config_frame;
    config_frame.can_dlc = 0x04;
    config_frame.can_id = 0x7FF;
    if (cmd == 0x00) {
        return;
    }

    /*
    cmd = 0x01: automatic telegram
    cmd = 0x02: answer mode
    cmd = 0x03: set zero
    */
    config_frame.data[0] = motor_id_ >> 8;
    config_frame.data[1] = motor_id_ & 0xff;
    config_frame.data[2] = 0x00;
    config_frame.data[3] = cmd;

    // transmit can bag
    Can::can_handle.CanTransmit(can_channel_, config_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

// Configure the kp,kd of position loop
void OdMotorDriver::OdSetPosParam(uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status) {
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 6;

    if (ack_status > 2)
        return;

    fdbKP = std::min(fdbKP, (uint16_t)10000);
    fdbKD = std::min(fdbKD, (uint16_t)10000);

    tx_frame.data[0] = 0xC0 | ack_status;
    tx_frame.data[1] = 0x03;
    tx_frame.data[2] = fdbKP >> 8;
    tx_frame.data[3] = fdbKP & 0xff;
    tx_frame.data[4] = fdbKD >> 8;
    tx_frame.data[5] = fdbKD & 0xff;

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

// Configure the kp,kd of speed loop
void OdMotorDriver::OdSetSpdParam(uint16_t linkage, uint16_t speedKI, uint8_t ack_status) {
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 6;

    if (ack_status > 2)
        return;

    linkage = std::min(linkage, (uint16_t)10000);
    speedKI = std::min(speedKI, (uint16_t)10000);

    tx_frame.data[0] = 0xC0 | ack_status;
    tx_frame.data[1] = 0x02;
    tx_frame.data[2] = speedKI >> 8;
    tx_frame.data[3] = speedKI & 0xff;
    tx_frame.data[4] = linkage >> 8;
    tx_frame.data[5] = linkage & 0xff;

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void OdMotorDriver::OdSetFilterParam(uint16_t position_kd_filter, uint16_t torque_factor, uint8_t ack_status) {
    can_frame_t tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 6;

    if (ack_status > 2)
        return;

    position_kd_filter = std::min(position_kd_filter, (uint16_t)10000);
    torque_factor = std::min(torque_factor, (uint16_t)10000);

    tx_frame.data[0] = 0xC0 | ack_status;
    tx_frame.data[1] = 0x04;
    tx_frame.data[2] = position_kd_filter >> 8;
    tx_frame.data[3] = position_kd_filter & 0xff;
    tx_frame.data[4] = torque_factor >> 8;
    tx_frame.data[5] = torque_factor & 0xff;

    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
}

void OdMotorDriver::OdSetCommunicationMode(uint8_t communication_mode) {
    OdMotorDriver::ConfigOdMotor(communication_mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(setup_sleep_time));
}

bool OdMotorDriver::OdCheckId() {
    can_frame_t tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x04;
    tx_frame.data[0] = 0xFF;
    tx_frame.data[1] = 0xFF;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = 0x82;
    Can::can_handle.CanTransmit(can_channel_, tx_frame);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response_count++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(normal_sleep_time));
    if (get_motor_msg()->motor_id == motor_id_) {
        return true;
    } else {
        return false;
    }
}

void OdMotorDriver::CanSendMsg(can_frame_t& tx_frame) { Can::can_handle.CanTransmit(can_channel_, tx_frame); }

}  // namespace arm
