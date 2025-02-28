
#include <iostream>

#include <airbot/motor_driver.hpp>
#include <airbot/dm_motor_driver.hpp>
#include <airbot/od_motor_driver.hpp>

namespace arm {
uint8_t MotorDriver::motor_error_type_ = NONE_ERROR;

std::unique_ptr<MotorDriver> MotorDriver::MotorCreate(const CanChannel_e& can_channel, uint16_t motor_id, const std::string gripper_type) {
    if (motor_id >= 1 && motor_id <= 3) {
        return std::make_unique<OdMotorDriver>(can_channel, motor_id);
    } else if (motor_id >= 4 && motor_id <= 6) {
        return std::make_unique<DmMotorDriver>(can_channel, motor_id);
    } else if (motor_id == 7 || motor_id == 8) {
        if (gripper_type == "gripper" || gripper_type == "teacher") {
            return std::make_unique<DmMotorDriver>(can_channel, motor_id);
        } else if (gripper_type == "newteacher") {
            std::cout << "newteacher" << std::endl;
            return std::make_unique<OdMotorDriver>(can_channel, motor_id);
        } else {
            throw std::runtime_error("Gripper type not supported");
        }
    } else {
        throw std::runtime_error("Motor ID out of range");
    }
}

bool MotorDriver::MotorCurrentDetect() {
    static int detect_current_counter[6] = {0};
    if (get_motor_current() < -10.0f) {
        return false;
    }
    if (abs(get_motor_current()) > 25.0f) {
        detect_current_counter[motor_id_ - 1]++;
    } else {
        detect_current_counter[motor_id_ - 1] = 0;
    }
    if (detect_current_counter[motor_id_ - 1] > 60) {
        printf("motor %d is over current\n", motor_id_);
        motor_error_type_ = OVER_CURRENT;
        return true;
    }
    return false;
}

bool MotorDriver::MotorCommunicationDetect() {
    static int detect_communication_counter[6] = {0};
    if (heartbeat_detect_counter_ != 0) {
        detect_communication_counter[motor_id_ - 1]++;
    } else {
        heartbeat_detect_counter_++;
        detect_communication_counter[motor_id_ - 1] = 0;
    }
    if (detect_communication_counter[motor_id_ - 1] > 1e5) {
        printf("motor %d is communication error\n", motor_id_);
        motor_error_type_ = COMMUNICATION_ERROR;
        return true;
    }
    return false;
}

bool MotorDriver::MotorTemperatureDetect() {
    static int detect_temperature_counter[6] = {0};
    if (get_motor_msg()->motor_temperature > 80.0f) {
        detect_temperature_counter[motor_id_ - 1]++;
    } else {
        detect_temperature_counter[motor_id_ - 1] = 0;
    }
    if (detect_temperature_counter[motor_id_ - 1] > 60) {
        printf("motor %d is over temperature\n", motor_id_);
        motor_error_type_ = OVER_TEMPERATURE;
        return true;
    }
    return false;
}

bool MotorDriver::MotorErrorDetect() {
    return MotorCurrentDetect() || MotorCommunicationDetect() || MotorTemperatureDetect();
}

void MotorDriver::MotorErrorModeCmd() {
    switch (motor_error_type_) {
    case OVER_CURRENT:
        MotorPosModeCmd(0.0f, 0.5f);
        break;
    case OVER_TEMPERATURE:
        MotorPosModeCmd(0.0f, 0.5f);
        break;
    case COMMUNICATION_ERROR:
        MotorPosModeCmd(get_motor_pos(), 0.5f);
        break;
    default:
        break;
    }
}

}  // namespace arm
