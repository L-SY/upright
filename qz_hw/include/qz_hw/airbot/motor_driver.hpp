#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <stdint.h>
#include <string.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <drivers/can.hpp>
#include <libraries/utils.hpp>
#include <airbot/board_driver.hpp>
#include <airbot/motor_message.hpp>
using std::vector;

namespace arm {
class MotorDriver : public BoardDriver {
public:
    enum MotorControlMode_e {
        NONE = 0,
        MIT = 1,
        POS = 2,
        SPD = 3,
    };
    enum MotorErrorType_e {
        NONE_ERROR = 0,
        OVER_CURRENT = 1,
        OVER_TEMPERATURE = 2,
        COMMUNICATION_ERROR = 3,
    };

    static constexpr double judgment_accuracy_threshold = 1e-2;
    const int normal_sleep_time = 5;
    const int setup_sleep_time = 500;
    // FIXME Change hard code boundaries to reading from URDF file
    static constexpr float joint_lower_bounder_[7] = {-2.97, -2.61, -0.153, -2.48, -1.5, -3.31, -10};
    static constexpr float joint_upper_bounder_[7] = {1.91, 0.185, 2.77, 2.37, 1.66, 3.31, 10};

    MotorDriver() {}
    virtual ~MotorDriver(){};

    static std::unique_ptr<MotorDriver> MotorCreate(const CanChannel_e& can_channel, uint16_t motor_id, const std::string gripper_type = std::string("newteacher"));

    virtual void MotorLock() = 0;
    virtual void MotorUnlock() = 0;
    virtual bool MotorInit() = 0;
    virtual void MotorDeInit() = 0;
    virtual bool MotorSetZero() = 0;
    virtual bool MotorWriteFlash() = 0;
    virtual vector<double> MotorBoundary() {
        return vector<double>{joint_lower_bounder_[motor_id_ - 1], joint_upper_bounder_[motor_id_ - 1]};
    };
    virtual void MotorGetParam(uint8_t param_cmd) = 0;
    // to enum and union

    virtual void MotorPosModeCmd(float pos, float spd) = 0;
    virtual void MotorSpdModeCmd(float spd) = 0;
    virtual void MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) = 0;
    virtual void MotorSetPosParam(float kp, float kd) = 0;
    virtual void MotorSetSpdParam(float kp, float ki) = 0;
    virtual void MotorSetFilterParam(float position_kd_filter, float torque_factor) = 0;
    virtual void set_motor_id(uint8_t motor_id) = 0;
    virtual void set_motor_control_mode(uint8_t motor_control_mode) = 0;
    virtual int get_response_count() const = 0;

    // to get torque
    // to get error code

    virtual const MotorMsg* get_motor_msg() = 0;
    virtual void MotorResetID() = 0;

    bool MotorCurrentDetect();
    bool MotorCommunicationDetect();
    bool MotorTemperatureDetect();
    bool MotorErrorDetect();
    void MotorErrorModeCmd();

    virtual uint8_t get_motor_id() { return motor_id_; }
    virtual uint8_t get_motor_control_mode() { return motor_control_mode_; }
    virtual float get_motor_pos() { return get_motor_msg()->pos; }
    virtual float get_motor_spd() { return get_motor_msg()->vel; }
    virtual float get_motor_current() { return get_motor_msg()->current; }
    virtual float get_motor_error_id() { return get_motor_msg()->error_id; }
    virtual float get_motor_temperature() { return get_motor_msg()->motor_temperature; }

    virtual void CanRxMsgCallback(const can_frame_t& rx_frame, uint8_t comm_mode) {};

    static uint8_t motor_error_type_;

protected:
    uint16_t motor_id_;
    uint8_t motor_control_mode_;  // 0:none 1:pos 2:spd 3:mit
    uint16_t heartbeat_detect_counter_;
};

// todo: move
union RV_TypeConvert {
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} inline rv_type_convert;

}  // namespace arm

#endif