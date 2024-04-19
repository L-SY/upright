#ifndef OD_MOTOR_DRIVER_HPP
#define OD_MOTOR_DRIVER_HPP

#include <mutex>

#include <airbot/motor_driver.hpp>

namespace arm {
class OdMotorDriver : public MotorDriver {
public:
    OdMotorDriver(const CanChannel_e& can_channel, uint16_t motor_id);
    ~OdMotorDriver();
    virtual void MotorLock() override;
    virtual void MotorUnlock() override;
    virtual bool MotorInit() override;
    virtual void MotorDeInit() override;
    virtual bool MotorSetZero() override;
    virtual bool MotorWriteFlash() override;

    virtual void MotorGetParam(uint8_t param_cmd) override;
    virtual void MotorPosModeCmd(float pos, float spd) override;
    virtual void MotorSpdModeCmd(float spd) override;
    virtual void MotorMitModeCmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void MotorSetPosParam(float kp, float kd) override;
    virtual void MotorSetSpdParam(float kp, float ki) override;
    virtual void MotorSetFilterParam(float position_kd_filter, float torque_factor) override;
    virtual void MotorResetID();
    virtual void set_motor_id(uint8_t motor_id) override;
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual const MotorMsg* get_motor_msg() override;
    virtual int get_response_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return response_count;
    }
    void CanRxMsgCallback(const can_frame_t& rx_frame, uint8_t comm_mode);

private:
    int response_count = 0;
    mutable std::mutex mutex_;
    const int kFirmWareVersion = 12;
    const float kKpMin = 0.0f;
    const float kKpMax = 500.0f;
    const float kKdMin = 0.0f;
    const float kKdMax = 5.0f;
    const float kIMin = -30.0f;
    const float kIMax = 30.0f;
    const float kPosMin = -12.5f;
    const float kPosMax = 12.5f;
    const float kSpdMin = -18.0f;
    const float kSpdMax = 18.0f;
    const float kTorqueMin = -30.0f;
    const float kTorqueMax = 30.0f;
    const float kOutPutMin = 0.0f;
    const float kOutPutMax = 36000.0f;
    const int linkage_kp = 60;
    const int speed_ki = 1000;
    const int feedback_kp = 60;
    const int feedback_kd = 4;
    const int position_kd_filter = 2000;
    const int torque_factor_b = 1400;
    const int torque_factor_s = 390;
    void ConfigOdMotor(uint8_t cmd);
    void OdSetPosParam(uint16_t fdbKP, uint16_t fdbKD, uint8_t ack_status);
    void OdSetSpdParam(uint16_t linkage, uint16_t speedKI, uint8_t ack_status);
    void OdSetFilterParam(uint16_t position_kd_filter, uint16_t torque_factor, uint8_t ack_status);
    void OdSetCommunicationMode(uint8_t communication_mode);  // 01: automatic telegram 02: answer mode
    bool OdCheckId();
    virtual void CanSendMsg(can_frame_t& tx_frame) override;
    float cur_ = 100.0f;
    uint8_t ack_status_ = 1;
    uint8_t communication_mode_ = 0;
    MotorMsg* can_motor_msg_;
};
}  // namespace arm
#endif
