#ifndef DM_MOTOR_DRIVER_HPP
#define DM_MOTOR_DRIVER_HPP

#include <mutex>

#include <airbot/motor_driver.hpp>

namespace arm {
class DmMotorDriver : public MotorDriver {
public:
    DmMotorDriver(const CanChannel_e& can_channel, uint16_t motor_id);
    ~DmMotorDriver();

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
    virtual void MotorResetID() override{};
    virtual void set_motor_id(uint8_t motor_id) override;
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual const MotorMsg* get_motor_msg() override;
    virtual int get_response_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return response_count;
    }
    void CanRxMsgCallback(const can_frame_t& rx_frame, uint8_t comm_mode) override;

private:
    int response_count = 0;
    mutable std::mutex mutex_;
    bool param_cmd_flag_[30] = {false};
    const int kFirmWareVersion = 3163;
    const float kKpMin = 0.0f;
    const float kKpMax = 500.0f;
    const float kKdMin = 0.0f;
    const float kKdMax = 5.0f;
    const float kIMin = -30.0f;
    const float kIMax = 30.0f;
    const float kPMax = 12.5f;
    const float kPMin = -12.5f;
    const float kSpdMin = -30.0f;
    const float kSpdMax = 30.0f;
    const float kTorqueMin = -10.0f;
    const float kTorqueMax = 10.0f;
    void DmMotorSetZero();
    void DmMotorClearError();
    void DmWriteRegister(uint8_t rid, float value);
    void DmWriteRegister(uint8_t rid, int32_t value);
    void DmSaveRegister(uint8_t rid);
    void DmLed(uint8_t lid, uint8_t freq);
    virtual void CanSendMsg(can_frame_t& tx_frame) {};
    MotorMsg* can_motor_msg_;
};
}  // namespace arm

#endif
