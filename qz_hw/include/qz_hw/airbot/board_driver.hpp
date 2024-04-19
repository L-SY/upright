#ifndef BOARD_DRIVER_HPP
#define BOARD_DRIVER_HPP
#include <stdint.h>

#include <drivers/can.hpp>
#include <string>

namespace arm {
class BoardDriver {
public:
    typedef enum {
        CMD_DEVICE_ID,
        CMD_HARDWARE_VERSION,
        CMD_FIRMWARE_VERSION,
        CMD_SN_CODE,
        CMD_ARM_SN_CODE,
        CMD_TYPE,
        CMD_MODE,
        CMD_RESERVER1,
        CMD_RESERVER2,
        CMD_RESERVER3,
        CMD_RESERVER4,
        CMD_RESERVER5,
        CMD_RESERVER6,
        CMD_RESERVER7,
        CMD_RESERVER8,
        CMD_RESERVER9,
        CMD_ZERE,
        CMD_POS,
        CMD_FORCE,
        CMD_SNAP_SIGNAL,
        CMD_PRODUCT_FLAG,
        CMD_LED_MODE,
    } CmdId;

    typedef enum {
        FRAME_1 = 1,
        FRAME_2,
        FRAME_3,
        FRAME_4,
    } FrameId;

    enum snap_mode_e {
        SNAP_RELEASE = 0,
        SNAP_SHORT_PRESS,
        SNAP_LONG_PRESS,
        SNAP_DOUBLE_PRESS,
    };

    BoardDriver() = default;
    ~BoardDriver(){};
    bool Init();
    uint16_t get_board_id() { return board_id_; }
    uint32_t get_ret_id() { return ret_id_; }
    std::string get_firmware_version() const { return firmware_version_; }
    std::string get_hardware_version() const { return hardware_version_; }
    std::string get_sn_code() const { return sn_code_; }
    std::string get_arm_sn_code() const { return arm_sn_code_; }
    uint32_t get_snap_mode();
    void GetCmd(uint8_t cmd_id);
    void SetCmd(uint8_t cmd_id, uint8_t framd_id, uint8_t* data);
    void SetCmd(uint8_t cmd_id, uint8_t framd_id, uint32_t data);
    virtual void CanSendMsg(can_frame_t& tx_frame) = 0;

    int get_RET_CMD() { return RET_CMD; }

protected:
    CanChannel_e can_channel_;
    uint16_t board_id_;
    std::string hardware_version_;
    std::string firmware_version_;
    std::string sn_code_;
    std::string arm_sn_code_;
    uint32_t ret_id_ = 0;
    uint32_t snap_mode_ = 0;

    int DEVICE_ID;
    int OPERATE_CMD = 0b0000;
    int GET_CMD = (OPERATE_CMD | 0);
    int SET_CMD = (OPERATE_CMD | 1);
    int RET_CMD = (OPERATE_CMD | 2);
    int GET_CMD_ID;
    int SET_CMD_ID;
    int RET_CMD_ID;
    int BROADCASE_CMD_ID = 0x7ff;
};
}  // namespace arm
#endif