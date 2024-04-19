#include <airbot/board_driver.hpp>

#include <chrono>
#include <thread>

uint16_t crc16(uint8_t *data, uint8_t len) {
    uint16_t crc = 0xffff;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xa001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

bool crc_check(uint8_t *data, uint8_t len) {
    uint16_t crc = crc16(data, len - 2);
    return (crc & 0xff) == data[len - 2] && (crc >> 8) == data[len - 1];
}


namespace arm {

bool BoardDriver::Init() {
    DEVICE_ID = get_board_id();
    GET_CMD_ID = (DEVICE_ID | GET_CMD << 7);
    SET_CMD_ID = (DEVICE_ID | SET_CMD << 7);
    RET_CMD_ID = (DEVICE_ID | RET_CMD << 7);
    GetCmd(CMD_DEVICE_ID);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    GetCmd(CMD_HARDWARE_VERSION);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    GetCmd(CMD_FIRMWARE_VERSION);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    GetCmd(CMD_SN_CODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    GetCmd(CMD_ARM_SN_CODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    printf(
        "board_id: %d\thardware_version: %s\tfirmware_version: %s\tsn_code: %s\t "
        "arm_sn_code: %s\n",
        DEVICE_ID, hardware_version_.c_str(), firmware_version_.c_str(), sn_code_.c_str(), arm_sn_code_.c_str());
    if (ret_id_ != DEVICE_ID) {
        printf("board %d id error or device lost connection!!!\n", DEVICE_ID);
        return false;
    }
    if (hardware_version_ < "V2.0.0" || hardware_version_ == "V255.255.255") {
        printf("board %d hardware version error!!!\n", DEVICE_ID);
        return false;
    }
    if (firmware_version_ < "V2.4.0" || firmware_version_ == "V255.255.255") {
        printf("board %d firmware version error!!!\n", DEVICE_ID);
        return false;
    }
    if (sn_code_ < "0000000000000000" || sn_code_ > "zzzzzzzzzzzzzzzz") {
        printf("board %d sn_code error!!!\n", DEVICE_ID);
        return false;
    }
    if (arm_sn_code_ < "0000000000000000" || arm_sn_code_ > "zzzzzzzzzzzzzzzz") {
        printf("board %d arm_sn_code error!!!\n", DEVICE_ID);
        return false;
    }
    return true;
}

void BoardDriver::GetCmd(uint8_t cmd_id) {
    can_frame_t tx_frame;
    tx_frame.can_id = GET_CMD_ID;
    tx_frame.can_dlc = 2;
    tx_frame.data[0] = cmd_id;
    tx_frame.data[1] = FRAME_1;
    CanSendMsg(tx_frame);
}

void BoardDriver::SetCmd(uint8_t cmd_id, uint8_t framd_id, uint8_t *data) {
    can_frame_t tx_frame;
    tx_frame.can_id = SET_CMD_ID;
    tx_frame.can_dlc = 8;
    tx_frame.data[0] = cmd_id;
    tx_frame.data[1] = framd_id;
    for (int i = 0; i < 4; i++) {
        tx_frame.data[i + 2] = data[i];
    }
    uint16_t crc = crc16(tx_frame.data, 6);
    tx_frame.data[6] = (uint8_t)(crc & 0x00FF);
    tx_frame.data[7] = (uint8_t)((crc & 0xFF00) >> 8);
    CanSendMsg(tx_frame);
}

void BoardDriver::SetCmd(uint8_t cmd_id, uint8_t framd_id, uint32_t data) {
    can_frame_t tx_frame;
    tx_frame.can_id = SET_CMD_ID;
    tx_frame.can_dlc = 8;
    tx_frame.data[0] = cmd_id;
    tx_frame.data[1] = framd_id;
    for (int i = 0; i < 4; i++) {
        tx_frame.data[i + 2] = (uint8_t)(data >> (i * 8));
    }
    uint16_t crc = crc16(tx_frame.data, 6);
    tx_frame.data[6] = (uint8_t)(crc & 0x00FF);
    tx_frame.data[7] = (uint8_t)((crc & 0xFF00) >> 8);
    CanSendMsg(tx_frame);
}

uint32_t BoardDriver::get_snap_mode() {
    uint32_t res = snap_mode_;
    snap_mode_ = SNAP_RELEASE;
    return res;
}

}  // namespace arm