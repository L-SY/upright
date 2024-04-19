#include <drivers/imu_c_board.hpp>

void GetImuAngle(uint8_t *data) {
    imu_c_board.last_rev[0] = clock();
    imu_c_board.last_imu[0] = imu_c_board.imu[0];
    imu_c_board.last_imu[2] = imu_c_board.imu[2];
    imu_c_board.last_imu[1] = imu_c_board.imu[1];

    imu_c_board.imu[0] = (int16_t)(data[0] << 8 | data[1]) * M_PI / 32768.f;
    imu_c_board.imu[2] = (int16_t)(data[2] << 8 | data[3]) * M_PI / 32768.f;
    imu_c_board.imu[1] = (int16_t)(data[4] << 8 | data[5]) * M_PI / 32768.f;
    imu_c_board.connected[0] = true;
}

void GetImuGyro(uint8_t *data) {
    imu_c_board.last_rev[1] = clock();
    imu_c_board.gyro[0] = (int16_t)(data[0] << 8 | data[1]) * 50.0f / 32768.f;
    imu_c_board.gyro[1] = (int16_t)(data[2] << 8 | data[3]) * 50.0f / 32768.f;
    imu_c_board.gyro[2] = (int16_t)(data[4] << 8 | data[5]) * 50.0f / 32768.f;
    imu_c_board.connected[1] = true;
}

void GetImuAcc(uint8_t *data) {
    imu_c_board.last_rev[2] = clock();
    imu_c_board.accel[0] = (int16_t)(data[0] << 8 | data[1]) * 50.0f / 32768.f;
    imu_c_board.accel[1] = (int16_t)(data[2] << 8 | data[3]) * 50.0f / 32768.f;
    imu_c_board.accel[2] = (int16_t)(data[4] << 8 | data[5]) * 50.0f / 32768.f;
    imu_c_board.connected[2] = true;
}

void ImuCallbackFunc(can_frame_t* frame) {
    switch (frame->can_id) {
        case 0x80:
            GetImuAngle(frame->data);
            break;
        case 0x81:
            GetImuGyro(frame->data);
            break;
        case 0x82:
            GetImuAcc(frame->data);
            break;
        default:
            break;
    }
}

IMU_Float_t imu_c_board = {0};
