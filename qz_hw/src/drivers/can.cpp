#include "drivers/can.hpp"

Can::Can() {}

Can::~Can() {
    if (can0_adapter_.is_open()) {
        can0_adapter_.close();
    }
    if (can1_adapter_.is_open()) {
        can1_adapter_.close();
    }
    if (can2_adapter_.is_open()) {
        can2_adapter_.close();
    }
    if (can3_adapter_.is_open()) {
        can3_adapter_.close();
    }
    if (can4_adapter_.is_open()) {
        can4_adapter_.close();
    }
    if (can5_adapter_.is_open()) {
        can5_adapter_.close();
    }
}

void Can::CanTransmit(const CanChannel_e& can_channel, can_frame_t& tx_frame) {
    switch (can_channel) {
    case CAN0:
        can0_adapter_.transmit(&tx_frame);
        break;
    case CAN1:
        can1_adapter_.transmit(&tx_frame);
        break;
    case CAN2:  
        can2_adapter_.transmit(&tx_frame);
        break;
    case CAN3:
        can3_adapter_.transmit(&tx_frame);
        break;
    case CAN4:
        can4_adapter_.transmit(&tx_frame);
        break;
    case CAN5:
        can5_adapter_.transmit(&tx_frame);
        break;
    default:
        break;
    }
}

void Can::SetCanCallback(const CanChannel_e& can_channel, CanCbkFunc can_callback) {
    if (can_channel == CAN0) {
        can0_adapter_.reception_handler_data = (void *)this;
        can0_adapter_.reception_handler = [](can_frame_t *frame, void *ptr) { ((Can *)ptr)->Can0Callback(frame); };
        can0_adapter_.open("can0");
        can0_callback_ = can_callback;
    } else if (can_channel == CAN1) {
        can1_adapter_.reception_handler_data = (void *)this;
        can1_adapter_.reception_handler = [](can_frame_t *frame, void *ptr) { ((Can *)ptr)->Can1Callback(frame); };
        can1_adapter_.open("can1");
        can1_callback_ = can_callback;
    } else if (can_channel == CAN2) {
        can2_adapter_.reception_handler_data = (void *)this;
        can2_adapter_.reception_handler = [](can_frame_t *frame, void *ptr) { ((Can *)ptr)->Can2Callback(frame); };
        can2_adapter_.open("can2");
        can2_callback_ = can_callback;
    } else if (can_channel == CAN3) {
        can3_adapter_.reception_handler_data = (void *)this;
        can3_adapter_.reception_handler = [](can_frame_t *frame, void *ptr) { ((Can *)ptr)->Can3Callback(frame); };
        can3_adapter_.open("can3");
        can3_callback_ = can_callback;
    } else if (can_channel == CAN4) {
        can4_adapter_.reception_handler_data = (void *)this;
        can4_adapter_.reception_handler = [](can_frame_t *frame, void *ptr) { ((Can *)ptr)->Can4Callback(frame); };
        can4_adapter_.open("can4");
        can4_callback_ = can_callback;
    } else if (can_channel == CAN5) {
        can5_adapter_.reception_handler_data = (void *)this;
        can5_adapter_.reception_handler = [](can_frame_t *frame, void *ptr) { ((Can *)ptr)->Can5Callback(frame); };
        can5_adapter_.open("can5");
        can5_callback_ = can_callback;
    }
}

void Can::Can0Callback(can_frame_t *frame) {
    can_frame_t tmp_frame = *frame;
    if (can0_callback_ != nullptr) {
        can0_callback_(tmp_frame);
    } else {
        printf("Can0 callback is not set\n");
    }
}

void Can::Can1Callback(can_frame_t *frame) {
    can_frame_t tmp_frame = *frame;
    if (can1_callback_ != nullptr) {
        can1_callback_(tmp_frame);
    } else {
        printf("Can1 callback is not set\n");
    }
}

void Can::Can2Callback(can_frame_t *frame) {
    can_frame_t tmp_frame = *frame;
    if (can2_callback_ != nullptr) {
        can2_callback_(tmp_frame);
    } else {
        printf("Can2 callback is not set\n");
    }
}

void Can::Can3Callback(can_frame_t *frame) {
    can_frame_t tmp_frame = *frame;
    if (can3_callback_ != nullptr) {
        can3_callback_(tmp_frame);
    } else {
        printf("Can3 callback is not set\n");
    }
}

void Can::Can4Callback(can_frame_t *frame) {
    can_frame_t tmp_frame = *frame;
    if (can4_callback_ != nullptr) {
        can4_callback_(tmp_frame);
    } else {
        printf("Can4 callback is not set\n");
    }
}

void Can::Can5Callback(can_frame_t *frame) {
    can_frame_t tmp_frame = *frame;
    if (can5_callback_ != nullptr) {
        can5_callback_(tmp_frame);
    } else {
        printf("Can5 callback is not set\n");
    }
}

Can Can::can_handle;
