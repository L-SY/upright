
#ifndef _CAN_HPP_
#define _CAN_HPP_

#include <stdint.h>
#include <string.h>
#include <iostream>

#include <libcan/SocketCAN.h>

typedef enum CanChannel {
    CAN0,
    CAN1,
    CAN2,
    CAN3,
    CAN4,
    CAN5,
    // CAN6,
    // CAN7,
    // CAN_LAST
} CanChannel_e;

typedef void (*CanCbkFunc)( can_frame_t&);
struct CanCbkPair {
    CanCbkFunc cbkfunc;
    SocketCAN sktcan;
};

class Can {
public:
    Can();
    ~Can();

    void CanTransmit(const CanChannel_e& can_channel, can_frame_t& tx_frame);
    void SetCanCallback(const CanChannel_e& can_channel, CanCbkFunc can_callback);

    void Can0Callback(can_frame_t *frame);
    void Can1Callback(can_frame_t *frame);
    void Can2Callback(can_frame_t *frame);
    void Can3Callback(can_frame_t *frame);
    void Can4Callback(can_frame_t *frame);
    void Can5Callback(can_frame_t *frame);

    static Can can_handle;

private:
    CanCbkFunc can0_callback_;
    CanCbkFunc can1_callback_;
    CanCbkFunc can2_callback_;
    CanCbkFunc can3_callback_;
    CanCbkFunc can4_callback_;
    CanCbkFunc can5_callback_;

    SocketCAN can0_adapter_;
    SocketCAN can1_adapter_;
    SocketCAN can2_adapter_;
    SocketCAN can3_adapter_;
    SocketCAN can4_adapter_;
    SocketCAN can5_adapter_;
};

#endif