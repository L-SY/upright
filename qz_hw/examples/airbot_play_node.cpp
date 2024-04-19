#include <ros/ros.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <thread>

#include <app/airbot_play.hpp>

extern std::thread AitbotPlayThread;

int main(int argc, char **argv) {
    ros::init(argc, argv, "airbot_play");
    ROS_WARN("airbot play node start");

    ros::NodeHandle node;

    int can_device;
    bool is_teacher;
    bool with_gripper;

    ros::param::get("~can_device", can_device);
    CanChannel_e can_channel = static_cast<CanChannel_e>(can_device);
    ros::param::get("~is_teacher", is_teacher);
    ros::param::get("~with_gripper", with_gripper);
    std::cout << "is_teacher: " << is_teacher << std::endl;
    std::cout << "with_gripper: " << with_gripper << std::endl;

    AirbotPlay armE(node, can_channel, is_teacher, with_gripper);

    AitbotPlayThread = std::thread([&]() {
        ROS_WARN("airbot thread start");
        auto cstart = std::chrono::high_resolution_clock::now();
        while (ros::ok()) {
            cstart = std::chrono::high_resolution_clock::now();
            armE.Handle();
            std::this_thread::sleep_until(cstart + std::chrono::milliseconds(10));
        }
    });

    auto start = std::chrono::high_resolution_clock::now();
    while (ros::ok()) {
        start = std::chrono::high_resolution_clock::now();
        ros::spinOnce();
        std::this_thread::sleep_until(start + std::chrono::milliseconds(10));
    }

    AitbotPlayThread.join();
    std::cout << "Airbot thread destroyed" << std::endl;

    return 0;
}
