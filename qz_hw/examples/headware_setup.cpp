#include <ros/ros.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <thread>

#include "app/robot.hpp"

extern std::thread RobotThread;

int main(int argc, char **argv) {
    ros::init(argc, argv, "arx7");
    ROS_WARN("arx7 start");

    ros::NodeHandle node;

    bool chassis_with_arm;
    bool with_gripper;

    ros::param::get("~with_arm", chassis_with_arm);
    ROS_INFO("chassis_with_arm: %s", (chassis_with_arm?"true":"false"));
    ros::param::get("~with_gripper", with_gripper);
    std::cout << "with_gripper: " << with_gripper << std::endl;

    Robot RobotE(node, chassis_with_arm, with_gripper);

    RobotThread = std::thread([&]() {
        RobotE.Init();
        ROS_WARN("arx7 init finished");
        auto cstart = std::chrono::high_resolution_clock::now();
        while (ros::ok()) {
            cstart = std::chrono::high_resolution_clock::now();
            RobotE.Handle();
            std::this_thread::sleep_until(cstart + std::chrono::milliseconds(2));
        }
    });

    auto start = std::chrono::high_resolution_clock::now();
    while (ros::ok()) {
        start = std::chrono::high_resolution_clock::now();
        ros::spinOnce();
        std::this_thread::sleep_until(start + std::chrono::milliseconds(2));
    }

    RobotThread.join();
    std::cout << "robot thread destroyed" << std::endl;

    return 0;
}
