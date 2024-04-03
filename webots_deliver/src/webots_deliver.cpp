//
// Created by lsy on 24-3-1.
//
#include <webots_deliver/webots_deliver.h>


using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    ros::init(argc, argv, "ros_webots_deliver");
    ros::NodeHandle nh("~");
    int timeStep = (int) robot->getBasicTimeStep();

    webots_deliver::RosWebotsDeliver *ros_webots_deliver = new webots_deliver::RosWebotsDeliver(nh);
    ros_webots_deliver->initVelocityMode(robot);

    while (ros::ok() && robot->step(timeStep) != -1) {
        ros::spinOnce();
        ros_webots_deliver->velocityModeUpdate(robot);
    };

    delete robot;
    return 0;
}