//
// Created by lsy on 24-3-1.
//
#include <webots_deliver/webots_deliver.h>


using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    ros::init(argc, argv, "ros_webots_deliver");
    ros::NodeHandle nh("~");
    int timeStep = (int)robot->getBasicTimeStep();
//    webots_deliver::getMotorInfo(robot);
    webots_deliver::enablePositionSensor(robot,100);
    webots_deliver::setVelocityMode(robot);
//    Only for test basic command
//    std::vector<double> position_cmd = {3.0, 1.57, 1.0, -1.0, 1.57, 1.57};

    webots_deliver::RosWebotsDeliver* ros_webots_deliver = new webots_deliver::RosWebotsDeliver(nh);
    std::vector<double> velocity_cmd = {0., 0., 0., 0., 0., 0.};
    while (ros::ok() && robot->step(timeStep) != -1) {
        ros::spinOnce();
        webots_deliver::getMotorPositions(robot, true);
//        Only test ros webots connect
//        if (!ros_webots_deliver->mpc_policy_.inputTrajectory.empty()) {
//            for (int i = 0; i < ros_webots_deliver->INPUT_DIM; ++i) {
//                velocity_cmd[i] = static_cast<double>(ros_webots_deliver->mpc_policy_.inputTrajectory.begin()->value[i]);
//            }
//        }
        for (int i = 0; i < ros_webots_deliver->INPUT_DIM; ++i)
            velocity_cmd[i] = ros_webots_deliver->rosMotorCmd_[i];
        ros_webots_deliver->pubWebotsMotorState(robot);
        webots_deliver::setMotorCommands(robot,velocity_cmd,webots_deliver::VELOCITY);

    };
    delete robot;
    return 0;
}