//
// Created by lsy on 24-3-13.
//
//
// Created by yuchen on 23-9-2.
//

#include "upright_control_ros/synchronized_module/RosReferenceManager.h"

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <upright_control/MobileManipulatorInterface.h>

int main(int argc, char **argv) {
    const std::string robotName = "mobile_manipulator";

    // task file
//    std::vector<std::string> programArgs{};
//    ::ros::removeROSArgs(argc, argv, programArgs);
//    if (programArgs.size() <= 1) {
//        throw std::runtime_error("No task file specified. Aborting.");
//    }
//    std::string taskFileFolderName = std::string(programArgs[1]);

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mpc");
    ros::NodeHandle nodeHandle;

    // Robot interface
    const std::string taskFile =
            ros::package::getPath("mobile_manipulator_assets") + "/config/OCS2_mobile_manipulator/task.info";
    const std::string libFolder =
            ros::package::getPath("mobile_manipulator_assets") + "/auto_generated/OCS2_mobile_manipulator";
    const std::string urdfFile = ros::package::getPath("mobile_manipulator_assets") +
                                 "/description/OCS2_mobile_manipulator/urdf/ridgeback_ur5.urdf";
    upright::ControllerInterface mobileManipulatorInetface(taskFile, libFolder, urdfFile);
    // ROS ReferenceManager
    auto rosReferenceManagerPtr =
            std::make_shared<ddt::RosReferenceManager>("/mobile_manipulator",
                                                       mobileManipulatorInetface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nodeHandle);
    // MPC
    std::unique_ptr<ocs2::MultipleShootingMpc> mpc_ptr = mobileManipulatorInetface.get_mpc();
    mpc_ptr->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    // Launch MPC ROS node
    ocs2::MPC_ROS_Interface mpcNode(*mpc_ptr, robotName);
    mpcNode.launchNodes(nodeHandle);
    ROS_INFO_STREAM("MobileManipulatorMpcNode Start");
    // Successful exit
    return 0;
}