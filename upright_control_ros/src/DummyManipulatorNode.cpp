//
// Created by lsy on 24-3-13.
//

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <upright_control/MobileManipulatorInterface.h>

#include "upright_control_ros/MobileManipulatorVisualization.h"

int main(int argc, char** argv) {
    const std::string robotName = "mobile_manipulator";

    // task file
//    std::vector<std::string> programArgs{};
//    ::ros::removeROSArgs(argc, argv, programArgs);
//    if (programArgs.size() <= 1) {
//        throw std::runtime_error("No task file specified. Aborting.");
//    }
//    std::string taskFileFolderName = std::string(programArgs[1]);

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mrt");
    ros::NodeHandle nodeHandle;

    // Interface
    const std::string taskFile = ros::package::getPath("upright_control") + "/config" + "/task.info";
    const std::string libFolder = ros::package::getPath("mobile_manipulator_assets") + "/auto_generated";
    const std::string urdfFile = ros::package::getPath("mobile_manipulator_assets") + "/description/urdf/ridgeback_ur5.urdf";
    upright::ControllerInterface mobileManipulatorInetface(taskFile, libFolder, urdfFile);

    // MRT
    ocs2::MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&mobileManipulatorInetface.getRollout());
    mrt.launchNodes(nodeHandle);

    // Visualization
    auto manipulatorDummyVisualization = std::make_shared<ddt::MobileManipulatorDummyVisualization>(nodeHandle, mobileManipulatorInetface);

    // Dummy balance
    ocs2::MRT_ROS_Dummy_Loop dummyMobileManipulator(mrt, mobileManipulatorInetface.mpcSettings().mrtDesiredFrequency_,
                                                    mobileManipulatorInetface.mpcSettings().mpcDesiredFrequency_);
    dummyMobileManipulator.subscribeObservers({manipulatorDummyVisualization});

    // initial state
    ocs2::SystemObservation initObservation;
    initObservation.state = mobileManipulatorInetface.getInitialState();
//    initObservation.input.setZero(rm::INPUT_DIM);
    initObservation.time = 0.0;

    // initial command
    const ocs2::TargetTrajectories initTargetTrajectories({initObservation.time}, {initObservation.state}, {initObservation.input});

    // Run dummy (loops while ros is ok)
    dummyMobileManipulator.run(initObservation, initTargetTrajectories);

    // Successful exit
    return 0;
}