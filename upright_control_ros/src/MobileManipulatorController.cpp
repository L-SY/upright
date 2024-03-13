//
// Created by lsy on 24-3-6.
//

#include "upright_control_ros/MobileManipulatorController.h"

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <pluginlib/class_list_macros.h>



namespace ddt{

    bool MobileManipulatorController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                             ros::NodeHandle &controller_nh) {
        // Initialize OCS2
        std::string taskFile;
        std::string libFolder;
        std::string urdfFile;
        controller_nh.getParam("/taskFile", taskFile);
        controller_nh.getParam("/libFolder", libFolder);
        controller_nh.getParam("/urdfFile", urdfFile);

        mobileManipulatorInterface_ = std::make_shared<ocs2::mobile_manipulator::MobileManipulatorInterface>(taskFile, libFolder, urdfFile);
        setupMpc(controller_nh);
        setupMrt();
//    // Visualization
//    ros::NodeHandle nh;
//    visualizer_ = std::make_shared<ocs2::mobile_manipulator::MobileManipulatorDummyVisualization>(nh, *mobileManipulatorInterface_);

        currentObservation_.time = 0.0;
        currentObservation_.state.setZero(STATE_DIM);
        currentObservation_.input.setZero(INPUT_DIM);

        // Hardware interface
        auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
        jointHandles_.push_back(effortJointInterface->getHandle("ur_arm_shoulder_pan_joint"));
        jointHandles_.push_back(effortJointInterface->getHandle("ur_arm_shoulder_lift_joint"));
        jointHandles_.push_back(effortJointInterface->getHandle("ur_arm_elbow_joint"));
        jointHandles_.push_back(effortJointInterface->getHandle("ur_arm_wrist_1_joint"));
        jointHandles_.push_back(effortJointInterface->getHandle("ur_arm_wrist_2_joint"));
        jointHandles_.push_back(effortJointInterface->getHandle("ur_arm_wrist_3_joint"));

        return true;
    }

    void MobileManipulatorController::update(const ros::Time &time, const ros::Duration &period) {
        // Update the current state of the system
        updateStateEstimation(time, period);
        mpcMrtInterface_->setCurrentObservation(currentObservation_);

        switch (controlState_) {
            case ControllerState::NORMAL:
                normal(time, period);
                break;
            case ControllerState::UPRIGHT: //Add when normal is useful
                update(time, period);
                break;
            case ControllerState::UPRIGHT_AVOID: //Add when normal is useful
                uprightAvoid(time, period);
                break;
            case ControllerState::EE_SPACE_LOCK: //Add when normal is useful
                EESpaceLock(time, period);
                break;
        }

        // Visualization
//    visualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

        // Publish the observation. Only needed for the command interface
        observationPublisher_.publish(ocs2::ros_msg_conversions::createObservationMsg(currentObservation_));
    }

    void MobileManipulatorController::updateStateEstimation(const ros::Time& time, const ros::Duration& period){
        ocs2::vector_t jointPos(CONTROL_DIM), jointVel(CONTROL_DIM);
        for (size_t i = 0; i < CONTROL_DIM; ++i) {
            jointPos(i) = jointHandles_[i].getPosition();
            jointVel(i) = jointHandles_[i].getVelocity();
            currentObservation_.state(i) = jointPos(i);
        }

        currentObservation_.time += period.toSec();
    }
    void MobileManipulatorController::starting(const ros::Time &time) {
        updateStateEstimation(time,ros::Duration(0.001));
        currentObservation_.input.setZero(INPUT_DIM);
        ocs2::vector_t initDesiredState;
        initDesiredState.setZero(7);
        initDesiredState(0) = 0.25;
        initDesiredState(2) = 0.3;
        initDesiredState(6) = 1;
        ocs2::TargetTrajectories targetTrajectories({currentObservation_.time}, {initDesiredState}, {currentObservation_.input});

        // Set the first observation and command and wait for optimization to finish
        mpcMrtInterface_->setCurrentObservation(currentObservation_);
        mpcMrtInterface_->getReferenceManager().setTargetTrajectories(targetTrajectories);
        ROS_INFO_STREAM("Waiting for the initial policy ...");
        while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
            mpcMrtInterface_->advanceMpc();
            ros::WallRate(mobileManipulatorInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
        }
        ROS_INFO_STREAM("Initial policy has been received.");
        mpcRunning_ = true;
    }

    MobileManipulatorController::~MobileManipulatorController() {
        controllerRunning_ = false;
        if (mpcThread_.joinable()) {
            mpcThread_.join();
        }
        std::cerr << "########################################################################";
        std::cerr << "\n### MPC Benchmarking";
        std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
        std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    }

    void MobileManipulatorController::setupMpc(ros::NodeHandle &nh) {
        const std::string robotName = "ur5";

        // ROS ReferenceManager
        auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>("/mobile_manipulator",mobileManipulatorInterface_->getReferenceManagerPtr());
        rosReferenceManagerPtr->subscribe(nh);

        // MPC
        mpc_ = std::make_shared<ocs2::GaussNewtonDDP_MPC>(mobileManipulatorInterface_->mpcSettings(), mobileManipulatorInterface_->ddpSettings(), mobileManipulatorInterface_->getRollout(),
                                                          mobileManipulatorInterface_->getOptimalControlProblem(), mobileManipulatorInterface_->getInitializer());
        mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

        observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>("/mobile_manipulator_mpc_observation", 1);
    }

//``````````````````````OCS2 mobile_manipulator MRT setting``````````````````````````````
//    // MRT
//    MRT_ROS_Interface mrt(robotName);
//    mrt.initRollout(&interface.getRollout());
//    mrt.launchNodes(nodeHandle);
//
//    // Dummy MRT
//    MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
//    dummy.subscribeObservers({dummyVisualization});
//````````````````````````````````````````````````````````````````````````````````````````

    void MobileManipulatorController::setupMrt() {
        mpcMrtInterface_ = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
        mpcMrtInterface_->initRollout(&mobileManipulatorInterface_->getRollout());
        mpcTimer_.reset();

        controllerRunning_ = true;
        mpcThread_ = std::thread([&]() {
            while (controllerRunning_) {
                try {
                    ocs2::executeAndSleep(
                            [&]() {
                                if (mpcRunning_) {
                                    mpcTimer_.startTimer();
                                    mpcMrtInterface_->advanceMpc();
                                    mpcTimer_.endTimer();
                                }
                            },
                            mobileManipulatorInterface_->mpcSettings().mpcDesiredFrequency_);
                } catch (const std::exception& e) {
                    controllerRunning_ = false;
                    ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
                    stopRequest(ros::Time());
                }
            }
        });
        ocs2::setThreadPriority(mobileManipulatorInterface_->ddpSettings().threadPriority_, mpcThread_);
    }

    void MobileManipulatorController::normal(const ros::Time &time, const ros::Duration &period) {
        static ocs2::vector_t lastOptimizedState(7);

        // Load the latest MPC policy
        mpcMrtInterface_->updatePolicy();

        // Evaluate the current policy
        ocs2::vector_t optimizedState, optimizedInput;
        size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
        mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

        currentObservation_.input = optimizedInput;
        int index = 0;
        for (auto joint:jointHandles_) {
            joint.setCommand(currentObservation_.input(index));
            index++;
        }

        lastOptimizedState = optimizedState;
    }
}// namespace ddt
PLUGINLIB_EXPORT_CLASS(ddt::MobileManipulatorController, controller_interface::ControllerBase)