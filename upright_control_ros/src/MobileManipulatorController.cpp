//
// Created by lsy on 24-3-6.
//

#include "upright_control_ros/MobileManipulatorController.h"

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <pluginlib/class_list_macros.h>

namespace ddt {

bool MobileManipulatorController::init(hardware_interface::RobotHW *robot_hw,
                                       ros::NodeHandle &root_nh,
                                       ros::NodeHandle &controller_nh) {
  // Initialize OCS2
  std::string taskFile;
  std::string libFolder;
  std::string urdfFile;
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/libFolder", libFolder);
  controller_nh.getParam("/urdfFile", urdfFile);
  mobileManipulatorInterface_ = std::make_shared<upright::ControllerInterface>(
      taskFile, libFolder, urdfFile);
  setupMpc(controller_nh);
  setupMrt();

  // Visualization
  ros::NodeHandle nh;
  visualizer_ = std::make_shared<ddt::MobileManipulatorDummyVisualization>(
      nh, *mobileManipulatorInterface_);

  currentObservation_.time = 0.0;
  currentObservation_.state.setZero(STATE_DIM);
  currentObservation_.input.setZero(INPUT_DIM);

  jointVelLast_.resize(10);
  lastTime_ = ros::Time::now();

  // Hardware interface
  auto *effortJointInterface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  jointHandles_.push_back(effortJointInterface->getHandle("left_j3"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_j3"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint1"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint2"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint3"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint4"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint5"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint6"));
  //  auto *velocityJointInterface =
  //      robot_hw->get<hardware_interface::VelocityJointInterface>();
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("front_left_wheel"));
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("front_right_wheel"));
  //  jointHandles_.push_back(velocityJointInterface->getHandle("rear_left_wheel"));
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("rear_right_wheel"));
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("ur_arm_shoulder_pan_joint"));
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("ur_arm_shoulder_lift_joint"));
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("ur_arm_elbow_joint"));
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("ur_arm_wrist_1_joint"));
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("ur_arm_wrist_2_joint"));
  //  jointHandles_.push_back(
  //      velocityJointInterface->getHandle("ur_arm_wrist_3_joint"));
  controlState_ = UPRIGHT;

  // Odom TF
  odom2base_.header.frame_id = "world";
  odom2base_.header.stamp = ros::Time::now();
  odom2base_.child_frame_id = "diablo_base_link";

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, 0);
  odom2base_.transform.rotation.x = quaternion.x();
  odom2base_.transform.rotation.y = quaternion.y();
  odom2base_.transform.rotation.z = quaternion.z();
  odom2base_.transform.rotation.w = quaternion.w();

  odom2base_.transform.translation.x = 0;
  odom2base_.transform.translation.y = 0;
  odom2base_.transform.translation.z = 0;
  tfRtBroadcaster_.init(root_nh);
  tfRtBroadcaster_.sendTransform(odom2base_);

  ROS_INFO_STREAM("MobileManipulatorController Init Finish!");
  return true;
}

void MobileManipulatorController::update(const ros::Time &time,
                                         const ros::Duration &period) {
  // Update the current state of the system
  updateTfOdom(time, period);
  updateStateEstimation(time, period);
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  switch (controlState_) {
  case ControllerState::NORMAL:
    normal(time, period);
    break;
  case ControllerState::UPRIGHT: // Add when normal is useful
    upright(time, period);
    break;
  case ControllerState::UPRIGHT_AVOID: // Add when normal is useful
    uprightAvoid(time, period);
    break;
  case ControllerState::EE_SPACE_LOCK: // Add when normal is useful
    EESpaceLock(time, period);
    break;
  }

  // Visualization
  visualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(),
                      mpcMrtInterface_->getCommand());

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(
      ocs2::ros_msg_conversions::createObservationMsg(currentObservation_));
}

void MobileManipulatorController::updateTfOdom(const ros::Time &time,
                                               const ros::Duration &period) {
  ocs2::vector_t position(3);
  position(0) = odom2base_.transform.translation.x;
  position(1) = odom2base_.transform.translation.y;
  position(2) = 0;
  double yaw = yawFromQuat(odom2base_.transform.rotation);
  yaw += currentObservation_.state(10) * period.toSec();
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  odom2base_.transform.rotation.x = quaternion.x();
  odom2base_.transform.rotation.y = quaternion.y();
  odom2base_.transform.rotation.z = quaternion.z();
  odom2base_.transform.rotation.w = quaternion.w();

  position(0) += currentObservation_.state(9) * period.toSec() *
                 cos(currentObservation_.state(2));
  position(1) += currentObservation_.state(9) * period.toSec() *
                 sin(currentObservation_.state(2));

  odom2base_.header.stamp = time;
  odom2base_.transform.translation.x = position(0);
  odom2base_.transform.translation.y = position(1);
  odom2base_.transform.translation.z = position(2);

  tfRtBroadcaster_.sendTransform(odom2base_);
}

void MobileManipulatorController::updateStateEstimation(
    const ros::Time &time, const ros::Duration &period) {
  ocs2::vector_t jointPos(CONTROL_DIM), jointVel(CONTROL_DIM);
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    jointPos(i) = jointHandles_[i].getPosition();
    jointVel(i) = jointHandles_[i].getVelocity();
  }

  currentObservation_.state(0) = odom2base_.transform.translation.x;
  currentObservation_.state(1) = odom2base_.transform.translation.y;
  double yaw = yawFromQuat(odom2base_.transform.rotation);
  currentObservation_.state(2) = yaw;
  double xV = (jointVel(0) + jointVel(1)) * wheeelR_ / 2;
  double wV = (-jointVel(0) + jointVel(1)) / baseL_;
  currentObservation_.state(9) = xV;
  currentObservation_.state(10) = wV;
  double xVLast = (jointVelLast_(0) + jointVelLast_(1)) * wheeelR_ / 2;
  double wVLast = (-jointVelLast_(0) + jointVelLast_(1)) / baseL_;

  currentObservation_.state(18) = (xV - xVLast) / (time - lastTime_).toSec();
  currentObservation_.state(19) = (wV - wVLast) / (time - lastTime_).toSec();
  //      for arm
  for (int i = 0; i < 6; ++i) {
    currentObservation_.state(3 + i) = jointPos(2 + i);
    currentObservation_.state(11 + i) = jointVel(2 + i);
    currentObservation_.state(19 + i) =
        (jointVel(2 + i) - jointVelLast_(2 + i)) / (time - lastTime_).toSec();
  }
  currentObservation_.time += period.toSec();
  jointVelLast_ = jointVel;
  lastTime_ = time;
}

void MobileManipulatorController::starting(const ros::Time &time) {
  updateTfOdom(time, ros::Duration(0.001));
  updateStateEstimation(time, ros::Duration(0.001));
  currentObservation_.input.setZero(INPUT_DIM);
  currentObservation_.state = mobileManipulatorInterface_->getInitialState();
  ocs2::vector_t initDesiredState(7);

  initDesiredState.head(3) << 0, 0, 1;
  initDesiredState.tail(4)
      << Eigen::Quaternion<ocs2::scalar_t>(1, 0, 0, 0).coeffs();
  ocs2::TargetTrajectories targetTrajectories({currentObservation_.time},
                                              {initDesiredState},
                                              {currentObservation_.input});
  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(
      targetTrajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(
        mobileManipulatorInterface_->mpcSettings().mrtDesiredFrequency_)
        .sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");
  mpcRunning_ = true;
}

MobileManipulatorController::~MobileManipulatorController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "###############################################################"
               "#########";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds()
            << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds()
            << "[ms]." << std::endl;
}

void MobileManipulatorController::setupMpc(ros::NodeHandle &nh) {
  const std::string robotName = "diablo_manipulator";

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<ddt::RosReferenceManager>(
      "/mobile_manipulator",
      mobileManipulatorInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  // MPC
  mpc_ = mobileManipulatorInterface_->get_mpc();
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(
      "/mobile_manipulator_mpc_observation", 1);
}

//``````````````````````OCS2 mobile_manipulator MRT
// setting``````````````````````````````
//    // MRT
//    MRT_ROS_Interface mrt(robotName);
//    mrt.initRollout(&interface.getRollout());
//    mrt.launchNodes(nodeHandle);
//
//    // Dummy MRT
//    MRT_ROS_Dummy_Loop dummy(mrt,
//    interface.mpcSettings().mrtDesiredFrequency_,
//    interface.mpcSettings().mpcDesiredFrequency_);
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
      } catch (const std::exception &e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  // TODO : Add config in task.info
  ocs2::setThreadPriority(50, mpcThread_);
}

void MobileManipulatorController::normal(const ros::Time &time,
                                         const ros::Duration &period) {
  static ocs2::vector_t lastOptimizedState(25);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode =
      0; // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time,
                                   currentObservation_.state, optimizedState,
                                   optimizedInput, plannedMode);

  currentObservation_.input = optimizedInput;
  int index = 0;
  for (auto joint : jointHandles_) {
    joint.setCommand(currentObservation_.input(index));
    index++;
  }

  lastOptimizedState = optimizedState;
}

void MobileManipulatorController::upright(const ros::Time &time,
                                          const ros::Duration &period) {
  static ocs2::vector_t lastOptimizedState(25);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();
  // Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode =
      0; // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time,
                                   currentObservation_.state, optimizedState,
                                   optimizedInput, plannedMode);
  currentObservation_.input = optimizedInput;

  //      for wheel control
  double xV = optimizedState(9);
  double wV = optimizedState(10);

  jointHandles_[0].setCommand(xV / wheeelR_ - baseL_ * wV / (2 * wheeelR_));
  jointHandles_[1].setCommand(xV / wheeelR_ + baseL_ * wV / (2 * wheeelR_));
  //  ROS_INFO_STREAM("left V  =" << xV / wheeelR_ - baseL_ * wV / (2 *
  //  wheeelR_)); ROS_INFO_STREAM("right V  =" << xV / wheeelR_ + baseL_ * wV /
  //  (2 * wheeelR_));
  //      for arm control
  for (int i = 0; i < 6; ++i)
    jointHandles_[i + 2].setCommand(optimizedState(11 + i));

  lastOptimizedState = optimizedState;
}

} // namespace ddt
PLUGINLIB_EXPORT_CLASS(ddt::MobileManipulatorController,
                       controller_interface::ControllerBase)