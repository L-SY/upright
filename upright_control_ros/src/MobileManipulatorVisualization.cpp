//
// Created by lsy on 24-3-13.
//
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <ros/package.h>
#include <tf/tf.h>
#include <urdf/model.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <upright_control/MobileManipulatorInterface.h>
#include <upright_control/common/implementation/AccessHelperFunctionsImpl.h>
#include <upright_control/common/implementation/FactoryFunctionsImpl.h>
#include <upright_control/dynamics/MobileManipulatorInfo.h>

#include <upright_control_ros/MobileManipulatorVisualization.h>

namespace ddt {

template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::Header &header) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->header = header;
  }
}

template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->id = startId++;
  }
}

void MobileManipulatorDummyVisualization::launchVisualizerNode(
    ros::NodeHandle &nodeHandle) {
  // load a kdl-tree from the urdf robot description and initialize the robot
  // state publisher
  ROS_INFO_STREAM("VIS INTER");
  const std::string urdfName = "robot_description";
  urdf::Model model;
  if (!model.initParam(urdfName)) {
    ROS_ERROR("URDF model load was NOT successful");
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
  }
  robotStatePublisherPtr_.reset(
      new robot_state_publisher::RobotStatePublisher(tree));
  robotStatePublisherPtr_->publishFixedTransforms(true);
  stateOptimizedPublisher_ =
      nodeHandle.advertise<visualization_msgs::MarkerArray>(
          "/mobile_manipulator/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>(
      "/mobile_manipulator/optimizedPoseTrajectory", 1);
  // Get ROS parameter
  std::string urdfFile, taskFile;
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/taskFile", taskFile);
  // read manipulator type
  upright::RobotBaseType robotBaseType = modelInfo_.robotBaseType;
  // read if self-collision checking active
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  bool activateSelfCollision = false;
  ocs2::loadData::loadPtreeValue(pt, activateSelfCollision,
                                 "selfCollision.activate", false);
  // create pinocchio interface
  ROS_INFO_STREAM("VIS BEFORE");
  ocs2::PinocchioInterface pinocchioInterface(
      upright::createPinocchioInterface(urdfFile, robotBaseType));
  // activate markers for self-collision visualization
  ROS_INFO_STREAM("VIS BEFORE");
  if (activateSelfCollision) {
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
    ocs2::loadData::loadStdVectorOfPair(taskFile,
                                        "selfCollision.collisionObjectPairs",
                                        collisionObjectPairs, true);
    ocs2::PinocchioGeometryInterface geomInterface(pinocchioInterface,
                                                   collisionObjectPairs);
    // set geometry visualization markers
    ROS_INFO_STREAM("VIS BEFORE");
    geometryVisualization_.reset(new ocs2::GeometryInterfaceVisualization(
        std::move(pinocchioInterface), geomInterface, nodeHandle));
  }
  ROS_INFO_STREAM("VIS FINISH");
}

void MobileManipulatorDummyVisualization::update(
    const ocs2::SystemObservation &observation,
    const ocs2::PrimalSolution &policy, const ocs2::CommandData &command) {
  const ros::Time timeStamp = ros::Time::now();

  publishObservation(timeStamp, observation);
  publishTargetTrajectories(timeStamp, command.mpcTargetTrajectories_);
  publishOptimizedTrajectory(timeStamp, policy);
  if (geometryVisualization_ != nullptr) {
    geometryVisualization_->publishDistances(observation.state);
  }
}

void MobileManipulatorDummyVisualization::publishObservation(
    const ros::Time &timeStamp, const ocs2::SystemObservation &observation) {
  // publish world -> base transform
  const auto r_world_base = getBasePosition(observation.state, modelInfo_);
  const Eigen::Quaternion<ocs2::scalar_t> q_world_base =
      getBaseOrientation(observation.state, modelInfo_);

  geometry_msgs::TransformStamped base_tf;
  base_tf.header.stamp = timeStamp;
  base_tf.header.frame_id = "world";
  base_tf.child_frame_id = modelInfo_.baseFrame;
  base_tf.transform.translation =
      ocs2::ros_msg_helpers::getVectorMsg(r_world_base);
  base_tf.transform.rotation =
      ocs2::ros_msg_helpers::getOrientationMsg(q_world_base);
  tfBroadcaster_.sendTransform(base_tf);

  // publish joints transforms
  const auto j_arm = upright::getArmJointAngles(observation.state, modelInfo_);
  std::map<std::string, ocs2::scalar_t> jointPositions;
  for (size_t i = 0; i < modelInfo_.dofNames.size(); i++) {
    jointPositions[modelInfo_.dofNames[i]] = j_arm(i);
    ROS_INFO_STREAM(modelInfo_.dofNames[i] << ": " << j_arm(i));
  }
  robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
}

void MobileManipulatorDummyVisualization::publishTargetTrajectories(
    const ros::Time &timeStamp,
    const ocs2::TargetTrajectories &targetTrajectories) {
  // publish command transform
  const Eigen::Vector3d eeDesiredPosition =
      targetTrajectories.stateTrajectory.back().head(3);
  Eigen::Quaterniond eeDesiredOrientation;
  eeDesiredOrientation.coeffs() =
      targetTrajectories.stateTrajectory.back().tail(4);
  geometry_msgs::TransformStamped command_tf;
  command_tf.header.stamp = timeStamp;
  command_tf.header.frame_id = "world";
  command_tf.child_frame_id = "command";
  command_tf.transform.translation =
      ocs2::ros_msg_helpers::getVectorMsg(eeDesiredPosition);
  command_tf.transform.rotation =
      ocs2::ros_msg_helpers::getOrientationMsg(eeDesiredOrientation);
  tfBroadcaster_.sendTransform(command_tf);
}

void MobileManipulatorDummyVisualization::publishOptimizedTrajectory(
    const ros::Time &timeStamp, const ocs2::PrimalSolution &policy) {
  const ocs2::scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const std::array<ocs2::scalar_t, 3> red{0.6350, 0.0780, 0.1840};
  const std::array<ocs2::scalar_t, 3> blue{0, 0.4470, 0.7410};
  const auto &mpcStateTrajectory = policy.stateTrajectory_;
  visualization_msgs::MarkerArray markerArray;

  // Base trajectory
  std::vector<geometry_msgs::Point> baseTrajectory;
  baseTrajectory.reserve(mpcStateTrajectory.size());
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // End effector trajectory
  const auto &model = pinocchioInterface_.getModel();
  auto &data = pinocchioInterface_.getData();
  std::vector<geometry_msgs::Point> endEffectorTrajectory;
  endEffectorTrajectory.reserve(mpcStateTrajectory.size());
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
                [&](const Eigen::VectorXd &state) {
                  pinocchio::forwardKinematics(model, data, state);
                  pinocchio::updateFramePlacements(model, data);
                  const auto eeIndex = model.getBodyId(modelInfo_.eeFrame);
                  const ocs2::vector_t eePosition =
                      data.oMf[eeIndex].translation();
                  endEffectorTrajectory.push_back(
                      ocs2::ros_msg_helpers::getPointMsg(eePosition));
                });

  markerArray.markers.emplace_back(ocs2::ros_msg_helpers::getLineMsg(
      std::move(endEffectorTrajectory), blue, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "EE Trajectory";

  // Extract base pose from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
                [&](const ocs2::vector_t &state) {
                  // extract from observation
                  const auto r_world_base = getBasePosition(state, modelInfo_);
                  const Eigen::Quaternion<ocs2::scalar_t> q_world_base =
                      getBaseOrientation(state, modelInfo_);

                  // convert to ros message
                  geometry_msgs::Pose pose;
                  pose.position =
                      ocs2::ros_msg_helpers::getPointMsg(r_world_base);
                  pose.orientation =
                      ocs2::ros_msg_helpers::getOrientationMsg(q_world_base);
                  baseTrajectory.push_back(pose.position);
                  poseArray.poses.push_back(std::move(pose));
                });

  markerArray.markers.emplace_back(ocs2::ros_msg_helpers::getLineMsg(
      std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "Base Trajectory";

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
               ocs2::ros_msg_helpers::getHeaderMsg("world", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ocs2::ros_msg_helpers::getHeaderMsg("world", timeStamp);

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);
}
} // namespace ddt
