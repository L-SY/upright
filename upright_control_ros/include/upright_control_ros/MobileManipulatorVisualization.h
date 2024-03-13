//
// Created by lsy on 24-3-13.
//

#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <upright_control/MobileManipulatorInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include "upright_control/dynamics/MobileManipulatorInfo.h"

namespace ddt {

    class MobileManipulatorDummyVisualization final : public ocs2::DummyObserver {
    public:
        MobileManipulatorDummyVisualization(ros::NodeHandle& nodeHandle, const upright::ControllerInterface& interface)
                : pinocchioInterface_(interface.getPinocchioInterface()), modelInfo_(interface.getManipulatorModelInfo()) {
            launchVisualizerNode(nodeHandle);
        }

        ~MobileManipulatorDummyVisualization() override = default;

        void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& policy, const ocs2::CommandData& command) override;

    private:
        void launchVisualizerNode(ros::NodeHandle& nodeHandle);

        void publishObservation(const ros::Time& timeStamp, const ocs2::SystemObservation& observation);
        void publishTargetTrajectories(const ros::Time& timeStamp, const ocs2::TargetTrajectories& targetTrajectories);
        void publishOptimizedTrajectory(const ros::Time& timeStamp, const ocs2::PrimalSolution& policy);

        ocs2::PinocchioInterface pinocchioInterface_;
        const upright::MobileManipulatorInfo modelInfo_;
        std::vector<std::string> removeJointNames_;

        std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
        tf::TransformBroadcaster tfBroadcaster_;

        ros::Publisher stateOptimizedPublisher_;
        ros::Publisher stateOptimizedPosePublisher_;

        std::unique_ptr<ocs2::GeometryInterfaceVisualization> geometryVisualization_;
    };

}  // namespace mobile_manipulator