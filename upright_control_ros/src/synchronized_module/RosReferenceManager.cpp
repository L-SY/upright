//#include "ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h"

#include "upright_control_ros/synchronized_module/RosReferenceManager.h"
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

#include <ros/transport_hints.h>

// MPC messages
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_msgs/mpc_target_trajectories.h>

namespace ddt {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    RosReferenceManager::RosReferenceManager(std::string topicPrefix, std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
            : ReferenceManagerDecorator(std::move(referenceManagerPtr)), topicPrefix_(std::move(topicPrefix)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    void RosReferenceManager::subscribe(ros::NodeHandle& nodeHandle) {
        // ModeSchedule
        auto modeScheduleCallback = [this](const ocs2_msgs::mode_schedule::ConstPtr& msg) {
            auto modeSchedule = ocs2::ros_msg_conversions::readModeScheduleMsg(*msg);
            referenceManagerPtr_->setModeSchedule(std::move(modeSchedule));
        };
        modeScheduleSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mode_schedule>(topicPrefix_ + "_mode_schedule", 1, modeScheduleCallback);

        // TargetTrajectories
        auto targetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) {
            auto targetTrajectories = ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
            referenceManagerPtr_->setTargetTrajectories(std::move(targetTrajectories));
        };
        targetTrajectoriesSubscriber_ =
                nodeHandle.subscribe<ocs2_msgs::mpc_target_trajectories>(topicPrefix_ + "_mpc_target", 1, targetTrajectoriesCallback);
    }

}  // namespace ddt
