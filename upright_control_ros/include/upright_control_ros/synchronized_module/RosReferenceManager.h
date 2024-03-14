#pragma once

#include <memory>
#include <string>
#include <utility>

#include <ros/ros.h>

#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>

namespace ddt {

/**
 * Decorates ReferenceManager with ROS subscribers to receive ModeSchedule and TargetTrajectories through ROS messages.
 */
class RosReferenceManager final : public ocs2::ReferenceManagerDecorator {
    public:
        /**
         * Constructor which decorates referenceManagerPtr.
         *
         * @param [in] topicPrefix: The ReferenceManager will subscribe to "topicPrefix_mode_schedule" and "topicPrefix_mpc_target"
         * @param [in] referenceManagerPtr: The ReferenceManager which will be decorated with ROS subscribers functionalities.
         * topics to receive user-commanded ModeSchedule and TargetTrajectories respectively.
         */
        explicit RosReferenceManager(std::string topicPrefix, std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr);

        ~RosReferenceManager() override = default;

        /**
         * Creates a pointer to RosReferenceManager using a the derived class of type ReferenceManagerInterface, i.e.
         * DerivedReferenceManager(args...).
         *
         * @param [in] topicPrefix: The ReferenceManager will subscribe to "topicPrefix_mode_schedule" and "topicPrefix_mpc_target"
         * topics to receive user-commanded ModeSchedule and TargetTrajectories respectively.
         * @param args: arguments to forward to the constructor of DerivedReferenceManager
         */
        template <class ReferenceManagerType, class... Args>
        static std::unique_ptr<RosReferenceManager> create(const std::string& topicPrefix, Args&&... args);

        /**
         * Subscribers to "topicPrefix_mode_schedule" and "topicPrefix_mpc_target" topics to receive respectively:
         * (1) ModeSchedule : The predefined mode schedule for time-triggered hybrid systems.
         * (2) TargetTrajectories : The commanded TargetTrajectories.
         */
        void subscribe(ros::NodeHandle& nodeHandle);

    private:
        const std::string topicPrefix_;

        ::ros::Subscriber modeScheduleSubscriber_;
        ::ros::Subscriber targetTrajectoriesSubscriber_;
    };

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    template <class ReferenceManagerType, class... Args>
    std::unique_ptr<RosReferenceManager> RosReferenceManager::create(const std::string& topicPrefix, Args&&... args) {
        auto referenceManagerPtr = std::make_shared<ReferenceManagerType>(std::forward<Args>(args)...);
        return std::unique_ptr<RosReferenceManager>(new RosReferenceManager(topicPrefix, std::move(referenceManagerPtr)));
    }

}  // namespace ddt
