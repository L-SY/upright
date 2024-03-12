//
// Created by lsy on 24-3-6.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>


namespace generally {
    class GenerallyHW : public hardware_interface::RobotHW {
    public:
        GenerallyHW() = default;
        /** \brief Get necessary params from param server. Init hardware_interface.
         *
         * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission
         *
         * @param root_nh Root node-handle of a ROS node.
         * @param robot_hw_nh Node-handle for robot hardware.
         * @return True when init successful, False when failed.
         */
        bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

    protected:
        // Interface
        hardware_interface::JointStateInterface jointStateInterface_;  // NOLINT(misc-non-private-member-variables-in-classes)
        hardware_interface::ImuSensorInterface imuSensorInterface_;    // NOLINT(misc-non-private-member-variables-in-classes)
        hardware_interface::EffortJointInterface effortJointInterface_;
        std::vector<hardware_interface::JointHandle> effort_joint_handles_{}; // NOLINT(misc-non-private-member-variables-in-classes)
        // URDF model of the robot
        std::shared_ptr<urdf::Model> urdfModel_;  // NOLINT(misc-non-private-member-variables-in-classes)

    private:
        /** \brief Load urdf of robot from param server.
         *
         * Load urdf of robot from param server.
         *
         * @param rootNh Root node-handle of a ROS node
         * @return True if successful.
         */
        bool loadUrdf(ros::NodeHandle& rootNh);
    };

}  // namespace legged