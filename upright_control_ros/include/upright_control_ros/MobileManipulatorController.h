//
// Created by lsy on 24-3-6.
//

#pragma once

#include "definitions.h"

#include <controller_interface/multi_interface_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_effort_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_core/Types.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>

namespace ddt{
    class MobileManipulatorController
            : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>{
        enum ControllerState { NORMAL, UPRIGHT, UPRIGHT_AVOID, EE_SPACE_LOCK};

    public:
        MobileManipulatorController() = default;
        ~MobileManipulatorController() override;

        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
        void update(const ros::Time& time, const ros::Duration& period) override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

    protected:
        virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
        void normal(const ros::Time& time, const ros::Duration& period);
        void upright(const ros::Time& time, const ros::Duration& period);
        void uprightAvoid(const ros::Time& time, const ros::Duration& period);
        void EESpaceLock(const ros::Time& time, const ros::Duration& period);

        virtual void setupMpc(ros::NodeHandle& nh);
        virtual void setupMrt();

        // Interface
        std::shared_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> UR5Interface_;
        std::vector<hardware_interface::JointHandle> jointHandles_;
//    hardware_interface::ImuSensorHandle imuSensorHandle_;

        // State Estimation
        ocs2::SystemObservation currentObservation_;

        // Nonlinear MPC
        std::shared_ptr<ocs2::MPC_BASE> mpc_;
        std::shared_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface_;
//    LeggedBalanceParameters params_;

        // Visualization
//    std::shared_ptr<ocs2::mobile_manipulator::MobileManipulatorDummyVisualization> visualizer_;
        ros::Publisher observationPublisher_;

    private:
        int controlState_ = NORMAL;
        std::thread mpcThread_;
        std::atomic_bool controllerRunning_{}, mpcRunning_{};
        ocs2::benchmark::RepeatedTimer mpcTimer_;
    };
}