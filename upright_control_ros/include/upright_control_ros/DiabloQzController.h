//
// Created by lsy on 24-3-6.
//

#pragma once

#include "MobileManipulatorControlCmd.h"
#include "definitions.h"
#include "pinocchio_interface/pinocchio_interface.h"
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>

#include "generally_hw/hardware_interface/HybridJointInterface.h"
#include "upright_control_ros/MobileManipulatorVisualization.h"
#include "upright_control_ros/synchronized_module/RosReferenceManager.h"
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <qz_hw/hybrid_force.h>
#include <ros/package.h>
#include <upright_common/ori_tool.h>
#include <upright_common/tf_rt_broadcaster.h>
#include <upright_control/MobileManipulatorInterface.h>

namespace ddt {
class DiabloQzController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::HybridJointInterface,
          hardware_interface::VelocityJointInterface> {
  enum ControllerState {
    NORMAL,  // Only try to achieve EE to reach a specified point in space
    UPRIGHT, // On the basis of normal, increase the importance of keeping the
             // objects on EE upright
    UPRIGHT_AVOID, // On the basis of upright, increase the function of avoid
                   // obstacle
    EE_SPACE_LOCK  // Try to lock the EE on a space point while move the  robot
                   // base
  };

public:
  DiabloQzController() = default;

  ~DiabloQzController() override;

  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  void starting(const ros::Time &time) override;

  void stopping(const ros::Time & /*time*/) override { mpcRunning_ = false; }

protected:
  virtual void updateStateEstimation(const ros::Time &time,
                                     const ros::Duration &period);

  void updateTfOdom(const ros::Time &time, const ros::Duration &period);

  void normal(const ros::Time &time, const ros::Duration &period);

  void upright(const ros::Time &time, const ros::Duration &period);

  void uprightAvoid(const ros::Time &time, const ros::Duration &period);

  void EESpaceLock(const ros::Time &time, const ros::Duration &period);

  void worldV2BaseV();

  virtual void setupMpc(ros::NodeHandle &nh);

  virtual void setupMrt();

  // Interface
  std::shared_ptr<upright::ControllerInterface> mobileManipulatorInterface_;
  std::vector<hardware_interface::JointHandle> jointHandles_;

  // State Estimation
  ocs2::SystemObservation currentObservation_;

  // Nonlinear MPC
  std::unique_ptr<ocs2::MultipleShootingMpc> mpc_;
  std::shared_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<ddt::MobileManipulatorDummyVisualization> visualizer_;
  ros::Publisher observationPublisher_;

private:
  int controlState_ = UPRIGHT;
  double init_x_, init_y_, init_z_;
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  ocs2::benchmark::RepeatedTimer mpcTimer_;
  //  Pinocchio interface
  pinocchio_interface::PinocchioInterface pinocchioInterface_;

  // Odom TF
  upright::TfRtBroadcaster tfRtBroadcaster_;
  geometry_msgs::TransformStamped odom2base_{};

  std::vector<hardware_interface::HybridJointHandle> hybridJointHandles_;
  std::vector<hardware_interface::JointHandle> velocityJointHandles_;
};
} // namespace ddt