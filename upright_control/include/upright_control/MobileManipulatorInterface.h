#pragma once

#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include <upright_control/constraint/ObstacleConstraint.h>
#include <upright_control/common/Types.h>
#include <upright_control/dynamics/MobileManipulatorInfo.h>
#include <upright_control/common/implementation/AccessHelperFunctionsImpl.h>
#include <upright_control/common/InterfaceSettings.h>

namespace upright {

    class ControllerInterface final : public ocs2::RobotInterface {
    public:
        explicit ControllerInterface(const std::string &taskFile, const std::string &libraryFolder,
                                     const std::string &urdfFileconst);

        const ocs2::vector_t &getInitialState() { return initial_state_; }

        ocs2::multiple_shooting::Settings &sqpSettings() { return settings_.sqp; }

        ocs2::mpc::Settings &mpcSettings() { return settings_.mpc; }

        const ocs2::OptimalControlProblem &getOptimalControlProblem() const override { return problem_; }

        std::shared_ptr<ocs2::ReferenceManagerInterface>
        getReferenceManagerPtr() const override { return reference_manager_ptr_; }

        const ocs2::RolloutBase &getRollout() const { return *rollout_ptr_; }

        const ocs2::Initializer &getInitializer() const override { return *initializer_ptr_; }

        std::unique_ptr<ocs2::MultipleShootingMpc> get_mpc();

        const ocs2::PinocchioInterface &getPinocchioInterface() const { return *pinocchio_interface_ptr; }

        const MobileManipulatorInfo &getManipulatorModelInfo() const { return *mobileManipulatorInfo_; }

        ocs2::PinocchioEndEffectorKinematicsCppAd &
        get_end_effector_kinematics() const { return *end_effector_kinematics_ptr_; }

    private:
        ControllerSettings settings_;
        ocs2::OptimalControlProblem problem_;
        std::unique_ptr<ocs2::RolloutBase> rollout_ptr_;
        VecXd initial_state_;
        std::unique_ptr<ocs2::Initializer> initializer_ptr_;
        std::unique_ptr<MobileManipulatorInfo> mobileManipulatorInfo_;
        std::shared_ptr<ocs2::ReferenceManager> reference_manager_ptr_;
        std::unique_ptr<ocs2::PinocchioInterface> pinocchio_interface_ptr;
        std::unique_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd> end_effector_kinematics_ptr_;

        std::unique_ptr<ocs2::StateInputCost> get_quadratic_state_input_cost();

        std::unique_ptr<ocs2::StateCost>
        getEndEffectorConstraint(const std::string &taskFile, const std::string &prefix);

        // Hard static obstacle avoidance constraint.
        std::unique_ptr<ocs2::StateConstraint> get_obstacle_constraint(
                ocs2::PinocchioInterface &pinocchio_interface,
                ocs2::PinocchioGeometryInterface &geom_interface,
                const ObstacleSettings &settings, const std::string &library_folder,
                bool recompile_libraries);

        // Soft static obstacle avoidance constraint.
        std::unique_ptr<ocs2::StateCost> get_soft_obstacle_constraint(
                ocs2::PinocchioInterface &pinocchio_interface,
                ocs2::PinocchioGeometryInterface &geom_interface,
                const ObstacleSettings &settings, const std::string &library_folder,
                bool recompile_libraries);

        // Hard state and input limits.
        std::unique_ptr<ocs2::StateInputConstraint> get_joint_state_input_limit_constraint();

        // Soft state and input limits
        std::unique_ptr<ocs2::StateInputCost> get_soft_joint_state_input_limit_constraint();

        // Hard balancing inequality constraints.
        std::unique_ptr<ocs2::StateInputConstraint> get_balancing_constraint(
                const ocs2::PinocchioEndEffectorKinematicsCppAd &
                end_effector_kinematics,
                bool recompileLibraries);

        // Soft version of the balancing constraints (i.e. formulated as a cost via
        // penalty functions).
        std::unique_ptr<ocs2::StateInputCost> get_soft_balancing_constraint(
                const ocs2::PinocchioEndEffectorKinematicsCppAd &
                end_effector_kinematics,
                bool recompileLibraries);

        std::unique_ptr<ocs2::StateInputConstraint> get_object_dynamics_constraint(
                const ocs2::PinocchioEndEffectorKinematicsCppAd &
                end_effector_kinematics,
                bool recompileLibraries);

        std::unique_ptr<ocs2::StateInputCost> get_soft_object_dynamics_constraint(
                const ocs2::PinocchioEndEffectorKinematicsCppAd &
                end_effector_kinematics,
                bool recompileLibraries);

        std::unique_ptr<ocs2::StateInputConstraint> get_contact_force_constraint(
                const ocs2::PinocchioEndEffectorKinematicsCppAd &
                end_effector_kinematics,
                bool recompileLibraries);

        std::unique_ptr<ocs2::StateInputCost> get_soft_contact_force_constraint(
                const ocs2::PinocchioEndEffectorKinematicsCppAd &
                end_effector_kinematics,
                bool recompileLibraries);
    };

}  // namespace upright
