// NOTE: pinocchio needs to be included before other things to prevent the
// compiler fussing
#include "upright_control/MobileManipulatorInterface.h"

#include <hpp/fcl/shape/geometric_shapes.h>
//#include <ocs2_core/constraint/BoundConstraint.h>
#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_core/penalties/penalties/DoubleSidedPenalty.h>
#include <ocs2_core/penalties/penalties/QuadraticPenalty.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

#include <upright_control/common/InterfaceSettings.h>
#include <upright_control/constraint/ConstraintType.h>
#include <upright_control/constraint/BoundedBalancingConstraints.h>
#include <upright_control/constraint/EndEffectorBoxConstraint.h>
#include <upright_control/constraint/JointStateInputLimits.h>
#include <upright_control/constraint/ObstacleConstraint.h>
#include <upright_control/constraint/EndEffectorConstraint.h>
#include <upright_control/constraint/ProjectilePathConstraint.h>
#include <upright_control/constraint/ProjectilePlaneConstraint.h>
#include <upright_control/constraint/StateToStateInputConstraint.h>

#include <upright_control/cost/EndEffectorCost.h>
#include <upright_control/cost/QuadraticJointStateInputCost.h>

#include <upright_control/dynamics/BaseType.h>
#include <upright_control/dynamics/SystemDynamics.h>
#include <upright_control/dynamics/SystemPinocchioMapping.h>
#include <upright_control/dynamics/MobileManipulatorInfo.h>

#include <upright_control/InertialAlignment.h>
#include <upright_control/common/implementation/FactoryFunctionsImpl.h>
#include <upright_control/common/PrasingBlancingConfig.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace upright {

    std::tuple<pinocchio::Model, pinocchio::GeometryModel>
    build_dynamic_obstacle_model(const std::vector<DynamicObstacle> &obstacles) {
        pinocchio::Model model;
        model.name = "dynamic_obstacles";
        pinocchio::GeometryModel geom_model;

        for (const auto &obstacle: obstacles) {
            // free-floating joint
            std::string joint_name = obstacle.name + "_joint";
            auto joint_placement = pinocchio::SE3::Identity();
            auto joint_id = model.addJoint(0, pinocchio::JointModelTranslation(),
                                           joint_placement, joint_name);

            // body
            ocs2::scalar_t mass = 1.0;
            auto body_placement = pinocchio::SE3::Identity();
            auto inertia = pinocchio::Inertia::FromSphere(mass, obstacle.radius);
            model.appendBodyToJoint(joint_id, inertia, body_placement);

            // collision model
            pinocchio::GeometryObject::CollisionGeometryPtr shape_ptr(
                    new hpp::fcl::Sphere(obstacle.radius));
            pinocchio::GeometryObject geom_obj(obstacle.name, joint_id, shape_ptr,
                                               body_placement);
            geom_model.addGeometryObject(geom_obj);
        }

        return {model, geom_model};
    }

    pinocchio::GeometryModel build_geometry_model(const std::string &urdf_path) {
        ocs2::PinocchioInterface::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);
        pinocchio::GeometryModel geom_model;
        pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::COLLISION,
                                   geom_model);
        return geom_model;
    }

    void add_ground_plane(const ocs2::PinocchioInterface::Model &model,
                          pinocchio::GeometryModel &geom_model) {
        auto ground_placement = pinocchio::SE3::Identity();
        pinocchio::GeometryObject::CollisionGeometryPtr ground_shape_ptr(
                new hpp::fcl::Halfspace(Vec3d::UnitZ(), 0));
        pinocchio::GeometryObject ground_obj("ground", model.frames[0].parent,
                                             ground_shape_ptr, ground_placement);
        geom_model.addGeometryObject(ground_obj);
    }

    ControllerInterface::ControllerInterface(const std::string &taskFile, const std::string &libraryFolder,
                                             const std::string &urdfFile) {
        settings_ = creatControllerSetting(taskFile, libraryFolder, urdfFile);
        // Pinocchio interface
        std::cerr << "Robot URDF: " << settings_.robot_urdf_path << std::endl;
        pinocchio_interface_ptr.reset(
                new ocs2::PinocchioInterface(createPinocchioInterface(
                        settings_.robot_urdf_path, settings_.robot_base_type)));

        mobileManipulatorInfo_.reset(
                new MobileManipulatorInfo(createMobileManipulatorInfo(getPinocchioInterface(), settings_)));

        const bool recompile_libraries = settings_.recompile_libraries;
        settings_.sqp.integratorType = ocs2::SensitivityIntegratorType::RK4;

        // Dynamics
        // NOTE: we don't have any branches here because every system we use
        // currently is an integrator
        std::unique_ptr<ocs2::SystemDynamicsBase> dynamics_ptr(
                new SystemDynamics<NonholonomicDynamics<ocs2::ad_scalar_t>>(
                        "system_dynamics", settings_.dims, settings_.lib_folder,
                        recompile_libraries, true));

        // Rollout
        rollout_ptr_.reset(
                new ocs2::TimeTriggeredRollout(*dynamics_ptr, settings_.rollout));

        // Reference manager
        reference_manager_ptr_.reset(new ocs2::ReferenceManager);

        // Optimal control problem
        problem_.dynamicsPtr = std::move(dynamics_ptr);

        // Regularization cost
        problem_.costPtr->add("state_input_cost", get_quadratic_state_input_cost());

        // Build the end effector kinematics
        NonholonomicPinocchioMapping<ocs2::ad_scalar_t> mapping(settings_.dims.robot);

        /* Constraints */
        if (settings_.limit_constraint_type == ConstraintType::Soft) {
            problem_.softConstraintPtr->add(
                    "joint_state_input_limits",
                    get_soft_joint_state_input_limit_constraint());
            std::cerr << "Soft state and input limits are enabled." << std::endl;
        } else {
            // std::unique_ptr<ocs2::StateInputConstraint>
            //     joint_state_input_constraint(new JointStateInputConstraint(
            //         settings_.dims, settings_.state_limit_lower,
            //         settings_.state_limit_upper, settings_.input_limit_lower,
            //         settings_.input_limit_upper));
            // problem_.inequalityConstraintPtr->add(
            //     "joint_state_input_limits",
            //     std::move(joint_state_input_constraint));

            problem_.boundConstraintPtr->setZero(settings_.dims.x(),
                                                 settings_.dims.u());
            problem_.boundConstraintPtr->state_lb_.head(settings_.dims.robot.x) =
                    settings_.state_limit_lower;
            problem_.boundConstraintPtr->state_ub_.head(settings_.dims.robot.x) =
                    settings_.state_limit_upper;
            problem_.boundConstraintPtr->setStateIndices(0, settings_.dims.robot.x);

            problem_.boundConstraintPtr->input_lb_.head(settings_.dims.robot.u) =
                    settings_.input_limit_lower;
            problem_.boundConstraintPtr->input_ub_.head(settings_.dims.robot.u) =
                    settings_.input_limit_upper;
            problem_.boundConstraintPtr->setInputIndices(0, settings_.dims.robot.u);

            std::cerr << "Hard state and input limits are enabled." << std::endl;
        }

//        // Collision avoidance
//        if (settings_.obstacle_settings.enabled) {
//            ocs2::PinocchioGeometryInterface geom_interface(
//                    *pinocchio_interface_ptr);
//
//            // Add obstacle collision objects to the geometry model, so we can check
//            // them against the robot.
//            std::string obs_urdf_path =
//                    settings_.obstacle_settings.obstacle_urdf_path;
//            if (obs_urdf_path.size() > 0) {
//                std::cout << "Obstacle URDF: " << obs_urdf_path << std::endl;
//                pinocchio::GeometryModel obs_geom_model =
//                        build_geometry_model(obs_urdf_path);
//                geom_interface.addGeometryObjects(obs_geom_model);
//            }
//
//            const auto& model = pinocchio_interface_ptr->getModel();
//            auto& geom_model = geom_interface.getGeometryModel();
//            add_ground_plane(model, geom_model);
//
//            // Add dynamic obstacles.
//            if (settings_.obstacle_settings.dynamic_obstacles.size() > 0) {
//                ocs2::PinocchioInterface::Model dyn_obs_model, new_model;
//                pinocchio::GeometryModel dyn_obs_geom_model, new_geom_model;
//
//                std::tie(dyn_obs_model, dyn_obs_geom_model) =
//                        build_dynamic_obstacle_model(
//                                settings_.obstacle_settings.dynamic_obstacles);
//
//                // Update models
//                pinocchio::appendModel(
//                        model, dyn_obs_model, geom_model, dyn_obs_geom_model, 0,
//                        pinocchio::SE3::Identity(), new_model, new_geom_model);
//
//                pinocchio_interface_ptr.reset(
//                        new ocs2::PinocchioInterface(new_model));
//                geom_interface = ocs2::PinocchioGeometryInterface(new_geom_model);
//            }
//
//            std::cout << *pinocchio_interface_ptr << std::endl;
//
//            if (settings_.obstacle_settings.constraint_type ==
//                ConstraintType::Soft) {
//                problem_.stateSoftConstraintPtr->add(
//                        "obstacle_avoidance",
//                        get_soft_obstacle_constraint(
//                                *pinocchio_interface_ptr, geom_interface,
//                                settings_.obstacle_settings, settings_.lib_folder,
//                                recompile_libraries));
//                std::cerr << "Soft obstacle avoidance constraints are enabled."
//                          << std::endl;
//            } else {
//                // Get the usual state constraint
//                std::unique_ptr<ocs2::StateConstraint> obstacle_constraint =
//                        get_obstacle_constraint(
//                                *pinocchio_interface_ptr, geom_interface,
//                                settings_.obstacle_settings, settings_.lib_folder,
//                                recompile_libraries);
//
//                // Map it to a state-input constraint so it works with the current
//                // implementation of the hard inequality constraints
//                problem_.inequalityConstraintPtr->add(
//                        "obstacle_avoidance",
//                        std::unique_ptr<ocs2::StateInputConstraint>(
//                                new StateToStateInputConstraint(*obstacle_constraint)));
//                std::cerr << "Hard obstacle avoidance constraints are enabled."
//                          << std::endl;
//            }
//        } else {
//            std::cerr << "Obstacle avoidance is disabled." << std::endl;
//        }
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::cout << settings_.dims.x() << std::endl;
        ocs2::PinocchioEndEffectorKinematicsCppAd end_effector_kinematics(
                *pinocchio_interface_ptr, mapping, {settings_.end_effector_link_name},
                settings_.dims.x(), settings_.dims.u(), "end_effector_kinematics",
                settings_.lib_folder, true, false);
        // Store for possible use by other callers.
        end_effector_kinematics_ptr_.reset(end_effector_kinematics.clone());
        // End effector pose cost
        std::unique_ptr<ocs2::StateCost> end_effector_cost(new EndEffectorCost(
                settings_.end_effector_weight, end_effector_kinematics));
        problem_.stateCostPtr->add("end_effector_cost",
                                   std::move(end_effector_cost));
        // Alternative auto-diff version with full Hessian
        // std::unique_ptr<ocs2::StateCost> end_effector_cost(new
        // EndEffectorCostCppAd(
        //     settings_.end_effector_weight, end_effector_kinematics,
        //     settings_.dims, recompile_libraries));
        // problem_.stateCostPtr->add("end_effector_cost",
        //                            std::move(end_effector_cost));
        // End effector position box constraint


        problem_.stateSoftConstraintPtr->add("endEffector",
                                             getEndEffectorConstraint(taskFile, "Constraint.endEffector"));
        problem_.finalSoftConstraintPtr->add("finalEndEffector",
                                             getEndEffectorConstraint(taskFile, "Constraint.finalEndEffector"));

        if (settings_.end_effector_box_constraint_enabled) {
            std::cout << "End effector box constraint is enabled." << std::endl;
            std::unique_ptr<ocs2::StateConstraint> end_effector_box_constraint(
                    new EndEffectorBoxConstraint(
                            settings_.xyz_lower, settings_.xyz_upper,
                            end_effector_kinematics, *reference_manager_ptr_));
            std::cout << "End effector box ." << std::endl;
            problem_.inequalityConstraintPtr->add(
                    "end_effector_box_constraint",
                    std::unique_ptr<ocs2::StateInputConstraint>(
                            new StateToStateInputConstraint(*end_effector_box_constraint)));
            std::cout << "End effector box ." << std::endl;
        } else {
            std::cout << "End effector box constraint is disabled." << std::endl;
        }



        // Experimental: projectile avoidance constraint
//        if (settings_.projectile_path_constraint_enabled) {
//            // TODO: hardcoded link name
//            ocs2::PinocchioEndEffectorKinematicsCppAd
//                    end_effector_collision_kinematics(
//                    *pinocchio_interface_ptr, mapping,
//                    settings_.projectile_path_collision_links, settings_.dims.x(),
//                    settings_.dims.u(), "end_effector_collision_kinematics",
//                    settings_.lib_folder, recompile_libraries, false);
//
//            std::unique_ptr<ocs2::StateConstraint> projectile_constraint(
//                    new ProjectilePathConstraint(end_effector_collision_kinematics,
//                                                 *reference_manager_ptr_,
//                                                 settings_.projectile_path_distances,
//                                                 settings_.projectile_path_scale));
//            // new ProjectilePlaneConstraint(end_effector_collision_kinematics,
//            //                              *reference_manager_ptr_,
//            //                              settings_.projectile_path_distance));
//            problem_.inequalityConstraintPtr->add(
//                    "projectile_constraint",
//                    std::unique_ptr<ocs2::StateInputConstraint>(
//                            new StateToStateInputConstraint(*projectile_constraint)));
//        }
//
//        // Inertial alignment
        if (settings_.inertial_alignment_settings.cost_enabled) {
            std::unique_ptr<ocs2::StateInputCost> inertial_alignment_cost(
                    new InertialAlignmentCostGaussNewton(
                            end_effector_kinematics, settings_.inertial_alignment_settings,
                            settings_.gravity, settings_.dims, true));
            problem_.costPtr->add("inertial_alignment_cost",
                                  std::move(inertial_alignment_cost));
            std::cout << "Inertial alignment cost enabled." << std::endl;
        }
        if (settings_.inertial_alignment_settings.constraint_enabled) {
            std::unique_ptr<ocs2::StateInputConstraint>
                    inertial_alignment_constraint(new InertialAlignmentConstraint(
                    end_effector_kinematics, settings_.inertial_alignment_settings,
                    settings_.gravity, settings_.dims, recompile_libraries));
            problem_.inequalityConstraintPtr->add(
                    "inertial_alignment_constraint",
                    std::move(inertial_alignment_constraint));
            std::cout << "Inertial alignment constraint enabled." << std::endl;
        }

        // TODO we're getting too nested here
        if (settings_.balancing_settings.enabled) {
            if (settings_.balancing_settings.use_force_constraints) {
                problem_.equalityConstraintPtr->add(
                        "object_dynamics",
                        get_object_dynamics_constraint(end_effector_kinematics,
                                                       recompile_libraries));

                // Inequalities for the friction cones
                // NOTE: the hard inequality constraints appear to work much better
                // (avoid phantom gradients and such)
                if (settings_.balancing_settings.constraint_type ==
                    ConstraintType::Soft) {
                    std::cerr
                            << "Soft contact force-based balancing constraints enabled."
                            << std::endl;
                    problem_.softConstraintPtr->add(
                            "contact_forces",
                            get_soft_contact_force_constraint(end_effector_kinematics,
                                                              recompile_libraries));
                } else {
                    std::cerr
                            << "Hard contact force-based balancing constraints enabled."
                            << std::endl;
                    const bool frictionless = (settings_.dims.nf == 1);
                    if (frictionless) {
                        // lower bounds are already zero, make the upper ones
                        // arbitrary high values
                        problem_.boundConstraintPtr->input_ub_
                                .tail(settings_.dims.f())
                                .setConstant(1e6);
                        // indicate that all inputs are now box constrained (real
                        // inputs and contact forces)
                        problem_.boundConstraintPtr->setInputIndices(
                                0, settings_.dims.u());

                        std::cout
                                << problem_.boundConstraintPtr->input_lb_.transpose()
                                << std::endl;
                        std::cout
                                << problem_.boundConstraintPtr->input_ub_.transpose()
                                << std::endl;
                    } else {
                        problem_.inequalityConstraintPtr->add(
                                "contact_forces",
                                get_contact_force_constraint(end_effector_kinematics,
                                                             recompile_libraries));
                    }
                }
            } else {
                if (settings_.balancing_settings.constraint_type ==
                    ConstraintType::Soft) {
                    std::cerr << "Soft ZMP/limit surface-based balancing "
                                 "constraints enabled."
                              << std::endl;

                    problem_.softConstraintPtr->add(
                            "balancing",
                            get_soft_balancing_constraint(end_effector_kinematics,
                                                          recompile_libraries));

                } else {
                    std::cerr << "Hard ZMP/limit surface-based balancing "
                                 "constraints enabled."
                              << std::endl;
                    problem_.inequalityConstraintPtr->add(
                            "balancing",
                            get_balancing_constraint(end_effector_kinematics,
                                                     recompile_libraries));
                }
            }
        } else {
            std::cerr << "Balancing constraints disabled." << std::endl;
        }


        std::cout << " Balancing init finish " << std::endl;
        // Initialization state
        if (settings_.use_operating_points) {
            initializer_ptr_.reset(new ocs2::OperatingPoints(
                    settings_.operating_times, settings_.operating_states,
                    settings_.operating_inputs));
        } else {
            initializer_ptr_.reset(
                    new ocs2::DefaultInitializer(settings_.dims.u()));
        }

        // reference_manager_ptr_->setTargetTrajectories(settings_.target_trajectory);

        initial_state_ = settings_.initial_state;
        std::cerr << "Initial State:   " << initial_state_.transpose() << std::endl;
    }

    std::unique_ptr<ocs2::MultipleShootingMpc> ControllerInterface::get_mpc() {
        // if (settings_.solver_method == ControllerSettings::SolverMethod::DDP) {
        //     return std::unique_ptr<ocs2::MPC_BASE>(new ocs2::GaussNewtonDDP_MPC(
        //         mpcSettings_, ddpSettings_, *rollout_ptr_, problem_,
        //         *initializer_ptr_));
        // } else {
        return std::unique_ptr<ocs2::MultipleShootingMpc>(new ocs2::MultipleShootingMpc(
                settings_.mpc, settings_.sqp, problem_, getInitializer()));
        // }
    }

    std::unique_ptr<ocs2::StateInputCost>
    ControllerInterface::get_quadratic_state_input_cost() {
        // augment R with cost on the contact forces
//        MatXd input_weight =
//                settings_.balancing_settings.force_weight *
//                MatXd::Identity(settings_.dims.u(), settings_.dims.u());
//        input_weight.topLeftCorner(settings_.dims.robot.u, settings_.dims.robot.u) =
//                settings_.input_weight;
        MatXd input_weight = settings_.input_weight;

        // TODO do I need weight on obstacle dynamics?
//        MatXd state_weight = MatXd::Zero(settings_.dims.x(), settings_.dims.x());
//        state_weight.topLeftCorner(settings_.dims.robot.x, settings_.dims.robot.x) =
//                settings_.state_weight;
        MatXd state_weight = settings_.state_weight;;

        std::cout << "Q: " << state_weight << std::endl;
        std::cout << "R: " << input_weight << std::endl;

//        return std::unique_ptr<ocs2::StateInputCost>(
//                new QuadraticJointStateInputCost(state_weight, input_weight, settings_.xd));
        return std::unique_ptr<ocs2::StateInputCost>(
                new QuadraticJointStateInputCost(state_weight, input_weight));
    }

    std::unique_ptr<ocs2::StateCost>
    ControllerInterface::getEndEffectorConstraint(const std::string &taskFile, const std::string &prefix) {
        ocs2::scalar_t muPosition = 1.0;
        ocs2::scalar_t muOrientation = 1.0;
        const std::string name = "WRIST_2";

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::cerr << "\n #### " << prefix << " Settings: ";
        std::cerr << "\n #### =============================================================================\n";
        ocs2::loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
        ocs2::loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
        std::cerr << " #### =============================================================================\n";

        if (reference_manager_ptr_ == nullptr) {
            throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
        }

        std::unique_ptr<ocs2::StateConstraint> constraint;
        constraint.reset(new EndEffectorConstraint(get_end_effector_kinematics(), *reference_manager_ptr_));

        std::vector<std::unique_ptr<ocs2::PenaltyBase>> penaltyArray(6);
        std::generate_n(penaltyArray.begin(), 3, [&] { return std::make_unique<ocs2::QuadraticPenalty>(muPosition); });
        std::generate_n(penaltyArray.begin() + 3, 3,
                        [&] { return std::make_unique<ocs2::QuadraticPenalty>(muOrientation); });

        return std::make_unique<ocs2::StateSoftConstraint>(std::move(constraint), std::move(penaltyArray));
    }

    std::unique_ptr<ocs2::StateInputConstraint>
    ControllerInterface::get_balancing_constraint(
            const ocs2::PinocchioEndEffectorKinematicsCppAd &end_effector_kinematics,
            bool recompile_libraries) {
        return std::unique_ptr<ocs2::StateInputConstraint>(
                new NominalBalancingConstraints(
                        end_effector_kinematics, settings_.balancing_settings,
                        settings_.gravity, settings_.dims, recompile_libraries));
    }

    std::unique_ptr<ocs2::StateInputCost>
    ControllerInterface::get_soft_balancing_constraint(
            const ocs2::PinocchioEndEffectorKinematicsCppAd &end_effector_kinematics,
            bool recompile_libraries) {
        // compute the hard constraint
        std::unique_ptr<ocs2::StateInputConstraint> constraint =
                get_balancing_constraint(end_effector_kinematics, recompile_libraries);

        // make it soft via penalty function
        std::vector<std::unique_ptr<ocs2::PenaltyBase>> penaltyArray(
                constraint->getNumConstraints(0));
        for (int i = 0; i < constraint->getNumConstraints(0); i++) {
            penaltyArray[i].reset(new ocs2::RelaxedBarrierPenalty(
                    {settings_.balancing_settings.mu,
                     settings_.balancing_settings.delta}));
        }

        return std::unique_ptr<ocs2::StateInputCost>(
                new ocs2::StateInputSoftConstraint(std::move(constraint),
                                                   std::move(penaltyArray)));
    }

    std::unique_ptr<ocs2::StateInputConstraint>
    ControllerInterface::get_object_dynamics_constraint(
            const ocs2::PinocchioEndEffectorKinematicsCppAd &end_effector_kinematics,
            bool recompile_libraries) {
        return std::unique_ptr<ocs2::StateInputConstraint>(
                new ObjectDynamicsConstraints(
                        end_effector_kinematics, settings_.balancing_settings,
                        settings_.gravity, settings_.dims, recompile_libraries));
    }

    std::unique_ptr<ocs2::StateInputCost>
    ControllerInterface::get_soft_object_dynamics_constraint(
            const ocs2::PinocchioEndEffectorKinematicsCppAd &end_effector_kinematics,
            bool recompile_libraries) {
        // compute the hard constraint
        std::unique_ptr<ocs2::StateInputConstraint> constraint =
                get_object_dynamics_constraint(end_effector_kinematics,
                                               recompile_libraries);

        // make it soft via penalty function: since this is an equality constraint,
        // we use a quadratic penalty
        // TODO may need to increase the scaling
        std::vector<std::unique_ptr<ocs2::PenaltyBase>> penaltyArray(
                constraint->getNumConstraints(0));
        for (int i = 0; i < constraint->getNumConstraints(0); i++) {
            penaltyArray[i].reset(
                    new ocs2::QuadraticPenalty(settings_.balancing_settings.mu));
        }

        return std::unique_ptr<ocs2::StateInputCost>(
                new ocs2::StateInputSoftConstraint(std::move(constraint),
                                                   std::move(penaltyArray)));
    }

    std::unique_ptr<ocs2::StateInputConstraint>
    ControllerInterface::get_contact_force_constraint(
            const ocs2::PinocchioEndEffectorKinematicsCppAd &end_effector_kinematics,
            bool recompile_libraries) {
        return std::unique_ptr<ocs2::StateInputConstraint>(
                new ContactForceBalancingConstraints(
                        end_effector_kinematics, settings_.balancing_settings,
                        settings_.gravity, settings_.dims, recompile_libraries));
    }

    std::unique_ptr<ocs2::StateInputCost>
    ControllerInterface::get_soft_contact_force_constraint(
            const ocs2::PinocchioEndEffectorKinematicsCppAd &end_effector_kinematics,
            bool recompile_libraries) {
        // compute the hard constraint
        std::unique_ptr<ocs2::StateInputConstraint> constraint =
                get_contact_force_constraint(end_effector_kinematics,
                                             recompile_libraries);

        // make it soft via penalty function
        std::vector<std::unique_ptr<ocs2::PenaltyBase>> penaltyArray(
                constraint->getNumConstraints(0));
        for (int i = 0; i < constraint->getNumConstraints(0); i++) {
            penaltyArray[i].reset(new ocs2::SquaredHingePenalty(
                    {1, settings_.balancing_settings.delta}));  // TODO
        }

        return std::unique_ptr<ocs2::StateInputCost>(
                new ocs2::StateInputSoftConstraint(std::move(constraint),
                                                   std::move(penaltyArray)));
    }

    std::unique_ptr<ocs2::StateConstraint>
    ControllerInterface::get_obstacle_constraint(
            ocs2::PinocchioInterface &pinocchio_interface,
            ocs2::PinocchioGeometryInterface &geom_interface,
            const ObstacleSettings &settings, const std::string &library_folder,
            bool recompile_libraries) {
        std::cerr << "Number of extra collision spheres = "
                  << settings.extra_spheres.size() << std::endl;

        const auto &model = pinocchio_interface.getModel();
        std::vector<pinocchio::GeometryObject> extra_spheres;
        for (const auto &sphere: settings.extra_spheres) {
            // The collision sphere is specified relative to a link, but the
            // geometry interface works relative to joints. Thus we need to find
            // the parent joint and the sphere's transform w.r.t. it.
            // TODO if possible, it would be better to specify w.r.t. to a joint
            pinocchio::FrameIndex parent_frame_id =
                    model.getFrameId(sphere.parent_frame_name);
            pinocchio::Frame parent_frame = model.frames[parent_frame_id];

            pinocchio::JointIndex parent_joint_id = parent_frame.parent;
            Mat3d R = Mat3d::Identity();
            pinocchio::SE3 T_jf = parent_frame.placement;
            pinocchio::SE3 T_fs(R, sphere.offset);
            pinocchio::SE3 T_js = T_jf * T_fs;  // sphere relative to joint

            pinocchio::GeometryObject::CollisionGeometryPtr geom_ptr(
                    new hpp::fcl::Sphere(sphere.radius));
            extra_spheres.push_back(pinocchio::GeometryObject(
                    sphere.name, parent_joint_id, geom_ptr, T_js));
        }
        geom_interface.addGeometryObjects(extra_spheres);

        const auto &geom_model = geom_interface.getGeometryModel();
        for (int i = 0; i < geom_model.ngeoms; ++i) {
            std::cout << geom_model.geometryObjects[i].name << std::endl;
        }

        geom_interface.addCollisionPairsByName(settings.collision_link_pairs);

        const size_t nc = geom_interface.getNumCollisionPairs();
        std::cerr << "Testing for " << nc << " collision pairs." << std::endl;

        std::vector<hpp::fcl::DistanceResult> distances =
                geom_interface.computeDistances(pinocchio_interface);
        for (int i = 0; i < distances.size(); ++i) {
            std::cout << "dist = " << distances[i].min_distance << std::endl;
        }

        SystemPinocchioMapping<TripleIntegratorPinocchioMapping<ocs2::scalar_t>,
                ocs2::scalar_t>
                mapping(settings_.dims);

        return std::unique_ptr<ocs2::StateConstraint>(
                new ocs2::SelfCollisionConstraintCppAd(
                        pinocchio_interface, mapping, geom_interface,
                        settings.minimum_distance, "obstacle_avoidance", library_folder,
                        recompile_libraries, false));
    }

    std::unique_ptr<ocs2::StateCost>
    ControllerInterface::get_soft_obstacle_constraint(
            ocs2::PinocchioInterface &pinocchio_interface,
            ocs2::PinocchioGeometryInterface &geom_interface,
            const ObstacleSettings &settings, const std::string &library_folder,
            bool recompile_libraries) {
        std::unique_ptr<ocs2::StateConstraint> constraint =
                get_obstacle_constraint(pinocchio_interface, geom_interface, settings,
                                        library_folder, recompile_libraries);

        std::unique_ptr<ocs2::PenaltyBase> penalty(
                new ocs2::RelaxedBarrierPenalty({settings.mu, settings.delta}));

        return std::unique_ptr<ocs2::StateCost>(new ocs2::StateSoftConstraint(
                std::move(constraint), std::move(penalty)));
    }

    std::unique_ptr<ocs2::StateInputConstraint>
    ControllerInterface::get_joint_state_input_limit_constraint() {
        VecXd state_limit_lower = settings_.state_limit_lower;
        VecXd state_limit_upper = settings_.state_limit_upper;

        VecXd input_limit_lower = settings_.input_limit_lower;
        VecXd input_limit_upper = settings_.input_limit_upper;

        std::cout << "state limit lower: " << state_limit_lower.transpose()
                  << std::endl;
        std::cout << "state limit upper: " << state_limit_upper.transpose()
                  << std::endl;
        std::cout << "input limit lower: " << input_limit_lower.transpose()
                  << std::endl;
        std::cout << "input limit upper: " << input_limit_upper.transpose()
                  << std::endl;

        return std::unique_ptr<ocs2::StateInputConstraint>(
                new JointStateInputLimits(settings_.dims));
    }

    std::unique_ptr<ocs2::StateInputCost>
    ControllerInterface::get_soft_joint_state_input_limit_constraint() {
        std::unique_ptr<ocs2::StateInputConstraint> constraint =
                get_joint_state_input_limit_constraint();

        VecXd state_limit_lower = settings_.state_limit_lower;
        VecXd state_limit_upper = settings_.state_limit_upper;
        ocs2::scalar_t state_limit_mu = settings_.state_limit_mu;
        ocs2::scalar_t state_limit_delta = settings_.state_limit_delta;

        VecXd input_limit_lower = settings_.input_limit_lower;
        VecXd input_limit_upper = settings_.input_limit_upper;
        ocs2::scalar_t input_limit_mu = settings_.input_limit_mu;
        ocs2::scalar_t input_limit_delta = settings_.input_limit_delta;

        auto num_constraints = constraint->getNumConstraints(0);
        std::unique_ptr<ocs2::PenaltyBase> barrierFunction;
        std::vector<std::unique_ptr<ocs2::PenaltyBase>> penaltyArray(
                num_constraints);

        // State penalty
        for (int i = 0; i < settings_.dims.robot.x; i++) {
            // barrierFunction.reset(new ocs2::RelaxedBarrierPenalty(
            //     {state_limit_mu, state_limit_delta}));
            barrierFunction.reset(
                    new ocs2::SquaredHingePenalty({1, state_limit_delta}));
            penaltyArray[i].reset(new ocs2::DoubleSidedPenalty(
                    state_limit_lower(i), state_limit_upper(i),
                    std::move(barrierFunction)));
        }

        // Input penalty
        for (int i = 0; i < settings_.dims.robot.u; i++) {
            // barrierFunction.reset(new ocs2::RelaxedBarrierPenalty(
            //     {input_limit_mu, input_limit_delta}));
            barrierFunction.reset(
                    new ocs2::SquaredHingePenalty({1, input_limit_delta}));
            penaltyArray[settings_.dims.robot.x + i].reset(
                    new ocs2::DoubleSidedPenalty(input_limit_lower(i),
                                                 input_limit_upper(i),
                                                 std::move(barrierFunction)));
        }

        return std::unique_ptr<ocs2::StateInputCost>(
                new ocs2::StateInputSoftConstraint(std::move(constraint),
                                                   std::move(penaltyArray)));
    }

}  // namespace upright
