//
// Created by lsy on 24-3-13.
//

#pragma once

#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

#include "upright_core/Nominal.h"
#include "upright_control/common/FactoryFunctions.h"

namespace upright {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    ocs2::PinocchioInterface createPinocchioInterface(const std::string &robotUrdfPath, const RobotBaseType &type) {
        switch (type) {
            case RobotBaseType::Nonholonomic: {
                // add X-yaw joint for the wheel-base
                pinocchio::JointModelComposite jointComposite(3);
                jointComposite.addJoint(pinocchio::JointModelPX());
                jointComposite.addJoint(pinocchio::JointModelPY());
                jointComposite.addJoint(pinocchio::JointModelRZ());
                return ocs2::getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
            }
            case RobotBaseType::Omnidirectional: {
                // add XY-yaw joint for the wheel-base
                pinocchio::JointModelComposite jointComposite(3);
                jointComposite.addJoint(pinocchio::JointModelPX());
                jointComposite.addJoint(pinocchio::JointModelPY());
                jointComposite.addJoint(pinocchio::JointModelRZ());
                return ocs2::getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
            }
            default:
                throw std::invalid_argument("Invalid manipulator model type provided.");
        }
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    ocs2::PinocchioInterface createPinocchioInterface(const std::string &robotUrdfPath, const RobotBaseType &type,
                                                      const std::vector<std::string> &jointNames) {
        using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

        // parse the URDF
        const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
        // remove extraneous joints from urdf
        ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
        for (joint_pair_t &jointPair: newModel->joints_) {
            if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) != jointNames.end()) {
                jointPair.second->type = urdf::Joint::FIXED;
            }
        }
        // resolve for the robot type
        switch (type) {
            case RobotBaseType::Nonholonomic: {
                // add X-yaw joint for the wheel-base
                pinocchio::JointModelComposite jointComposite(3);
                jointComposite.addJoint(pinocchio::JointModelPX());
                jointComposite.addJoint(pinocchio::JointModelPY());
                jointComposite.addJoint(pinocchio::JointModelRZ());
                return ocs2::getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
            }
            case RobotBaseType::Omnidirectional: {
                // add XY-yaw joint for the wheel-base
                pinocchio::JointModelComposite jointComposite(3);
                jointComposite.addJoint(pinocchio::JointModelPX());
                jointComposite.addJoint(pinocchio::JointModelPY());
                jointComposite.addJoint(pinocchio::JointModelRZ());
                return ocs2::getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
            }
            default:
                throw std::invalid_argument("Invalid manipulator model type provided.");
        }
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    RobotBaseType loadRobotBaseType(const std::string &configFilePath, const std::string &fieldName) {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(configFilePath, pt);
        const size_t type = pt.template get<size_t>(fieldName);
        return static_cast<RobotBaseType>(type);
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    MobileManipulatorInfo
    createMobileManipulatorInfo(const ocs2::PinocchioInterface &interface, const ControllerSettings &settings) {
        const auto &model = interface.getModel();

        MobileManipulatorInfo info;
        info.robotBaseType = settings.robot_base_type;
        // Use six-axis arm
        switch (info.robotBaseType) {
            case RobotBaseType::Nonholonomic: {
                info.OCPDim.robot.q = 9; // base: x,y,theta ; arm: q0,q1,q2,q3,q4,q5,q6
                info.OCPDim.robot.v = 8; // base: Vx, Vyaw ; arm: v0,v1,v2,v3,v4,v5,v6
                info.OCPDim.robot.x = 25; // 8 + 9 +9
                info.OCPDim.robot.u = 8; // // base: Jerk_x, Jerk_yaw ; arm: Jerk0,Jerk1,Jerk2,Jerk3,Jerk4,Jerk5,Jerk6
                break;
            }
            case RobotBaseType::Omnidirectional: {
                info.OCPDim.robot.q = 9; // base: x,y,theta ; arm: q0,q1,q2,q3,q4,q5,q6
                info.OCPDim.robot.v = 9; // base: Vx,Vy,Vyaw ; arm: v0,v1,v2,v3,v4,v5,v6
                info.OCPDim.robot.x = 27; // 9 + 9 + 9
                info.OCPDim.robot.u = 9; // // base: Jerk_x,Jerk_y,Jerk_yaw ; arm: Jerk0,Jerk1,Jerk2,Jerk3,Jerk4,Jerk5,Jerk6
                break;
            }
            default:
                throw std::invalid_argument("Invalid manipulator model type provided.");
                break;
        }

        // store frame names for using later.
        info.eeFrame = settings.end_effector_link_name;
        info.baseFrame = settings.base_link_name;
        // get name of arm joints.
        const auto &jointNames = model.names;
        info.dofNames = std::vector<std::string>(jointNames.end() - info.armDim, jointNames.end());

        return info;
    }
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    template<typename Scalar>
    std::map<std::string, BalancedObject<Scalar>> loadObjects(const YAML::Node &node) {
        // TODO: make it clear
        std::map<std::string, BalancedObject<Scalar>> objects;
        for (const auto &object: node) {
            std::string name = object.first.as<std::string>();
            std::string shape = object.second["shape"].as<std::string>();
            Scalar mass = object.second["mass"].as<Scalar>();
            Scalar com_height, r_tau, mu;
            std::vector<Scalar> yamlComOffset;
            Vec3<Scalar> comOffset, com;
            Mat3<Scalar> inertia;
            const YAML::Node &comOffsetNode = object.second["com_offset"];
            for (const auto &length: comOffsetNode)
                comOffset << length.as<Scalar>();
            if (shape == "cuboid" || shape == "wedge") {
                const YAML::Node &sideLengthsNode = object.second["side_lengths"];
                Vec3<Scalar> sideLengths;
                for (const auto &length: sideLengthsNode)
                    sideLengths << length.as<Scalar>();
                if (shape == "cuboid") {
                    inertia = cuboid_inertia_matrix(mass, sideLengths);
                    com << 0., sideLengths(1) / 2, 0.;
                    com_height = com(1);
                    r_tau = rectangle_r_tau(sideLengths(0), sideLengths(1));
                }

            } else if (shape == "cylinder") {
                Scalar radius = object.second["radius"].as<Scalar>();
                Scalar height = object.second["height"].as<Scalar>();
                inertia = cylinder_inertia_matrix(mass, radius, height);
                com << 0., height / 2, 0.;
                com_height = com(1);
                r_tau = circle_r_tau(radius);
            }
            RigidBody<Scalar> rigidBody(mass, inertia, com);
            BalancedObject<Scalar> balancedObject(rigidBody, com_height, r_tau, mu);
            objects.push_back(std::make_pair(name, balancedObject));
        }
        return objects;
    }




/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    template<typename Scalar>
    std::vector<ContactPoint<Scalar>> loadContact(const YAML::Node &node) {
//    contacts:
//    -
//        first:
//        second:
//        mu:
//        mu_margin:
//        support_area_inset:
        std::vector<ContactPoint<Scalar>> contactPoints;
        for (const auto &contactPoint: node) {
            ContactPoint<Scalar> ContactPoint;
            ContactPoint.object1_name = contactPoint["first"].as<std::string>();
            ContactPoint.object2_name = contactPoint["second"].as<std::string>();
            ContactPoint.mu = contactPoint["mu"].as<Scalar>();
            ContactPoint.mu_margin = contactPoint["mu_margin"].as<Scalar>();
            ContactPoint.mu = contactPoint["mu"].as<Scalar>();
            contactPoints.push_back(ContactPoint);
        }
        return contactPoints;
    }




/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
    ControllerSettings
    creatControllerSetting(const std::string &taskFile, const std::string &libraryFolder, const std::string &urdfFile) {
        // check that task file exists
        boost::filesystem::path taskFilePath(taskFile);
        if (boost::filesystem::exists(taskFilePath)) {
            std::cerr << "[MobileManipulatorInterface] Loading task file: " << taskFilePath << std::endl;
        } else {
            throw std::invalid_argument("[MobileManipulatorInterface] Task file not found: " + taskFilePath.string());
        }
        // check that urdf file exists
        boost::filesystem::path urdfFilePath(urdfFile);
        if (boost::filesystem::exists(urdfFilePath)) {
            std::cerr << "[MobileManipulatorInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
        } else {
            throw std::invalid_argument("[MobileManipulatorInterface] URDF file not found: " + urdfFilePath.string());
        }
        // create library folder if it does not exist
        boost::filesystem::path libraryFolderPath(libraryFolder);
        boost::filesystem::create_directories(libraryFolderPath);
        std::cerr << "[MobileManipulatorInterface] Generated library path: " << libraryFolderPath << std::endl;

        // read the task file
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        // resolve meta-information about the model

        ControllerSettings settings;
//  OCS2 base setting
        settings.mpc = ocs2::mpc::loadSettings(taskFile, "mpc");
        settings.sqp = ocs2::multiple_shooting::loadSettings(taskFile, "sqp");
        settings.rollout = ocs2::rollout::loadSettings(taskFile, "rollout");
        settings.lib_folder = libraryFolder;
        settings.robot_urdf_path = urdfFile;

//  Robot setting
        ocs2::loadData::loadCppDataType(taskFile, "robot.dims.q", settings.dims.robot.q);
        ocs2::loadData::loadCppDataType(taskFile, "robot.dims.v", settings.dims.robot.v);
        ocs2::loadData::loadCppDataType(taskFile, "robot.dims.x", settings.dims.robot.x);
        ocs2::loadData::loadCppDataType(taskFile, "robot.dims.u", settings.dims.robot.u);
        ocs2::loadData::loadCppDataType(taskFile, "robot.tool_link_name", settings.end_effector_link_name);
        ocs2::loadData::loadCppDataType(taskFile, "robot.base_link_name", settings.base_link_name);
        ocs2::loadData::loadCppDataType(taskFile, "robot.arm_dim", settings.arm_dim);
        std::string baseTypeStr;
        ocs2::loadData::loadCppDataType(taskFile, "robot.base_type", baseTypeStr);
        settings.robot_base_type = robot_base_type_from_string(baseTypeStr);
        ocs2::matrix_t initState(settings.dims.robot.x, 1);
        ocs2::loadData::loadEigenMatrix(taskFile, "robot.x0", initState);
        settings.initial_state = initState;

//    settings.gravity
        ocs2::matrix_t gravity_vec(3, 1);
        ocs2::loadData::loadEigenMatrix(taskFile, "gravity", gravity_vec);
        settings.gravity = gravity_vec;

        ocs2::loadData::loadCppDataType(taskFile, "mobile_manipulator_interface.recompileLibraries",
                                        settings.recompile_libraries);
        ocs2::loadData::loadCppDataType(taskFile, "mobile_manipulator_interface.debug", settings.debug);

// Weights Matrix
        ocs2::matrix_t inputMatrix(settings.dims.robot.u, settings.dims.robot.u);
        ocs2::matrix_t stateMatrix(settings.dims.robot.x, settings.dims.robot.x);
        ocs2::matrix_t EEMatrix(settings.arm_dim, settings.arm_dim);
        ocs2::loadData::loadEigenMatrix(taskFile, "Cost.input", inputMatrix);
        ocs2::loadData::loadEigenMatrix(taskFile, "Cost.state", stateMatrix);
        ocs2::loadData::loadEigenMatrix(taskFile, "Cost.end_effector", EEMatrix);
        settings.input_weight = inputMatrix;
        settings.state_weight = stateMatrix;
        settings.end_effector_weight = EEMatrix;

// Constraint
// EE box constraint
        ocs2::matrix_t xyzLower(3, 1), xyzUpper(3, 1);
        xyzLower.setZero();
        xyzUpper.setZero();
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.endEffectorBoxConstraint.enabled",
                                        settings.end_effector_box_constraint_enabled);
        ocs2::loadData::loadEigenMatrix(taskFile, "Constraint.endEffectorBoxConstraint.xyz_lower", xyzLower);
        ocs2::loadData::loadEigenMatrix(taskFile, "Constraint.endEffectorBoxConstraint.xyz_upper", xyzUpper);
        settings.xyz_lower = xyzLower;
        settings.xyz_upper = xyzUpper;
// Limit constraint
        std::string limit_constraint_type;
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.limits.constraint_type", limit_constraint_type);
        settings.limit_constraint_type = constraint_type_from_string(limit_constraint_type);
        ocs2::matrix_t inputLower(8, 1), inputUpper(8, 1), stateLower(25, 1), stateUpper(25, 1);

        ocs2::loadData::loadEigenMatrix(taskFile, "Constraint.limits.input_lower", inputLower);
        ocs2::loadData::loadEigenMatrix(taskFile, "Constraint.limits.input_upper", inputUpper);
        ocs2::loadData::loadEigenMatrix(taskFile, "Constraint.limits.state_lower", stateLower);
        ocs2::loadData::loadEigenMatrix(taskFile, "Constraint.limits.state_upper", stateUpper);
        settings.input_limit_lower = inputLower;
        settings.input_limit_upper = inputUpper;
        settings.state_limit_lower = stateLower;
        settings.state_limit_upper = stateUpper;
        std::cout << "Finish Get Basic Info" << std::endl;

//  TODO:  Add The last config afterbasic function is done
//  Inertial alignment
        ocs2::loadData::loadCppDataType(taskFile, "inertial_alignment.cost_enabled",
                                        settings.inertial_alignment_settings.cost_enabled);
        ocs2::loadData::loadCppDataType(taskFile, "inertial_alignment.constraint_enabled",
                                        settings.inertial_alignment_settings.constraint_enabled);
        ocs2::loadData::loadCppDataType(taskFile, "inertial_alignment.use_angular_acceleration",
                                        settings.inertial_alignment_settings.use_angular_acceleration);
        ocs2::loadData::loadCppDataType(taskFile, "inertial_alignment.cost_weight",
                                        settings.inertial_alignment_settings.cost_weight);
        ocs2::matrix_t contact_plane_normal(3, 1), com(3, 1), contact_plane_span(2, 3);
        ocs2::loadData::loadEigenMatrix(taskFile, "inertial_alignment.contact_plane_normal", contact_plane_normal);
        ocs2::loadData::loadEigenMatrix(taskFile, "inertial_alignment.contact_plane_span", contact_plane_span);
        ocs2::loadData::loadEigenMatrix(taskFile, "inertial_alignment.com", com);
        settings.inertial_alignment_settings.contact_plane_normal = contact_plane_normal;
        settings.inertial_alignment_settings.contact_plane_span = contact_plane_span;
        settings.inertial_alignment_settings.com = com;
        std::cout << "Finish Get Inertial alignment Info" << std::endl;

//    Balance
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.enabled", settings.balancing_settings.enabled);
        std::string constraintTypeStr;
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.constraint_type", constraintTypeStr);
        settings.balancing_settings.constraint_type = constraint_type_from_string(constraintTypeStr);
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.arrangement",
                                        settings.balancing_settings.arrangement_name);
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.use_force_constraints",
                                        settings.balancing_settings.use_force_constraints);
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.enable_normal_constraint",
                                        settings.balancing_settings.constraints_enabled.normal);
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.enable_friction_constraint",
                                        settings.balancing_settings.constraints_enabled.friction);
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.enable_zmp_constraint",
                                        settings.balancing_settings.constraints_enabled.zmp);
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.force_weight",
                                        settings.balancing_settings.force_weight);
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.mu", settings.balancing_settings.mu);
        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.delta", settings.balancing_settings.delta);

        ocs2::loadData::loadCppDataType(taskFile, "Constraint.balancing.arrangement_path",
                                        settings.balancing_settings.arrangement_path);

//    YAML::Node config = YAML::LoadFile(settings.balancing_settings.arrangement_path);
//    const YAML::Node& objectsNode = config["objects"];
//    for (const auto& object : objectsNode){
//        std::string name = object["name"].as<std::string>();
//        std::string shape = object.second["shape"].as<std::string>();
//        double mass = object.second["mass"].as<double>();
//        std::vector<double> comOffset,position;
//        const YAML::Node& comOffsetNode = object.second["com_offset"];
//        const YAML::Node& positionNode = object.second["position"];
//        for (const auto& length : comOffsetNode)
//            comOffset.push_back(length.as<double>());
//        for (const auto& pos : positionNode)
//            position.push_back(pos.as<double>());
//        if (shape == "cuboid")
//        {
//            const YAML::Node& sideLengthsNode = object.second["side_lengths"];
//            std::vector<double> sideLengths;
//            for (const auto& length : sideLengthsNode)
//                sideLengths.push_back(length.as<double>());
//        } else if (shape == "cylinder")
//        {
//            double radius = object.second["radius"].as<double>();
//            double height = object.second["height"].as<double>();
//        }
//    }

        std::cout << "Finish Get Balance Info" << std::endl;

//    EstimationSettings estimationSettings;
//    TrackingSettings trackingSettings;
//    ocs2::loadData::loadCppDataType(taskFile, "estimation.robot_init_variance", estimationSettings.robot_init_variance);
//    ocs2::loadData::loadCppDataType(taskFile, "estimation.robot_process_variance", estimationSettings.robot_process_variance);
//    ocs2::loadData::loadCppDataType(taskFile, "estimation.robot_measurement_variance", estimationSettings.robot_measurement_variance);
//    settings.estimation = estimationSettings;
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.rate", trackingSettings.rate);
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.min_policy_update_time", trackingSettings.min_policy_update_time);
//
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.kp", trackingSettings.kp);
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.kv", trackingSettings.kv);
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.ka", trackingSettings.ka);
//
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.enforce_state_limits", trackingSettings.enforce_state_limits);
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.enforce_input_limits", trackingSettings.enforce_input_limits);
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.enforce_ee_position_limits", trackingSettings.enforce_ee_position_limits);
//
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.state_violation_margin", trackingSettings.state_violation_margin);
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.input_violation_margin", trackingSettings.input_violation_margin);
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.ee_position_violation_margin", trackingSettings.ee_position_violation_margin);
//    ocs2::loadData::loadCppDataType(taskFile, "tracking.use_projective", trackingSettings.use_projectile);
//    settings.tracking = trackingSettings;

        return settings;
    }


}
