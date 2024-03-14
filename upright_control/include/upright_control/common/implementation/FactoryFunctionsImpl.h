//
// Created by lsy on 24-3-13.
//

#pragma once

#include "upright_control/common/FactoryFunctions.h"
namespace upright{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const RobotBaseType& type){
    switch (type) {
        case RobotBaseType::Nonholonomic: {
            // add X-yaw joint for the wheel-base
            pinocchio::JointModelComposite jointComposite(3);
            jointComposite.addJoint(pinocchio::JointModelPX());
            jointComposite.addJoint(pinocchio::JointModelRZ());
            return ocs2::getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
            return ocs2::getPinocchioInterfaceFromUrdfFile(robotUrdfPath);
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
ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const RobotBaseType& type,
                                                  const std::vector<std::string>& jointNames){
    using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

    // parse the URDF
    const auto urdfTree = ::urdf::parseURDFFile(robotUrdfPath);
    // remove extraneous joints from urdf
    ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
    for (joint_pair_t& jointPair : newModel->joints_) {
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
            jointComposite.addJoint(pinocchio::JointModelRZ());
            return ocs2::getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);
            return ocs2::getPinocchioInterfaceFromUrdfFile(robotUrdfPath);
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
RobotBaseType loadRobotBaseType(const std::string& configFilePath, const std::string& fieldName)
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(configFilePath, pt);
    const size_t type = pt.template get<size_t>(fieldName);
    return static_cast<RobotBaseType>(type);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorInfo createMobileManipulatorInfo(const ocs2::PinocchioInterface& interface, const RobotBaseType& type,
                                                  const std::string& baseFrame, const std::string& eeFrame) {
    const auto& model = interface.getModel();

    MobileManipulatorInfo info;
    info.robotBaseType = type;
    // Use six-axis arm
    switch (type) {
        case RobotBaseType::Nonholonomic: {
            info.OCPDim.robot.q = 9; // base: x,y,theta ; arm: q0,q1,q2,q3,q4,q5,q6
            info.OCPDim.robot.v = 8; // base: Vx, Vyaw ; arm: v0,v1,v2,v3,v4,v5,v6
            info.OCPDim.robot.x = info.OCPDim.robot.q + info.OCPDim.robot.v + info.OCPDim.robot.v; // 8 + 9 +9
            info.OCPDim.robot.u = 8; // // base: Jerk_x, Jerk_yaw ; arm: Jerk0,Jerk1,Jerk2,Jerk3,Jerk4,Jerk5,Jerk6
            break;
        }
        case RobotBaseType::Omnidirectional: {
            info.OCPDim.robot.q = 9; // base: x,y,theta ; arm: q0,q1,q2,q3,q4,q5,q6
            info.OCPDim.robot.v = 9; // base: Vx,Vy,Vyaw ; arm: v0,v1,v2,v3,v4,v5,v6
            info.OCPDim.robot.x = info.OCPDim.robot.q + info.OCPDim.robot.v; info.OCPDim.robot.v; // 9 + 9 + 9
            info.OCPDim.robot.u = 9; // // base: Jerk_x,Jerk_y,Jerk_yaw ; arm: Jerk0,Jerk1,Jerk2,Jerk3,Jerk4,Jerk5,Jerk6
            break;
        }
        default:
            throw std::invalid_argument("Invalid manipulator model type provided.");
            break;
    }

    // store frame names for using later.
    info.eeFrame = eeFrame;
    info.baseFrame = baseFrame;
    // get name of arm joints.
    const auto& jointNames = model.names;
    info.dofNames = std::vector<std::string>(jointNames.end() - info.armDim, jointNames.end());

    return info;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerSettings creatControllerSetting(const std::string& taskFile, const std::string& libraryFolder,const std::string& urdfFile, MobileManipulatorInfo robot_info){
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

//  Robot setting
    settings.robot_base_type = robot_info.robotBaseType;
    settings.dims = robot_info.OCPDim;
    settings.end_effector_link_name = robot_info.eeFrame;

//    settings.initial_state = VecXd::setZero(robot_info.OCPDim.robot.x);
//    settings.gravity
    ocs2::loadData::loadCppDataType(taskFile, "mobile_manipulator_interface.recompileLibraries", settings.recompile_libraries);
    ocs2::loadData::loadCppDataType(taskFile, "mobile_manipulator_interface.debug", settings.debug);

    EstimationSettings estimationSettings;
    TrackingSettings trackingSettings;
    ocs2::loadData::loadCppDataType(taskFile, "estimation.robot_init_variance", estimationSettings.robot_init_variance);
    ocs2::loadData::loadCppDataType(taskFile, "estimation.robot_process_variance", estimationSettings.robot_process_variance);
    ocs2::loadData::loadCppDataType(taskFile, "estimation.robot_measurement_variance", estimationSettings.robot_measurement_variance);
    settings.estimation = estimationSettings;
    ocs2::loadData::loadCppDataType(taskFile, "tracking.rate", trackingSettings.rate);
    ocs2::loadData::loadCppDataType(taskFile, "tracking.min_policy_update_time", trackingSettings.min_policy_update_time);

    ocs2::loadData::loadCppDataType(taskFile, "tracking.kp", trackingSettings.kp);
    ocs2::loadData::loadCppDataType(taskFile, "tracking.kv", trackingSettings.kv);
    ocs2::loadData::loadCppDataType(taskFile, "tracking.ka", trackingSettings.ka);

    ocs2::loadData::loadCppDataType(taskFile, "tracking.enforce_state_limits", trackingSettings.enforce_state_limits);
    ocs2::loadData::loadCppDataType(taskFile, "tracking.enforce_input_limits", trackingSettings.enforce_input_limits);
    ocs2::loadData::loadCppDataType(taskFile, "tracking.enforce_ee_position_limits", trackingSettings.enforce_ee_position_limits);

    ocs2::loadData::loadCppDataType(taskFile, "tracking.state_violation_margin", trackingSettings.state_violation_margin);
    ocs2::loadData::loadCppDataType(taskFile, "tracking.input_violation_margin", trackingSettings.input_violation_margin);
    ocs2::loadData::loadCppDataType(taskFile, "tracking.ee_position_violation_margin", trackingSettings.ee_position_violation_margin);
    ocs2::loadData::loadCppDataType(taskFile, "tracking.use_projective", trackingSettings.use_projectile);
    settings.tracking = trackingSettings;
// Weights Matrix
    ocs2::matrix_t inputMatrix(robot_info.OCPDim.robot.u, robot_info.OCPDim.robot.u);
    ocs2::matrix_t stateMatrix(robot_info.OCPDim.robot.x, robot_info.OCPDim.robot.x);
    ocs2::matrix_t EEMatrix(robot_info.armDim, robot_info.armDim);
    ocs2::loadData::loadEigenMatrix(taskFile, "Cost.input.diag", inputMatrix);
    ocs2::loadData::loadEigenMatrix(taskFile, "Cost.state.diag", stateMatrix);
    ocs2::loadData::loadEigenMatrix(taskFile, "Cost.end_effector.diag", EEMatrix);
    settings.input_weight = inputMatrix;
    settings.state_weight = stateMatrix;
    settings.end_effector_weight = EEMatrix;
// Constraint
// EE box constraint
    ocs2::loadData::loadCppDataType(taskFile,"Constraint.endEffectorBoxConstraint.enabled",settings.end_effector_box_constraint_enabled);
    ocs2::loadData::loadEigenMatrix(taskFile,"Constraint.endEffectorBoxConstraint.xyz_lower",settings.xyz_lower);
    ocs2::loadData::loadEigenMatrix(taskFile,"Constraint.endEffectorBoxConstraint.xyz_lower",settings.xyz_upper);

//    Limit constraint
//    ocs2::loadData::loadCppDataType(taskFile, "Constraint.endEffectorBoxConstraint.input_limit_mu", settings.input_limit_mu);
//    ocs2::loadData::loadCppDataType(taskFile, "Constraint.endEffectorBoxConstraint.input_limit_delta", settings.input_limit_delta);
    std::string limit_constraint_type;
    ocs2::loadData::loadCppDataType(taskFile,"Constraint.limits.constraint_type",limit_constraint_type);
    settings.limit_constraint_type = constraint_type_from_string(limit_constraint_type);
    ocs2::loadData::loadEigenMatrix(taskFile,"Constraint.limits.input.lower",settings.input_limit_lower);
    ocs2::loadData::loadEigenMatrix(taskFile,"Constraint.limits.input.upper",settings.input_limit_upper);
    ocs2::loadData::loadEigenMatrix(taskFile,"Constraint.limits.state.lower",settings.state_limit_lower);
    ocs2::loadData::loadEigenMatrix(taskFile,"Constraint.limits.state.upper",settings.state_limit_upper);

    //  TODO:  Add Constraint to avoid the path of the projectile

    return settings;
}


}
