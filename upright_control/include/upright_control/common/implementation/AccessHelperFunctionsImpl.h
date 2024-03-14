#pragma once

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <upright_control/common/AccessHelpFunctions.h>
namespace upright{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const MobileManipulatorInfo& info) {
    assert(state.rows() == info.OCPDim.robot.x);
    assert(state.cols() == 1);
    // resolve the position vector based on robot type.
    switch (info.robotBaseType) {
        case RobotBaseType::Nonholonomic: {
            // for Nonholonomic wheel-based, we assume 2D base position
            return Eigen::Matrix<SCALAR, 3, 1>(state(0), state(1), 0.0);
        }
        case RobotBaseType::Omnidirectional: {
            // for Omnidirectional wheel-based, we assume 2D base position
            return Eigen::Matrix<SCALAR, 3, 1>(state(0), state(1), 0.0);
        }
        default:
            throw std::invalid_argument("Invalid manipulator model type provided.");
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const MobileManipulatorInfo& info) {
    assert(state.rows() == info.OCPDim.robot.x);
    assert(state.cols() == 1);
    // resolve the position vector based on robot type.
    switch (info.robotBaseType) {
        case RobotBaseType::Nonholonomic: {
            // for Nonholonomic wheel-based, wwe assume only yaw
            return Eigen::Quaternion<SCALAR>(Eigen::AngleAxis<SCALAR>(state(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ()));
        }
        case RobotBaseType::Omnidirectional: {
            // for Omnidirectional wheel-based, we assume only yaw
            return Eigen::Quaternion<SCALAR>(Eigen::AngleAxis<SCALAR>(state(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ()));
        }
        default:
            throw std::invalid_argument("Invalid manipulator model type provided.");
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const MobileManipulatorInfo& info) {
    assert(state.rows() == info.OCPDim.robot.x);
    assert(state.cols() == 1);
    const size_t startRow = info.OCPDim.robot.q - info.armDim;
    return Eigen::Block<Derived, -1, 1>(state.derived(), startRow, 0, info.armDim, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(const Eigen::MatrixBase<Derived>& state, const MobileManipulatorInfo& info) {
    assert(state.rows() == info.OCPDim.robot.x);
    assert(state.cols() == 1);
    // resolve for arm dof start index
    const size_t startRow = info.OCPDim.robot.q - info.armDim;
    return Eigen::Block<const Derived, -1, 1>(state.derived(), startRow, 0, info.armDim, 1);
}

}  // upright
