#pragma once

#include <Eigen/Core>

#include "upright_control/dynamics/MobileManipulatorInfo.h"
#include "upright_control/dynamics/BaseType.h"

namespace upright
{
/**
 * Provides read/write access to the base position.
 */
        template <typename SCALAR>
        Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const MobileManipulatorInfo& info);

/**
 * Provides read/write access to the base orientation.
 */
        template <typename SCALAR>
        Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const MobileManipulatorInfo& info);

/**
 * Provides read/write access to the arm joint angles.
 */
        template <typename Derived>
        Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const MobileManipulatorInfo& info);

/**
 * Provides read access to the arm joint angles.
 */
        template <typename Derived>
        const Eigen::Block<const Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const MobileManipulatorInfo& info);

}  // namespace upright


#include "implementation/AccessHelperFunctionsImpl.h"
