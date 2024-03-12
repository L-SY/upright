#pragma once

// C/C++
#include <string>
#include <vector>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/model.hpp>

#include "BaseType.h"
#include "Dimensions.h"
namespace upright {
/**
 * @brief A data structure to store manipulator information.
 *
 * The attributes are filled by resolving the URDF model parsed.
 */
    struct MobileManipulatorInfo {
        RobotBaseType robotBaseType;  // type of manipulator: floating-base, fully-actuated floating-base, wheel-base, default
        OptimizationDimensions OCPDim;
        const int armDim = 6;
        std::string baseFrame;                      // name of the root frame of the robot
        std::string eeFrame;                        // name of the end-effector frame of the robot
        std::vector<std::string> dofNames;          // name of the actuated DOFs in the robot
    };
}  // namespace upright
