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


    MobileManipulatorInfo createMobileManipulatorInfo(const ocs2::PinocchioInterface& interface, const RobotBaseType& type,
                                                    const std::string& baseFrame, const std::string& eeFrame) {
        const auto& model = interface.getModel();

        MobileManipulatorInfo info;
        info.robotBaseType = type;
        switch (type) {
            case RobotBaseType::Nonholonomic: {
                info.OCPDim.robot.q = 9; // base: x,y,theta ; arm: q0,q1,q2,q3,q4,q5,q6
                info.OCPDim.robot.v = 8; // base: Vx, Vyaw ; arm: v0,v1,v2,v3,v4,v5,v6
                info.OCPDim.robot.x = info.OCPDim.robot.q + info.OCPDim.robot.v; // 9 + 8
                info.OCPDim.robot.u = 8; // // base: Jerk_x, Jerk_yaw ; arm: Jerk0,Jerk1,Jerk2,Jerk3,Jerk4,Jerk5,Jerk6
                break;
            }
            case RobotBaseType::Omnidirectional: {
                info.OCPDim.robot.q = 9; // base: x,y,theta ; arm: q0,q1,q2,q3,q4,q5,q6
                info.OCPDim.robot.v = 9; // base: Vx,Vy,Vyaw ; arm: v0,v1,v2,v3,v4,v5,v6
                info.OCPDim.robot.x = info.OCPDim.robot.q + info.OCPDim.robot.v; // 9 + 9
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
}  // namespace upright
