/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include "upright_control/dynamics/MobileManipulatorInfo.h"
namespace upright {

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (Nonholonomic or Omnidirectional)
 * @return PinocchioInterface
 */
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

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (Nonholonomic or Omnidirectional)
 * @param [in] jointNames: The joint names from URDF to make fixed/unactuated.
 * @return PinocchioInterface
 */
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

/** Load ManipulatorModelType for a config file */
RobotBaseType loadRobotBaseType(const std::string& configFilePath, const std::string& fieldName = "robotBaseType")
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(configFilePath, pt);
    const size_t type = pt.template get<size_t>(fieldName);
    return static_cast<RobotBaseType>(type);
}

MobileManipulatorInfo createMobileManipulatorInfo(const ocs2::PinocchioInterface& interface, const RobotBaseType& type,
                                                  const std::string& baseFrame, const std::string& eeFrame) {
    const auto& model = interface.getModel();

    MobileManipulatorInfo info;
    info.robotBaseType = type;
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
}  // namespace upright
