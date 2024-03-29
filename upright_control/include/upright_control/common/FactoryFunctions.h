#pragma once

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <yaml-cpp/yaml.h>
#include "upright_control/dynamics/MobileManipulatorInfo.h"
#include "upright_control/common/InterfaceSettings.h"

namespace upright {

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (Nonholonomic or Omnidirectional)
 * @return PinocchioInterface
 */
ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const RobotBaseType& type);

/** Create a MobileManipulatorModel PinocchioInterface from a URDF
 * @param [in] robotUrdfPath: The robot URDF path.
 * @param [in] type: Type of robot model (Nonholonomic or Omnidirectional)
 * @param [in] jointNames: The joint names from URDF to make fixed/unactuated.
 * @return PinocchioInterface
 */
ocs2::PinocchioInterface createPinocchioInterface(const std::string& robotUrdfPath, const RobotBaseType& type,
                                                    const std::vector<std::string>& jointNames);

/** Load ManipulatorModelType for a config file */
RobotBaseType loadRobotBaseType(const std::string& configFilePath, const std::string& fieldName);

MobileManipulatorInfo createMobileManipulatorInfo(const ocs2::PinocchioInterface& interface, const ControllerSettings& controllerSettings);

ControllerSettings creatControllerSetting(const std::string& taskFile, const std::string& libraryFolder,const std::string& urdfFile);

template <typename Scalar>
std::map<std::string, BalancedObject<Scalar>> loadObjects(const YAML::Node& node);

template <typename Scalar>
std::vector<ContactPoint<Scalar>> loadContact(const YAML::Node& node);
}  // namespace upright