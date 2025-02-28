//
// Created by lsy on 24-3-14.
//

#include <iostream>

#include "ros/package.h"
#include "ros/ros.h"
#include "upright_control/MobileManipulatorInterface.h"
#include "upright_control/common/InterfaceSettings.h"
#include "upright_control/common/implementation/FactoryFunctionsImpl.h"

int main() {
  upright::ControllerSettings settings;
  const std::string taskFile = ros::package::getPath("upright_control") +
                               "/test/config/test_config.info";
  const std::string libFolder =
      ros::package::getPath("mm_assets") + "/auto_generated";
  const std::string urdfFile =
      ros::package::getPath("mm_assets") +
      "/description/ocs2_mobile_manipulator/urdf/ridgeback_ur5.urdf";
  settings = upright::creatControllerSetting(taskFile, libFolder, urdfFile);
  ROS_INFO_STREAM("GET FINISH");
  std::cout << settings << std::endl;
  ROS_INFO_STREAM("OUTPUT FINISH");
  return 0;
}