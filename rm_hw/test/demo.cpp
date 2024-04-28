//
// Created by lsy on 24-4-26.
//

#include "rm_hw/rmInterface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "api_moveJ_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spin(1);
  spin.start();
  rm_interface::rmInterface rmInterface(nh);
  ros::Rate loop_rate(100);  // Set a suitable rate to check the joystick status
  bool is_init = false;
  ROS_INFO_STREAM("Enter rm_test!");
  while (ros::ok())
  {
    if (rmInterface.ps5Joy_->getButtonState(joy::PS5ButtonMap::R1) == 1 && !is_init)
    {
      ROS_INFO_STREAM("R1 IS PRESS, now return home");
      rmInterface.moveToZero();
      is_init = true;
    }
    if (rmInterface.ps5Joy_->getAxisValue(joy::PS5ButtonMap::R3Horizontal) != 0)
    {
      ROS_INFO_STREAM("R1 IS PRESS, now servo mode");
      rmInterface.servoControl(rmInterface.ps5Joy_->getAxisValue(joy::PS5ButtonMap::R3Horizontal), 0, 0, 0, 0, 0, 0.2);
    }
    ros::spinOnce();  // Handle callbacks
    loop_rate.sleep();
  }
  return 0;
}