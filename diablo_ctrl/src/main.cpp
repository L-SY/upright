//
// Created by lsy on 24-4-18.
//
#include "diablo_ctrl/DiabloCtrl.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diablo_ctrl");
  ros::NodeHandle nh("~");
  std::string dev;
  if (nh.hasParam("/diablo_ctrl/dev"))
    nh.getParam("/diablo_ctrl/dev", dev);
  else
  {
    ROS_ERROR("No dev info in /diablo_ctrl/dev !");
    return -1;
  }
  ros::AsyncSpinner spinner(3);
  spinner.start();

  DIABLO::OSDK::HAL_Pi Hal;
  if (Hal.init(dev, 460800))
    return -1;

  DIABLO::OSDK::Vehicle vehicle(&Hal);
  if (vehicle.init())
    return -1;

  DIABLO::OSDK::Movement_Ctrl movementCtrl = *vehicle.movement_ctrl;

  try
  {
    diablo_ctrl::DiabloRobot diabloRobot(&vehicle, &movementCtrl, nh);
    // Wait until shutdown signal received
    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the diablo hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}