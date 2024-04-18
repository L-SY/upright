//
// Created by lsy on 24-4-18.
//
#include "diablo_sdk/DiabloCtrl.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diablo_ctrl");
  ros::NodeHandle nh("~");

  DIABLO::OSDK::HAL_Pi Hal;
  if (Hal.init())
    return -1;

  DIABLO::OSDK::Vehicle vehicle(&Hal);
  if (vehicle.init())
    return -1;

  vehicle.telemetry->activate();
  auto movementCtrl = vehicle.movement_ctrl;

  try
  {
    diablo_sdk::DiabloRobot diabloRobot(&vehicle, movementCtrl, nh);
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