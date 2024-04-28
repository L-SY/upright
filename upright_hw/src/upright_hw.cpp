//
// Created by lsy on 24-3-7.
//

#include "upright_hw/UprightHW.h"

#include <generally_hw/GenerallyHWLoop.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "upright_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~");

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try
  {
    // Create the hardware interface specific to your robot
    std::shared_ptr<generally::UprightHW> UprightHW = std::make_shared<generally::UprightHW>();
    // Initialize the hardware interface:
    // 1. retrieve configuration from rosparam
    // 2. initialize the hardware and interface it with ros_control
    UprightHW->init(nh, robotHwNh);
    // Start the control loop
    generally::GenerallyHWLoop controlLoop(nh, UprightHW);
    // Wait until shutdown signal received
    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}