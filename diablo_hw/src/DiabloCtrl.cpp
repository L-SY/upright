//
// Created by lsy on 24-4-18.
//
#include "diablo_hw/VulcanSerial/SerialPort.h"
#include "diablo_hw/OSDKVehicle.h"
#include "diablo_hw/DiabloCtrl.h"

namespace diablo_hw
{
DiabloRobot::DiabloRobot(DIABLO::OSDK::Vehicle* vehicle, DIABLO::OSDK::Movement_Ctrl* movementCtrl,
                         ros::NodeHandle nodeHandle)
  : vehicle_(vehicle), movementCtrl_(movementCtrl), loopRunning_(true)
{
  diabloCmdSub_ = nodeHandle.subscribe<geometry_msgs::Twist>("diablo_cmd", 1, &DiabloRobot::commandCB, this);
  testSub_ = nodeHandle.subscribe<std_msgs::String>("test_cmd", 1, &DiabloRobot::testCommandCB, this);
  imuPub_ = nodeHandle.advertise<sensor_msgs::Imu>("diablo_imu", 10);
  leftHipJointPub_ = nodeHandle.advertise<sensor_msgs::JointState>("left_hip_joint", 10);
  rightHipJointPub_ = nodeHandle.advertise<sensor_msgs::JointState>("right_hip_joint", 10);
  leftWheelJointPub_ = nodeHandle.advertise<sensor_msgs::JointState>("left_wheel_joint", 10);
  rightWheelJointPub_ = nodeHandle.advertise<sensor_msgs::JointState>("right_wheel_joint", 10);
  leftKneeJointPub_ = nodeHandle.advertise<sensor_msgs::JointState>("left_knee_joint", 10);
  rightKneeJointPub_ = nodeHandle.advertise<sensor_msgs::JointState>("right_knee_joint", 10);
  leftHipJoint_.name = { "left_hip" };
  rightHipJoint_.name = { "right_hip" };
  leftKneeJoint_.name = { "left_knee" };
  rightKneeJoint_.name = { "right_knee" };
  leftWheelJoint_.name = { "left_wheel" };
  rightWheelJoint_.name = { "right_wheel" };
  vehicle_->telemetry->activate();
  vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_10Hz);
  vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_50Hz);
  vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_50Hz);
  vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_50Hz);
  vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_10Hz);

  // Load ros params
  int error = 0;
  int threadPriority = 0;
  ros::NodeHandle nhP("~");
  error += static_cast<int>(!nhP.getParam("/diablo_hw/loop_frequency", loopHz_));
  error += static_cast<int>(!nhP.getParam("/diablo_hw/cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nhP.getParam("/diablo_hw/thread_priority", threadPriority));
  ROS_INFO_STREAM("loop_frequency = " << loopHz_);
  if (error > 0)
  {
    std::string error_message =
        "could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // Get current time for use with first update
  lastTime_ = Clock::now();

  // Setup loop thread
  loopThread_ = std::thread([&]() {
    while (loopRunning_)
    {
      update();
    }
  });
  sched_param sched{ .sched_priority = threadPriority };
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0)
  {
    ROS_WARN("Failed to set threads priority (one possible reason could be that the user and the group permissions "
             "are not set properly.).\n");
  }
}
void DiabloRobot::commandCB(const geometry_msgs::TwistConstPtr& msg)
{
  ROS_INFO_STREAM("111");
  cmd_rt_buffer_.writeFromNonRT(*msg);
}

void DiabloRobot::read()
{
  if (vehicle_->telemetry->newcome & 0x40)
  {
    // TODO ： Add robot state msgs
    vehicle_->telemetry->eraseNewcomeFlag(0xBF);
  }
  if (vehicle_->telemetry->newcome & 0x20)
  {
    imu_.orientation.w = vehicle_->telemetry->quaternion.w;
    imu_.orientation.x = vehicle_->telemetry->quaternion.x;
    imu_.orientation.y = vehicle_->telemetry->quaternion.y;
    imu_.orientation.z = vehicle_->telemetry->quaternion.z;
    vehicle_->telemetry->eraseNewcomeFlag(0xDF);
  }
  if (vehicle_->telemetry->newcome & 0x10)
  {
    imu_.linear_acceleration.x = vehicle_->telemetry->accl.x;
    imu_.linear_acceleration.y = vehicle_->telemetry->accl.y;
    imu_.linear_acceleration.z = vehicle_->telemetry->accl.z;
    vehicle_->telemetry->eraseNewcomeFlag(0xEF);
  }
  if (vehicle_->telemetry->newcome & 0x08)
  {
    imu_.angular_velocity.x = vehicle_->telemetry->gyro.x;
    imu_.angular_velocity.y = vehicle_->telemetry->gyro.y;
    imu_.angular_velocity.z = vehicle_->telemetry->gyro.z;
    vehicle_->telemetry->eraseNewcomeFlag(0xF7);
  }
  if (vehicle_->telemetry->newcome & 0x02)
  {
    // TODO : Add battery power get
    vehicle_->telemetry->eraseNewcomeFlag(0xFD);
  }
  if (vehicle_->telemetry->newcome & 0x01)
  {
    ROS_INFO_STREAM("1");
    leftHipJoint_.position = { vehicle_->telemetry->motors.left_hip.pos };
    leftHipJoint_.velocity = { vehicle_->telemetry->motors.left_hip.vel };
    leftHipJoint_.effort = { vehicle_->telemetry->motors.left_hip.iq };

    rightHipJoint_.position = { vehicle_->telemetry->motors.right_hip.pos };
    rightHipJoint_.velocity = { vehicle_->telemetry->motors.right_hip.vel };
    rightHipJoint_.effort = { vehicle_->telemetry->motors.right_hip.iq };

    leftKneeJoint_.position = { vehicle_->telemetry->motors.left_knee.pos };
    leftKneeJoint_.velocity = { vehicle_->telemetry->motors.left_knee.vel };
    leftKneeJoint_.effort = { vehicle_->telemetry->motors.left_knee.iq };

    rightKneeJoint_.position = { vehicle_->telemetry->motors.right_knee.pos };
    rightKneeJoint_.velocity = { vehicle_->telemetry->motors.right_knee.vel };
    rightKneeJoint_.effort = { vehicle_->telemetry->motors.right_knee.iq };

    leftWheelJoint_.position = { vehicle_->telemetry->motors.left_wheel.pos };
    leftWheelJoint_.velocity = { vehicle_->telemetry->motors.left_wheel.vel };
    leftWheelJoint_.effort = { vehicle_->telemetry->motors.left_wheel.iq };

    rightWheelJoint_.position = { vehicle_->telemetry->motors.right_wheel.pos };
    rightWheelJoint_.velocity = { vehicle_->telemetry->motors.right_wheel.vel };
    rightWheelJoint_.effort = { vehicle_->telemetry->motors.right_wheel.iq };
    vehicle_->telemetry->eraseNewcomeFlag(0xFE);
  }
  pubDiabloInfo();
}

void DiabloRobot::write()
{
  if (!movementCtrl_->in_control())
  {
    ROS_INFO_STREAM("Try to get the control of robot movement!.\n");
    movementCtrl_->obtain_control();
  }
  movementCtrl_->ctrl_data.up = 0.0f;
  movementCtrl_->ctrl_data.forward = 0.0f;
  movementCtrl_->ctrl_data.left = 0.0f;

  //  TODO: Should diy a msgs to control
  geometry_msgs::Twist cmd = *cmd_rt_buffer_.readFromRT();

  //  movementCtrl_->ctrl_mode_cmd = true;
  //  movementCtrl_->ctrl_mode_data.height_ctrl_mode = 1;
  //  movementCtrl_->ctrl_mode_data.pitch_ctrl_mode = 1;

  //  movementCtrl_->ctrl_data.forward = cmd.linear.x;
  movementCtrl_->ctrl_data.forward = 1;
  //  movementCtrl_->ctrl_data.up = cmd.linear.z;
  //  movementCtrl_->ctrl_data.pitch = cmd.angular.y;
  //  movementCtrl_->ctrl_data.roll = cmd.angular.x;
  //  movementCtrl_->ctrl_data.left = cmd.angular.z;
  //  ROS_INFO_STREAM(movementCtrl_->ctrl_data.forward);
  //  if (movementCtrl_->ctrl_mode_cmd)
  //  {
  //    uint8_t result = movementCtrl_->SendMovementModeCtrlCmd();
  //  }
  //  else
  //  {
  movementCtrl_->SendMovementCtrlCmd();
  //  }
}

void DiabloRobot::update()
{
  const auto currentTime = Clock::now();
  // Compute desired duration rounded to clock decimation
  const Duration desiredDuration(1.0 / loopHz_);

  // Get change in time
  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  // Check cycle time for excess delay
  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycle_time_error > cycleTimeErrorThreshold_)
  {
    //    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
    //                                                               << "cycle time: " << elapsedTime_ << "s, "
    //                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
  }

  // Input
  // get the hardware's state
  read();
  // Output
  // send the new command to hardware
  write();

  // Sleep
  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

}  // namespace diablo_hw