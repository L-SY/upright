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
  diabloCmdSub_ = nodeHandle.subscribe<geometry_msgs::Twist>("/diablo_cmd", 1, &DiabloRobot::commandCB, this);
  joySub_ = nodeHandle.subscribe<sensor_msgs::Joy>("/joy", 1, &DiabloRobot::joyCommandCB, this);
  imuPub_ = nodeHandle.advertise<sensor_msgs::Imu>("diablo_imu", 10);
  odomPub_ = nodeHandle.advertise<std_msgs::Float64MultiArray>("/diablo_odom", 10);
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
  vehicle_->telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_50Hz);
  vehicle_->telemetry->configUpdate();
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

    if (!is_init_)
    {
      init_odom_x_ = vehicle_->telemetry->motors.right_wheel.pos / 10.0;  // for compensation the init x
      is_init_ = true;
    }

    //    the msg is build by [ x, y, z, roll, pitch, yaw, vx, vy, vz, v_roll, v_pitch, v_yaw]
    std_msgs::Float64MultiArray diablo_odom_data;
    double yaw = vehicle_->telemetry->motors.left_wheel.pos / 10;
    double v_x = (vehicle_->telemetry->motors.left_wheel.vel) + (vehicle_->telemetry->motors.right_wheel.vel) / 2;
    const auto currentTime = Clock::now();
    Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
    double time = ros::Duration(time_span.count()).toSec();
    odom_y_ += sin(yaw) * v_x * time;
    diablo_odom_data.data.push_back(vehicle_->telemetry->motors.right_wheel.pos / 10.0 - init_odom_x_);  // x
    // for the loopHz is almost be follow and (1/ loopHz_) is small
    diablo_odom_data.data.push_back(odom_y_);                                                  // y
    diablo_odom_data.data.push_back(vehicle_->telemetry->motors.left_knee.pos / 10.0 - 0.05);  // z
    diablo_odom_data.data.push_back(0.);                                                       // roll
    diablo_odom_data.data.push_back(0.);                                                       // pitch
    diablo_odom_data.data.push_back(yaw);                                                      // yaw
    diablo_odom_data.data.push_back(v_x);                                                      // v_x
    diablo_odom_data.data.push_back(0.);                                                       // v_y
    diablo_odom_data.data.push_back(0.);                                                       // v_z
    diablo_odom_data.data.push_back(0.);                                                       // v_roll
    diablo_odom_data.data.push_back(0.);                                                       // v_pitch
    // 0.49 is the distance of two wheels
    diablo_odom_data.data.push_back((vehicle_->telemetry->motors.left_wheel.vel) -
                                    (vehicle_->telemetry->motors.right_wheel.vel) / 0.49);  // v_yaw
    //  double x = (vehicle_->telemetry->quaternion.x);
    //  double y = (vehicle_->telemetry->quaternion.y);
    //  double z = (vehicle_->telemetry->quaternion.z);
    //  double w = (vehicle_->telemetry->quaternion.w);
    //  double yaw = std::atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z);
    //  diablo_odom_data.data.push_back(yaw);
    odomPub_.publish(diablo_odom_data);
  }
  pubDiabloInfo();
}

void DiabloRobot::write()
{
  judgeControlMode();
  if (Mode_ != NONE)
    if (!movementCtrl_->in_control())
    {
      ROS_INFO_STREAM("Try to get the control of robot movement!.\n");
      movementCtrl_->obtain_control(500);
    }

  if (Mode_ == TOPIC)
  {
    movementCtrl_->ctrl_data.forward = 0.0f;
    movementCtrl_->ctrl_data.left = 0.0f;

    //  TODO: Should diy a msgs to control
    geometry_msgs::Twist cmd = *cmd_rt_buffer_.readFromRT();

    movementCtrl_->ctrl_data.forward = cmd.linear.x;
    movementCtrl_->ctrl_data.left = cmd.angular.z;
    ROS_INFO_STREAM(cmd.linear.x);
    movementCtrl_->SendMovementCtrlCmd();
  }
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
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                               << "cycle time: " << elapsedTime_ << "s, "
                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
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