
#include <chrono>
#include <csignal>
#include <thread>

#include <airbot/motor_driver.hpp>
#include <app/robot.hpp>
#include <drivers/can.hpp>
#include <drivers/imu_c_board.hpp>
#include <drivers/mit_motor.hpp>
#include <libraries/utils.hpp>

#define max_timeout_rx (100)

std::thread RobotThread;

float leg_motor_calibrate_pos[3] = {-0.654f, -2.243f, 0.0f};

uint8_t ht_left_id[3] = {HT_MOTOR_LEFT_HIP, HT_MOTOR_LEFT_KNEE,
                         HT_MOTOR_LEFT_ANKLE};
uint8_t ht_right_id[3] = {HT_MOTOR_RIGHT_HIP, HT_MOTOR_RIGHT_KNEE,
                          HT_MOTOR_RIGHT_ANKLE};

std::vector<std::unique_ptr<arm::MotorDriver>> arm_motor_driver;

// signal handler
static void arx7_signal_handler(int sig) {
  // make sure the robot thread is terminated before sending motor disable
  // cmd!!!!

  if (RobotThread.joinable() &&
      std::this_thread::get_id() != RobotThread.get_id()) {
    pthread_kill(RobotThread.native_handle(), sig);
  }

  int tx_cnt = 10;
  while (tx_cnt--) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    HTMotor_MITCommmand(CAN1, 0x01, 0, 0, 0, 0.0, 0.0); // 左上
    HTMotor_MITCommmand(CAN2, 0x01, 0, 0, 0, 0.0, 0.0); // 右上
    HTMotor_MITCommmand(CAN1, 0x02, 0, 0, 0, 0.0, 0.0); // 左中
    HTMotor_MITCommmand(CAN2, 0x02, 0, 0, 0, 0.0, 0.0); // 右中
    HTMotor_MITCommmand(CAN1, 0x04, 0, 0, 0, 0.0, 0.0); // 左下
    HTMotor_MITCommmand(CAN2, 0x04, 0, 0, 0, 0.0, 0.0); // 右下
  }
  ROS_WARN("motors set zero force. Signal handled.");
  ros::shutdown();
}

/***********************************************************************************************************/
/************************************************** Robot
 * **************************************************/
/***********************************************************************************************************/

Robot::Robot(ros::NodeHandle &nh, bool arm, bool gripper)
    : withArm_(arm), withGripper_(gripper) {
  std::signal(SIGINT, arx7_signal_handler);
  std::signal(SIGTERM, arx7_signal_handler);

  leg_cmd_timeout_cnt_ = 0;
  arm_cmd_timeout_cnt_ = 0;

  leg_high_level_offline_ = true;
  arm_high_level_offline_ = true;

  leg_cmd_.resetCommand();
  arm_cmd_.resetCommand();

  for (int i = 0; i < 3; i++) {
    HT_motor_left[i].id = ht_left_id[i];
    HT_motor_right[i].id = ht_right_id[i];
  }

  CanChannel_e arm_can_channel = CAN4;

  if (withArm_) {
    /************ TODO ************/
    std::string gripper_type("gripper");
    /********** TODO-END **********/
    for (uint8_t i = 0; i < (withGripper_ ? 7 : 6); i++) {
      arm_motor_driver.push_back(
          arm::MotorDriver::MotorCreate(arm_can_channel, i + 1, gripper_type));
    }
  }

  auto can0_callback = [](can_frame_t &frame) { ImuCallbackFunc(&frame); };
  Can::can_handle.SetCanCallback(CAN0, can0_callback);
  auto can1_callback = [](can_frame_t &frame) {
    HTMotorLeftCallbackFunc(&frame);
  };
  Can::can_handle.SetCanCallback(CAN1, can1_callback);
  auto can2_callback = [](can_frame_t &frame) {
    HTMotorRightCallbackFunc(&frame);
  };
  Can::can_handle.SetCanCallback(CAN2, can2_callback);

  if (withArm_) {
    auto can_arm_callback = [](can_frame_t &frame) {
      for (auto &&moto : arm_motor_driver) {
        auto motor_id = moto->get_motor_id();
        if (motor_id >= 1 && motor_id <= 3) {
          int ret_cmd = moto->get_RET_CMD();
          int ret_cmd_id = motor_id | ret_cmd << 7;
          if ((frame.can_id == motor_id) ||
              (frame.can_id == 0X7FF && frame.data[1] == motor_id) ||
              (frame.can_id == ret_cmd_id)) {
            moto->CanRxMsgCallback(frame, 0);
            break;
          }
        } else if (motor_id >= 4 && motor_id <= 6) {
          if ((frame.can_id == motor_id) ||
              (frame.data[0] == motor_id &&
               (frame.can_id == 0 || frame.can_id == 0x700 ||
                frame.can_id == 0x701 || frame.can_id == 0x702 ||
                frame.can_id == 0x703))) {
            moto->CanRxMsgCallback(frame, 0);
            break;
          }
        } else if (motor_id == 7) {
          // TODO with gripper
          if ((frame.can_id == motor_id) ||
              (frame.data[0] == motor_id &&
               (frame.can_id == 0 || frame.can_id == 0x700 ||
                frame.can_id == 0x701 || frame.can_id == 0x702 ||
                frame.can_id == 0x703))) {
            moto->CanRxMsgCallback(frame, 0);
            break;
          }
        }
      }
    };
    Can::can_handle.SetCanCallback(arm_can_channel, can_arm_callback);

    for (auto &&moto : arm_motor_driver) {
      moto->MotorInit();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      moto->set_motor_control_mode(arm::MotorDriver::MIT);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  imu_pub_ = nh.advertise<sensor_msgs::Imu>("/arx7/imu", 3);
  joint_state_pub_ =
      nh.advertise<sensor_msgs::JointState>("/arx7/leg_joint_state", 3);

  joint_state_msg_.position.resize(6);
  joint_state_msg_.velocity.resize(6);
  joint_state_msg_.effort.resize(6);
  joint_state_msg_.name = {"left_hip",  "left_knee",  "left_wheel",
                           "right_hip", "right_knee", "right_wheel"};

  joint_cmd_sub_ = nh.subscribe<qz_hw::hybrid_force>(
      "/arx7/leg_command", 3, &Robot::legCmdCallback_, this);

  if (withArm_) {
    arm_state_pub_ =
        nh.advertise<sensor_msgs::JointState>("/airbot_play/joint_states", 3);
    arm_state_msg_.name = {"joint1", "joint2", "joint3",
                           "joint4", "joint5", "joint6"};
    if (withGripper_) {
      arm_state_msg_.position.resize(7);
      arm_state_msg_.velocity.resize(7);
      arm_state_msg_.effort.resize(7);
      arm_state_msg_.name.push_back("gripper"); // :TODO: joint name gripper?
    } else {
      arm_state_msg_.position.resize(6);
      arm_state_msg_.velocity.resize(6);
      arm_state_msg_.effort.resize(6);
    }
    arm_cmd_sub_ = nh.subscribe<qz_hw::hybrid_force>(
        "/airbot_play/joint_command", 3, &Robot::armCmdCallback_, this);
  }
}

Robot::~Robot() {
  if (withArm_) {
    for (auto &&moto : arm_motor_driver) {
      moto->MotorDeInit();
    }
  }
}

void Robot::Init() { set_leg_zero_(); }

void Robot::Handle() {
  imu_yaw_update_();
  leg_update_();
  if (withArm_) {
    arm_update_();
  }
}

// yaw轴数据计算
void Robot::imu_yaw_update_() {
  yawRead_ = imu_c_board.imu[0];

  float p1, p2, total;
  if (yawRead_ > lastYawRead_) {
    p1 = yawRead_ - lastYawRead_;
    p2 = yawRead_ - 2 * M_PI - lastYawRead_;
  } else {
    p1 = yawRead_ - lastYawRead_;
    p2 = yawRead_ + 2 * M_PI - lastYawRead_;
  }

  total = fabs(p1) > fabs(p2) ? p2 : p1;
  realYaw_ = realYaw_ + total;
  lastYawRead_ = yawRead_;

  // check imu offline
  clock_t c_now = clock();
  bool imu_lost = false;
  static std::map<int, std::string> imu_id2name = {
      {0, "angle"}, {1, "gyro"}, {2, "acc"}};
  for (int i = 0; i < 3; i++) {
    if (imu_c_board.connected[i]) {
      if ((double)(c_now - imu_c_board.last_rev[i]) / CLOCKS_PER_SEC >
          0.02) { // 20 ms
        ROS_FATAL("IMU %s timeout", imu_id2name[i].c_str());
        imu_lost = true;
      }
    }
  }
  if (imu_lost) {
    ROS_FATAL("IMU lost, exiting");
    std::raise(SIGTERM);
  }

  quaterniond_t q;
  double cy = cos(imu_c_board.imu[0] * 0.5);
  double sy = sin(imu_c_board.imu[0] * 0.5);
  double cp = cos(imu_c_board.imu[1] * 0.5);
  double sp = sin(imu_c_board.imu[1] * 0.5);
  double cr = cos(-imu_c_board.imu[2] * 0.5);
  double sr = sin(-imu_c_board.imu[2] * 0.5);
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  imu_state_sim_msg_.orientation.x = q.x;
  imu_state_sim_msg_.orientation.y = q.y;
  imu_state_sim_msg_.orientation.z = q.z;
  imu_state_sim_msg_.orientation.w = q.w;
  imu_state_sim_msg_.angular_velocity.x = -imu_c_board.gyro[1];
  imu_state_sim_msg_.angular_velocity.y = imu_c_board.gyro[0];
  imu_state_sim_msg_.angular_velocity.z = imu_c_board.gyro[2];
  imu_state_sim_msg_.linear_acceleration.x = -imu_c_board.accel[0];
  imu_state_sim_msg_.linear_acceleration.y = imu_c_board.accel[1];
  imu_state_sim_msg_.linear_acceleration.z = imu_c_board.accel[2];
  imu_pub_.publish(imu_state_sim_msg_);
}

// MIT电机task
void Robot::leg_update_() {
  joint_state_msg_.header.stamp = ros::Time::now();
  for (int i = 0; i < 3; i++) {
    joint_state_msg_.position[i] = HT_motor_left[i].position;
    joint_state_msg_.velocity[i] = HT_motor_left[i].velocity;
    joint_state_msg_.effort[i] = HT_motor_left[i].current;
    joint_state_msg_.position[i + 3] = HT_motor_right[i].position;
    joint_state_msg_.velocity[i + 3] = HT_motor_right[i].velocity;
    joint_state_msg_.effort[i + 3] = HT_motor_right[i].current;
  }
  joint_state_pub_.publish(joint_state_msg_);
  leg_cmd_handler_();
}

// 机械臂 电机 task
void Robot::arm_update_() {
  // arm state
  arm_state_msg_.header.stamp = ros::Time::now();
  for (int i = 0; i < 6; i++) {
    arm_state_msg_.position[i] = arm_motor_driver[i]->get_motor_pos();
    arm_state_msg_.velocity[i] = arm_motor_driver[i]->get_motor_spd();
    arm_state_msg_.effort[i] = arm_motor_driver[i]->get_motor_current();
  }
  arm_state_pub_.publish(arm_state_msg_);
  arm_cmd_handler_();
}

// 电机位置控制
void Robot::leg_cmd_handler_() {
  if (leg_cmd_.exchange_num) {
    if (leg_high_level_offline_) {
      ROS_INFO("high level controller online (leg)");
      leg_high_level_offline_ = false;
    }
    leg_cmd_.exchange_num = 0;
    leg_cmd_timeout_cnt_ = 0;
  } else {
    if (!leg_high_level_offline_) {
      leg_cmd_timeout_cnt_++;
      if (leg_cmd_timeout_cnt_ >= max_timeout_rx) { // timeout zero force
        ROS_ERROR("high level controller offline (leg)");
        leg_high_level_offline_ = true;
        leg_cmd_timeout_cnt_ = max_timeout_rx;
        {
          std::lock_guard<std::mutex> cmd_lock(cmd_leg_mutex_);
          leg_cmd_.resetCommand();
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> cmd_lock(cmd_leg_mutex_);
    HTMotor_MITCommmand(CAN1, 0x01,
                        -(leg_cmd_.t_p[0] + HT_motor_left[0].position_zero),
                        -leg_cmd_.t_v[0], leg_cmd_.kp[0], leg_cmd_.kd[0],
                        -leg_cmd_.f_tau[0]); // 左上
    HTMotor_MITCommmand(CAN2, 0x01,
                        (leg_cmd_.t_p[3] + HT_motor_right[0].position_zero),
                        leg_cmd_.t_v[3], leg_cmd_.kp[3], leg_cmd_.kd[3],
                        leg_cmd_.f_tau[3]); // 右上
    HTMotor_MITCommmand(CAN1, 0x02,
                        -(leg_cmd_.t_p[1] + HT_motor_left[1].position_zero),
                        -leg_cmd_.t_v[1], leg_cmd_.kp[1], leg_cmd_.kd[1],
                        -leg_cmd_.f_tau[1]); // 左中
    HTMotor_MITCommmand(CAN2, 0x02,
                        (leg_cmd_.t_p[4] + HT_motor_right[1].position_zero),
                        leg_cmd_.t_v[4], leg_cmd_.kp[4], leg_cmd_.kd[4],
                        leg_cmd_.f_tau[4]); // 右中
    HTMotor_MITCommmand(CAN1, 0x04,
                        -(leg_cmd_.t_p[2] + HT_motor_left[2].position_zero),
                        -leg_cmd_.t_v[2], leg_cmd_.kp[2], leg_cmd_.kd[2],
                        -leg_cmd_.f_tau[2]); // 左下
    HTMotor_MITCommmand(CAN2, 0x04,
                        (leg_cmd_.t_p[5] + HT_motor_right[2].position_zero),
                        leg_cmd_.t_v[5], leg_cmd_.kp[5], leg_cmd_.kd[5],
                        leg_cmd_.f_tau[5]); // 右下
  }

  bool lost_connection = false;
  for (int i = 0; i < 3; i++) {
    HT_motor_left[i].tx_cnt++;
    HT_motor_right[i].tx_cnt++;
    if (HT_motor_left[i].tx_cnt > 10) {
      ROS_FATAL("left leg motor [%d] {1,2,4} lost connection", ht_left_id[i]);
      lost_connection = true;
    }
    if (HT_motor_right[i].tx_cnt > 10) {
      ROS_FATAL("right motor [%d] {1,2,4} lost connection", ht_right_id[i]);
      lost_connection = true;
    }
  }

  if (lost_connection) { // lost connection, broadcast zero force for 5 times
    ROS_FATAL("lost connection, exiting");
    std::raise(SIGTERM);
  }
}

void Robot::arm_cmd_handler_() {
  static uint8_t arm_ctr_id = 0;
  if (arm_cmd_.arm_exchange_num) {
    if (arm_high_level_offline_) {
      ROS_INFO("high level controller online (arm)");
      arm_high_level_offline_ = false;
    }
    arm_cmd_.arm_exchange_num = 0;
    arm_cmd_timeout_cnt_ = 0;
  } else {
    if (!arm_high_level_offline_) {
      arm_cmd_timeout_cnt_++;
      if (arm_cmd_timeout_cnt_ >= max_timeout_rx) { // timeout zero force
        ROS_ERROR("high level controller offline (arm)");
        arm_high_level_offline_ = true;
        arm_cmd_timeout_cnt_ = max_timeout_rx;
        {
          std::lock_guard<std::mutex> cmd_lock(cmd_arm_mutex_);
          arm_cmd_.resetCommand();
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> cmd_lock(cmd_arm_mutex_);
    if (arm_ctr_id < 6) {
      arm_motor_driver[arm_ctr_id]->MotorMitModeCmd(
          arm_cmd_.arm_t_p[arm_ctr_id], arm_cmd_.arm_t_v[arm_ctr_id],
          arm_cmd_.arm_kp[arm_ctr_id], arm_cmd_.arm_kd[arm_ctr_id],
          arm_cmd_.arm_f_tau[arm_ctr_id]);
    } else if (withGripper_) {
      arm_motor_driver[arm_ctr_id]->MotorMitModeCmd(
          arm_cmd_.arm_t_p[arm_ctr_id], arm_cmd_.arm_t_v[arm_ctr_id],
          arm_cmd_.arm_kp[arm_ctr_id], arm_cmd_.arm_kd[arm_ctr_id],
          arm_cmd_.arm_f_tau[arm_ctr_id]);
    }
  }

  arm_ctr_id = (arm_ctr_id + 1) % arm_motor_driver.size();

  bool lost_connection = false;
  for (auto &&moto : arm_motor_driver) {
    int count = moto->get_response_count();
    if (count > 100) {
      ROS_FATAL("arm motor [%d] {1-7} lost connection", moto->get_motor_id());
      lost_connection = true;
    }
  }

  if (lost_connection) {
    for (auto &&moto : arm_motor_driver) {
      int count = moto->get_response_count();
      printf("motor %d response count: %d\n", moto->get_motor_id(), count);
    }
    ROS_FATAL("lost connection, exiting");
    std::raise(SIGTERM);
  }
}

// 零点设置
void Robot::set_leg_zero_() {
  ROS_WARN("leg init begin");
  // 回零参数
  float Set_leg_zero_v = 1.0f;
  float Set_leg_zero_kv = 3.0f;
  float Set_leg_zero_I_line = 5.0f;

  // 到位标志
  bool can1_01_ok, can1_02_ok, can2_01_ok, can2_02_ok;
  can1_01_ok = can1_02_ok = can2_01_ok = can2_02_ok = false;

  // 电机初始化
  // 多次循环确保初始化成功
  for (int i = 0; i < 5; i++) {
    for (int j : ht_left_id) {
      HTMotor_Start(CAN1, j);
    }
    for (int j : ht_right_id) {
      HTMotor_Start(CAN2, j);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ROS_INFO(
      "Please put the robot in calibration configuration, and press enter");
  getchar();
  fflush(stdin);

  for (int cnt = 0; cnt < 10; cnt++) {
    for (int i : {0, 1, 2}) {
      HTMotor_MITCommmand(CAN1, ht_left_id[i], 0, 0, 0, 0, 0);
      HTMotor_MITCommmand(CAN2, ht_right_id[i], 0, 0, 0, 0, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // 延时确保到位
  ros::Time::sleepUntil(ros::Time::now() + ros::Duration(1));

  // 赋零点
  for (int i = 0; i < 3; i++) {
    HT_motor_left[i].position_zero =
        HT_motor_left[i].position - leg_motor_calibrate_pos[i];
    HT_motor_right[i].position_zero =
        HT_motor_right[i].position - leg_motor_calibrate_pos[i];
  }

  leg_cmd_.resetCommand();
  arm_cmd_.resetCommand();

  if (withArm_) {
    for (int i = 0; i < 3; i++) {
      arm_motor_driver[i]->MotorUnlock();
    }
  }

  ROS_WARN("leg init finished");
}

void Robot::legCmdCallback_(const qz_hw::hybrid_force::ConstPtr &joint_cmd) {
  std::lock_guard<std::mutex> cmd_lock(cmd_leg_mutex_);
  for (int i = 0; i < 6; i++) {
    leg_cmd_.t_p[i] = joint_cmd->positions[i];
    leg_cmd_.t_v[i] = joint_cmd->velocities[i];
    leg_cmd_.kp[i] = joint_cmd->kps[i];
    leg_cmd_.kd[i] = joint_cmd->kds[i];
    leg_cmd_.f_tau[i] = joint_cmd->effort[i];
  }
  leg_cmd_.exchange_num = 1; // update
}

void Robot::armCmdCallback_(const qz_hw::hybrid_force::ConstPtr &joint_cmd) {
  std::lock_guard<std::mutex> cmd_lock(cmd_arm_mutex_);
  for (int i = 0; i < joint_cmd->positions.size(); i++) {
    arm_cmd_.arm_t_p[i] = joint_cmd->positions[i];
    arm_cmd_.arm_t_v[i] = joint_cmd->velocities[i];
    arm_cmd_.arm_kp[i] = joint_cmd->kps[i];
    arm_cmd_.arm_kd[i] = joint_cmd->kds[i];
    arm_cmd_.arm_f_tau[i] = joint_cmd->effort[i];
  }
  arm_cmd_.arm_exchange_num = 1; // update
}
