
#include <chrono>
#include <csignal>
#include <thread>

#include <airbot/motor_driver.hpp>
#include <app/airbot_play.hpp>
#include <drivers/can.hpp>
#include <drivers/imu_c_board.hpp>
#include <drivers/mit_motor.hpp>
#include <libraries/utils.hpp>

#define max_timeout_rx (100)

std::thread AitbotPlayThread;

std::vector<std::unique_ptr<arm::MotorDriver>> airbot_motor_driver;

bool g_dm_gripper = false;

// signal handler
static void airbotplay_signal_handler(int sig) {
  // make sure the robot thread is terminated before sending motor disable
  // cmd!!!!

  if (AitbotPlayThread.joinable() &&
      std::this_thread::get_id() != AitbotPlayThread.get_id()) {
    pthread_kill(AitbotPlayThread.native_handle(), sig);
  }

  int tx_cnt = 10;
  while (tx_cnt--) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ROS_WARN("Signal handled.");
  ros::shutdown();
}

/***********************************************************************************************************/
/*********************************************** AirbotPlay
 * ************************************************/
/***********************************************************************************************************/

AirbotPlay::AirbotPlay(ros::NodeHandle &nh, CanChannel_e can_channel,
                       bool is_teacher, bool with_gripper)
    : teacher_arm_(is_teacher), with_gripper_(with_gripper) {
  std::signal(SIGINT, airbotplay_signal_handler);
  std::signal(SIGTERM, airbotplay_signal_handler);

  arm_cmd_timeout_cnt_ = 0;

  arm_high_level_offline_ = true;

  arm_cmd_.resetCommand();

  /************ TODO ************/
  std::string gripper_type("gripper");
  if (teacher_arm_) {
    gripper_type = std::string("newteacher");
    g_dm_gripper = false;
  } else {
    gripper_type = std::string("gripper");
    g_dm_gripper = true;
  }
  /********** TODO-END **********/
  for (uint8_t i = 0; i < (with_gripper_ ? 7 : 6); i++) {
    airbot_motor_driver.push_back(
        arm::MotorDriver::MotorCreate(can_channel, i + 1, gripper_type));
  }

  auto can_callback = [](can_frame_t &frame) {
    for (auto &&moto : airbot_motor_driver) {
      auto motor_id = moto->get_motor_id();
      if (motor_id >= 1 && motor_id <= 3) {
        // OdMotorDriver
        int ret_cmd = moto->get_RET_CMD();
        int ret_cmd_id = motor_id | ret_cmd << 7;
        if ((frame.can_id == motor_id) ||
            (frame.can_id == 0X7FF && frame.data[1] == motor_id) ||
            (frame.can_id == ret_cmd_id)) {
          moto->CanRxMsgCallback(frame, 0);
          break;
        }
      } else if (motor_id >= 4 && motor_id <= 6) {
        // DmMotorDriver
        if ((frame.can_id == motor_id) ||
            (frame.data[0] == motor_id &&
             (frame.can_id == 0 || frame.can_id == 0x700 ||
              frame.can_id == 0x701 || frame.can_id == 0x702 ||
              frame.can_id == 0x703))) {
          moto->CanRxMsgCallback(frame, 0);
          break;
        }
      } else if (motor_id == 7) {
        if (g_dm_gripper) {
          // DmMotorDriver
          if ((frame.can_id == motor_id) ||
              (frame.data[0] == motor_id &&
               (frame.can_id == 0 || frame.can_id == 0x700 ||
                frame.can_id == 0x701 || frame.can_id == 0x702 ||
                frame.can_id == 0x703))) {
            moto->CanRxMsgCallback(frame, 0);
            break;
          }
        } else {
          // OdMotorDriver
          int ret_cmd = moto->get_RET_CMD();
          int ret_cmd_id = motor_id | ret_cmd << 7;
          if ((frame.can_id == motor_id) ||
              (frame.can_id == 0X7FF && frame.data[1] == motor_id) ||
              (frame.can_id == ret_cmd_id)) {
            moto->CanRxMsgCallback(frame, 0);
            break;
          }
        }
      }
    }
  };
  Can::can_handle.SetCanCallback(can_channel, can_callback);

  for (auto &&moto : airbot_motor_driver) {
    moto->MotorInit();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    moto->set_motor_control_mode(arm::MotorDriver::MIT);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (teacher_arm_) {
    arm_state_pub_ = nh.advertise<sensor_msgs::JointState>(
        "/airbot_play/teacher_joint_states", 3);
  } else {
    arm_state_pub_ =
        nh.advertise<sensor_msgs::JointState>("/airbot_play/joint_states", 3);
  }

  arm_state_msg_.name = {"joint1", "joint2", "joint3",
                         "joint4", "joint5", "joint6"};
  if (with_gripper_) {
    arm_state_msg_.position.resize(7);
    arm_state_msg_.velocity.resize(7);
    arm_state_msg_.effort.resize(7);
    arm_state_msg_.name.push_back(
        g_dm_gripper ? ("gripper")
                     : ("newteacher")); // :TODO: joint name gripper?
  } else {
    arm_state_msg_.position.resize(6);
    arm_state_msg_.velocity.resize(6);
    arm_state_msg_.effort.resize(6);
  }

  if (teacher_arm_) {
    arm_cmd_sub_ = nh.subscribe<qz_hw::hybrid_force>(
        "/airbot_play/teacher_joint_command", 3, &AirbotPlay::armCmdCallback_,
        this);
  } else {
    arm_cmd_sub_ = nh.subscribe<qz_hw::hybrid_force>(
        "/airbot_play/joint_command", 3, &AirbotPlay::armCmdCallback_, this);
  }
}

AirbotPlay::~AirbotPlay() {
  ROS_INFO("AirbotPlay::~AirbotPlay()");
  for (auto &&moto : airbot_motor_driver) {
    moto->MotorDeInit();
  }
}

void AirbotPlay::Handle() { arm_update_(); }

// 机械臂 电机 task
void AirbotPlay::arm_update_() {
  // arm state
  arm_state_msg_.header.stamp = ros::Time::now();
  for (int i = 0; i < arm_state_msg_.name.size(); i++) {
    arm_state_msg_.position[i] = airbot_motor_driver[i]->get_motor_pos();
    arm_state_msg_.velocity[i] = airbot_motor_driver[i]->get_motor_spd();
    arm_state_msg_.effort[i] = airbot_motor_driver[i]->get_motor_current();
  }
  arm_state_pub_.publish(arm_state_msg_);
  arm_cmd_handler_();
}

void AirbotPlay::arm_cmd_handler_() {
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
    for (int i = 0; i < 6; i++) {
      airbot_motor_driver[i]->MotorMitModeCmd(
          arm_cmd_.arm_t_p[i], arm_cmd_.arm_t_v[i], arm_cmd_.arm_kp[i],
          arm_cmd_.arm_kd[i], arm_cmd_.arm_f_tau[i]);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (with_gripper_) {
      airbot_motor_driver[6]->MotorMitModeCmd(
          arm_cmd_.arm_t_p[6], arm_cmd_.arm_t_v[6], arm_cmd_.arm_kp[6],
          arm_cmd_.arm_kd[6], arm_cmd_.arm_f_tau[6]);
    }
  }

  bool lost_connection = false;
  for (auto &&moto : airbot_motor_driver) {
    int count = moto->get_response_count();
    if (count > 500) {
      ROS_FATAL("arm motor [%d] {1-7} lost connection", moto->get_motor_id());
      lost_connection = true;
    }
  }

  if (lost_connection) {
    for (auto &&moto : airbot_motor_driver) {
      int count = moto->get_response_count();
      printf("motor %d response count: %d\n", moto->get_motor_id(), count);
    }
    ROS_FATAL("lost connection, exiting");
    std::raise(SIGTERM);
  }
}

void AirbotPlay::armCmdCallback_(
    const qz_hw::hybrid_force::ConstPtr &joint_cmd) {
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
