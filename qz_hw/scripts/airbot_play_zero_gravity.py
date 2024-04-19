# ！env/usr/bin python3

import rospy
from sensor_msgs.msg import Joy, JointState

import os
import numpy as np
import pinocchio as pin
from qz_hw.msg import hybrid_force

NUM_ARM_JOINT = 6
CONTROL_FREQUENCY = 100
GRIPPER_JOY_CONTROL = False

CONTROL_FOLLOWER = False
JOINT_EFFORT_GAINS = np.array([0.65, 0.85, 0.75, 0.9, 0.9, 0.9])
JOINT_P_GAINS = np.array([10., 10., 10., 0.2, 0.2, 0.2])
FOLLOWER_JOINT_KP = np.array([75.0, 75.0, 75.0, 3.0, 3.0, 3.0])
MAX_EFFORT = 6.0

clip_joint_range = 2. * MAX_EFFORT / (FOLLOWER_JOINT_KP + 1e-6)


class AirbotPlayZeroGravity:
    def __init__(self, urdfs):
        global CONTROL_FOLLOWER

        self.arm_joint_range = np.array([
            [-3.14, -2.96, -0.087, -2.96, -1.74, -3.14],
            [2.09, 0.17, 3.14, 2.96, 1.74, 3.14]])

        while not rospy.is_shutdown():
            try:
                self.teacher_state_msg = rospy.wait_for_message("/airbot_play/teacher_joint_states", JointState,
                                                                timeout=2.0)
                rospy.loginfo("Get topic /airbot_play/teacher_joint_states")
                break
            except rospy.ROSException:
                rospy.logwarn("Waiting for message")
                continue

        while not rospy.is_shutdown():
            try:
                self.follow_state_msg = rospy.wait_for_message("/airbot_play/joint_states", JointState, timeout=3.0)
                rospy.loginfo("Get topic /airbot_play/joint_states")
                self.build_connection_with_follower = True
                CONTROL_FOLLOWER = True
                rospy.loginfo("Control follower")
                break
            except rospy.ROSException:
                rospy.logwarn("Waiting for message")
                continue

        self.recv_arm_msg_cnt = 0
        self.recv_state_msg = True
        self.tx_teacher_cmd_cnt = 0

        self.teacher_cmd_msg = hybrid_force()
        for i in range(6):
            self.teacher_cmd_msg.joint_names.append("joint{}".format(i + 1))
        if len(self.teacher_state_msg.position) > 6:
            self.teacher_cmd_msg.joint_names.append("newteacher")
            self.teacher_pin_model = pin.buildModelFromUrdf(urdfs["airbot_with_gripper"])
        else:
            self.teacher_pin_model = pin.buildModelFromUrdf(urdfs["airbot_play"])
        self.teacher_pin_data = self.teacher_pin_model.createData()

        self.teacher_cmd_puber = rospy.Publisher('/airbot_play/teacher_joint_command', hybrid_force, queue_size=5)
        rospy.Subscriber("/airbot_play/teacher_joint_states", JointState, self.teacherStateCallback)

        if CONTROL_FOLLOWER:
            self.tx_follow_cmd_cnt = 0
            self.follow_cmd_msg = hybrid_force()
            for i in range(6):
                self.follow_cmd_msg.joint_names.append("joint{}".format(i + 1))
            if len(self.teacher_state_msg.position) > 6:
                self.follow_cmd_msg.joint_names.append("gripper")
                self.follow_pin_model = pin.buildModelFromUrdf(urdfs["airbot_with_gripper"])
            else:
                self.follow_pin_model = pin.buildModelFromUrdf(urdfs["airbot_play"])
            self.follow_pin_data = self.follow_pin_model.createData()

            self.follow_cmd_puber = rospy.Publisher('/airbot_play/joint_command', hybrid_force, queue_size=5)
            rospy.Subscriber("/airbot_play/joint_states", JointState, self.followStateCallback)

        if GRIPPER_JOY_CONTROL:
            self.joy_cmd = Joy()
            self.joy_cmd.header.stamp = rospy.Time.now()
            self.joy_cmd.axes = [0., 0., 1., 0., 0., 1., 0., 0.]
            self.joy_cmd.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.joy_cmd_recv = False
            rospy.Subscriber("/joy", Joy, self.joyCallback)

    def teacherStateCallback(self, msg: JointState):
        self.teacher_state_msg = msg
        self.recv_state_msg = True
        self.tx_teacher_cmd_cnt = 0

    def followStateCallback(self, msg: JointState):
        self.follow_state_msg = msg
        self.recv_state_msg = True
        self.tx_follow_cmd_cnt = 0

    def joyCallback(self, msg: Joy):
        self.joy_cmd = msg
        self.joy_cmd_recv = True

    def update(self):

        if GRIPPER_JOY_CONTROL and self.joy_cmd_recv and self.joy_cmd.header.stamp.to_sec() - rospy.Time.now().to_sec() > 1.0:
            self.joy_cmd.axes = [0., 0., 1., 0., 0., 1., 0., 0.]
            self.joy_cmd.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.joy_cmd_recv = False

        q_teacher = np.array(self.teacher_state_msg.position)[:NUM_ARM_JOINT]
        v_teacher = np.array(self.teacher_state_msg.velocity)[:NUM_ARM_JOINT]
        pin.forwardKinematics(self.teacher_pin_model, self.teacher_pin_data, q_teacher, v_teacher,
                              np.zeros(NUM_ARM_JOINT))
        pin.nonLinearEffects(self.teacher_pin_model, self.teacher_pin_data, q_teacher, v_teacher)

        self.teacher_cmd_msg.header.stamp = rospy.Time.now()
        self.teacher_cmd_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.teacher_cmd_msg.kps = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.teacher_cmd_msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.teacher_cmd_msg.kds = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01]
        self.teacher_cmd_msg.effort = (JOINT_EFFORT_GAINS * self.teacher_pin_data.nle).tolist()

        if len(self.teacher_state_msg.position) > NUM_ARM_JOINT:
            self.teacher_cmd_msg.positions.append(0.0)
            self.teacher_cmd_msg.kps.append(0.0)
            if GRIPPER_JOY_CONTROL:
                axes_diff = self.joy_cmd.axes[2] - self.joy_cmd.axes[5]
                self.teacher_cmd_msg.velocities.append(axes_diff * 0.5)
                self.teacher_cmd_msg.kds.append(0.5)
                self.teacher_cmd_msg.effort.append(axes_diff * 0.3)
            else:
                self.teacher_cmd_msg.velocities.append(0.0)
                self.teacher_cmd_msg.kds.append(0.0)
                self.teacher_cmd_msg.effort.append(0.0)

        if CONTROL_FOLLOWER:
            if self.build_connection_with_follower:
                q_follower = np.array(self.follow_state_msg.position)[:NUM_ARM_JOINT]
                v_follower = np.array(self.follow_state_msg.velocity)[:NUM_ARM_JOINT]
                pin.forwardKinematics(self.follow_pin_model, self.follow_pin_data, q_follower, v_follower,
                                      np.zeros(NUM_ARM_JOINT))
                pin.nonLinearEffects(self.follow_pin_model, self.follow_pin_data, q_follower, v_follower)

                self.follow_cmd_msg.header.stamp = self.teacher_cmd_msg.header.stamp

                joint_diff = q_teacher - q_follower
                clip_joint_diff = np.clip(joint_diff, -clip_joint_range, clip_joint_range)

                self.follow_cmd_msg.positions = (q_follower + clip_joint_diff).tolist()
                ########### 这里是因为自研电机有bug ############
                self.follow_cmd_msg.positions[0] *= 1.25
                self.follow_cmd_msg.positions[1] *= 1.25
                self.follow_cmd_msg.positions[2] *= 1.25
                #############################################
                self.follow_cmd_msg.kps = FOLLOWER_JOINT_KP.tolist()
                self.follow_cmd_msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.follow_cmd_msg.kds = [1.0, 1.0, 1.0, 0.02, 0.02, 0.02]
                self.follow_cmd_msg.effort = np.clip(
                    JOINT_EFFORT_GAINS * self.follow_pin_data.nle + JOINT_P_GAINS * joint_diff, -MAX_EFFORT,
                    MAX_EFFORT).tolist()

                ################ 示教臂的力反馈 ###############
                # if np.any(np.abs(joint_diff) > 0.15) and np.all(np.abs(v_follower) < 1.0):
                self.teacher_cmd_msg.effort -= (0.5 * JOINT_P_GAINS * joint_diff)
                #############################################

                if len(self.follow_state_msg.position) > NUM_ARM_JOINT:
                    self.follow_cmd_msg.positions.append(0.0)
                    self.follow_cmd_msg.kps.append(0.0)
                    self.follow_cmd_msg.velocities.append(0.0)
                    self.follow_cmd_msg.kds.append(0.0)
                    self.follow_cmd_msg.effort.append(0.0)

                self.follow_cmd_puber.publish(self.follow_cmd_msg)

            self.tx_follow_cmd_cnt += 1
            if self.tx_follow_cmd_cnt > 20:
                if self.build_connection_with_follower:
                    self.build_connection_with_follower = False
                    rospy.logerr("Lost connection to airbot_play <FOLLOWER>")
            elif not self.build_connection_with_follower:
                self.build_connection_with_follower = True
                rospy.loginfo("Reconnect to airbot_play <FOLLOWER>")

        self.teacher_cmd_msg.effort = np.clip(self.teacher_cmd_msg.effort, -MAX_EFFORT, MAX_EFFORT).tolist()
        self.teacher_cmd_puber.publish(self.teacher_cmd_msg)

        self.tx_teacher_cmd_cnt += 1
        if self.tx_teacher_cmd_cnt > 20:
            rospy.logerr("Lost connection to airbot_play <TEACHER>")
            rospy.signal_shutdown("Lost connection to airbot_play teacher")


if __name__ == "__main__":

    rospy.init_node("airbot_play_zero_gravity")
    np.set_printoptions(precision=3, suppress=True, linewidth=1000, )

    model_pkg_dir = os.path.join(os.path.dirname(__file__), "../../arx7_model/")

    urdf_files = {
        "airbot_play": os.path.join(model_pkg_dir, "urdf/airbot_play_v2_1.urdf"),
        "airbot_with_gripper": os.path.join(model_pkg_dir, "urdf/airbot_with_gripper.urdf"),
    }

    airbot_play_zero_gravity = AirbotPlayZeroGravity(urdf_files)

    rate = rospy.Rate(CONTROL_FREQUENCY)
    while not rospy.is_shutdown():
        airbot_play_zero_gravity.update()
        rate.sleep()

    rospy.logwarn("airbot_play_zero_gravity is shutdown!")
