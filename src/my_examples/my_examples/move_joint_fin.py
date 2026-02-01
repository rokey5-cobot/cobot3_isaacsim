# move_joint_fin.py (FINAL)
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

import numpy as np
import math


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def param_array(node: Node, name: str, default_list):
    node.declare_parameter(name, default_list)
    return np.array(node.get_parameter(name).value, dtype=np.float64)


class TestROS2Bridge(Node):
    def __init__(self):
        super().__init__("test_ros2bridge")

        # ------------------------
        # Parameters
        # ------------------------
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("moverobot_topic", "/moverobot")
        self.declare_parameter("target_pose_topic", "/target_pose")

        # gripper demo topic (Bool)
        self.declare_parameter("gripper_topic", "/gripper_close")

        # publish frequency
        self.declare_parameter("publish_hz", 20.0)

        # poses (radian)
        self.home = param_array(self, "home_pose", [0, 0, 0, 0, 0, 0])
        self.approach_base = param_array(self, "approach_pose", [1.45, -0.55, 1.10, -1.15, 1.50, 0.0])
        self.pick_base     = param_array(self, "pick_pose",     [1.55, -0.70, 1.30, -1.25, 1.60, 0.0])
        self.retreat_base  = param_array(self, "retreat_pose",  [1.40, -0.45, 1.00, -1.05, 1.45, 0.0])

        # step timing (sec)
        self.declare_parameter("approach_sec", 1.2)
        self.declare_parameter("pick_sec", 1.2)
        self.declare_parameter("retreat_sec", 1.2)

        # retreat 이후 home 복귀 여부 (발표용: True면 마지막 자세에서 멈춤)
        self.declare_parameter("hold_after_retreat", True)
        self.declare_parameter("return_home_sec", 1.0)

        self.approach_sec = float(self.get_parameter("approach_sec").value)
        self.pick_sec = float(self.get_parameter("pick_sec").value)
        self.retreat_sec = float(self.get_parameter("retreat_sec").value)
        self.hold_after_retreat = bool(self.get_parameter("hold_after_retreat").value)
        self.return_home_sec = float(self.get_parameter("return_home_sec").value)

        # hint (target_pose -> joint_1)
        self.declare_parameter("use_target_hint", True)
        self.declare_parameter("hint_gain", 0.6)
        self.declare_parameter("hint_yaw_limit_deg", 90.0)

        self.use_target_hint = bool(self.get_parameter("use_target_hint").value)
        self.hint_gain = float(self.get_parameter("hint_gain").value)
        self.hint_yaw_limit = math.radians(float(self.get_parameter("hint_yaw_limit_deg").value))

        # ------------------------
        # Topics
        # ------------------------
        joint_cmd_topic = self.get_parameter("joint_command_topic").value
        moverobot_topic = self.get_parameter("moverobot_topic").value
        target_pose_topic = self.get_parameter("target_pose_topic").value
        gripper_topic = self.get_parameter("gripper_topic").value

        # ------------------------
        # Pub/Sub
        # ------------------------
        self.pub = self.create_publisher(JointState, joint_cmd_topic, 10)
        self.gripper_pub = self.create_publisher(Bool, gripper_topic, 10)

        self.sub_move = self.create_subscription(Bool, moverobot_topic, self.move_cb, 10)
        self.sub_target = self.create_subscription(PoseStamped, target_pose_topic, self.target_cb, 10)

        # ------------------------
        # JointState msg
        # ------------------------
        self.msg = JointState()
        self.msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.target_pose = self.home.copy()

        # target latch
        self.latched_target = None

        # state
        self.sequence_active = False
        self.sequence_step = 0
        self.step_timer = None

        # continuous publish
        hz = float(self.get_parameter("publish_hz").value)
        period = 1.0 / max(1.0, hz)
        self.timer = self.create_timer(period, self.pub_cb)

        self.get_logger().info(
            f"[move_joint] pub={joint_cmd_topic} sub=({moverobot_topic},{target_pose_topic}) "
            f"gripper={gripper_topic} hint={self.use_target_hint} gain={self.hint_gain}"
        )

    def target_cb(self, msg: PoseStamped):
        # defect 확정 순간 obb_node에서 1번 publish된 목표를 latch로 저장
        self.latched_target = msg

    def move_cb(self, msg: Bool):
        defect = bool(msg.data)

        if defect:
            if not self.sequence_active:
                print("target object detected!\nstarting to make an approach")
                self.start_sequence()
        else:
            # 정상(OK) 들어오면 즉시 HOME + 그리퍼 OPEN
            self.stop_sequence()
            self.target_pose = self.home.copy()
            self.publish_gripper(False)

    def pub_cb(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.position = self.target_pose.tolist()
        self.pub.publish(self.msg)

    def publish_gripper(self, close: bool):
        m = Bool()
        m.data = bool(close)
        self.gripper_pub.publish(m)
        self.get_logger().info(f"Gripper {'CLOSE' if close else 'OPEN'}")

    def apply_target_hint(self, base_pose: np.ndarray) -> np.ndarray:
        """
        진짜 IK 대신, target_pose의 (x,y) 방향만 사용해 joint_1만 살짝 보정(발표용 연출)
        """
        if (not self.use_target_hint) or (self.latched_target is None):
            return base_pose.copy()

        pose = base_pose.copy()
        x = float(self.latched_target.pose.position.x)
        y = float(self.latched_target.pose.position.y)

        yaw = math.atan2(y, x)
        yaw = clamp(yaw, -self.hint_yaw_limit, self.hint_yaw_limit)

        pose[0] = clamp(self.hint_gain * yaw, -math.pi, math.pi)
        return pose

    # ------------------------
    # Sequence control
    # ------------------------
    def start_sequence(self):
        self.stop_sequence()
        self.sequence_active = True
        self.sequence_step = 0
        self.advance_step()

    def advance_step(self):
        if not self.sequence_active:
            return

        if self.sequence_step == 0:
            # APPROACH
            self.target_pose = self.apply_target_hint(self.approach_base)
            self._schedule_next(self.approach_sec)

        elif self.sequence_step == 1:
            # PICK pose + 그리퍼 CLOSE(연출)
            self.target_pose = self.apply_target_hint(self.pick_base)
            self.publish_gripper(True)
            self._schedule_next(self.pick_sec)

        elif self.sequence_step == 2:
            # RETREAT + 그리퍼 OPEN(연출)
            self.target_pose = self.apply_target_hint(self.retreat_base)
            self.publish_gripper(False)

            if self.hold_after_retreat:
                self.sequence_active = False
                return
            else:
                self._schedule_next(self.retreat_sec)

        elif self.sequence_step == 3:
            # HOME 복귀
            self.target_pose = self.home.copy()
            self.sequence_active = False
            return

        self.sequence_step += 1

    def _schedule_next(self, sec: float):
        self.step_timer = self.create_timer(sec, self._step_timer_cb)

    def _step_timer_cb(self):
        if self.step_timer is not None:
            self.step_timer.cancel()
            self.step_timer = None
        self.advance_step()

    def stop_sequence(self):
        self.sequence_active = False
        self.sequence_step = 0
        if self.step_timer is not None:
            self.step_timer.cancel()
            self.step_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = TestROS2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
