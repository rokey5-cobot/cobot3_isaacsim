import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class TestROS2Bridge(Node):
    def __init__(self):
        super().__init__("test_ros2bridge")

        self.publisher_ = self.create_publisher(JointState, "/joint_command", 10)

        # JointState 기본 설정
        self.joint_state = JointState()
        self.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

        # 처음 시작할 때 로봇 관절은 IsaacSim에서 결정됨 (따로 설정 X)
        self.motion_enabled = True

        # 0.05초마다 publish
        self.timer = self.create_timer(0.05, self.timer_callback)

        # 42초 후 딱 한 번 실행
        self.create_timer(42.0, self.after_42_seconds)


    def timer_callback(self):
        """
        motion_enabled=True 동안에는 로봇이 기존 자세 유지.
        motion_enabled=False 되면 after_42_seconds 에서 설정한 target_pose 고정 유지.
        """
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)


    def after_42_seconds(self):
        print("Move to the location according to the judgment result")

        # 42초 후 목표 자세 (여기 원하는 대로 수정 가능)
        target_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        self.joint_state.position = target_pose.tolist()

        self.motion_enabled = False
        self.get_logger().info("=== Move to target pose after capturing ===")
        self.publisher_.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = TestROS2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

