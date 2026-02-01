# obb_node_fin.py (FINAL)
import rclpy
from rclpy.node import Node

import numpy as np
import math

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, Bool

import tf_transformations
from ultralytics import YOLO


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class OrientationEstimator(Node):
    def __init__(self):
        super().__init__('orientation_estimator')
        self.bridge = CvBridge()

        # ------------------------
        # Parameters
        # ------------------------
        self.declare_parameter("model_path", "/home/rokey/ros2_ws/best.pt")

        # yaw 판정(OK 범위, deg)
        self.declare_parameter("minangle_deg", 10.0)
        self.declare_parameter("maxangle_deg", 70.0)

        # 디바운스(연속 프레임)
        self.declare_parameter("defect_need", 5)
        self.declare_parameter("ok_need", 5)

        # Topics (in)
        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("depth_topic", "/depth")
        self.declare_parameter("camera_info_topic", "/camera_info")

        # Topics (out)
        self.declare_parameter("object_pose_topic", "/object_pose")
        self.declare_parameter("target_pose_topic", "/target_pose")     # latch
        self.declare_parameter("moverobot_topic", "/moverobot")
        self.declare_parameter("rotation_matrix_topic", "/rotation_matrix")
        self.declare_parameter("object_marker_topic", "/object_marker")
        self.declare_parameter("target_marker_topic", "/target_marker") # latch marker

        # marker on/off
        self.declare_parameter("publish_object_marker", True)
        self.declare_parameter("publish_target_marker", True)

        # yaw 보정(너가 쓰던 방식 유지용)
        self.declare_parameter("yaw_offset_deg", 90.0)    # pi/2
        self.declare_parameter("invert_yaw", True)

        # ------------------------
        # Read parameters
        # ------------------------
        self.model_path = self.get_parameter("model_path").value

        self.minangle = float(self.get_parameter("minangle_deg").value)
        self.maxangle = float(self.get_parameter("maxangle_deg").value)

        self.defect_need = int(self.get_parameter("defect_need").value)
        self.ok_need = int(self.get_parameter("ok_need").value)

        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        cam_topic = self.get_parameter("camera_info_topic").value

        self.object_pose_topic = self.get_parameter("object_pose_topic").value
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.moverobot_topic = self.get_parameter("moverobot_topic").value
        self.rotation_matrix_topic = self.get_parameter("rotation_matrix_topic").value
        self.object_marker_topic = self.get_parameter("object_marker_topic").value
        self.target_marker_topic = self.get_parameter("target_marker_topic").value

        self.publish_object_marker = bool(self.get_parameter("publish_object_marker").value)
        self.publish_target_marker = bool(self.get_parameter("publish_target_marker").value)

        self.yaw_offset = math.radians(float(self.get_parameter("yaw_offset_deg").value))
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)

        # ------------------------
        # Pub/Sub
        # ------------------------
        self.rgb_sub = self.create_subscription(Image, rgb_topic, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.cam_sub = self.create_subscription(CameraInfo, cam_topic, self.cam_callback, 10)

        self.object_pose_pub = self.create_publisher(PoseStamped, self.object_pose_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.move_pub = self.create_publisher(Bool, self.moverobot_topic, 10)
        self.matrix_pub = self.create_publisher(Float32MultiArray, self.rotation_matrix_topic, 10)

        self.object_marker_pub = self.create_publisher(Marker, self.object_marker_topic, 10)
        self.target_marker_pub = self.create_publisher(Marker, self.target_marker_topic, 10)

        # ------------------------
        # State
        # ------------------------
        self.rgb = None
        self.depth = None
        self.cam_info = None

        self.model = YOLO(self.model_path)

        self.defect_count = 0
        self.ok_count = 0
        self.current_defect = False   # 현재 /moverobot 상태

        self.get_logger().info(
            f"[obb_node] model={self.model_path} | OK yaw(deg)=({self.minangle}~{self.maxangle}) | "
            f"debounce defect_need={self.defect_need} ok_need={self.ok_need}\n"
            f"topics: in({rgb_topic},{depth_topic},{cam_topic}) out({self.object_pose_topic},{self.target_pose_topic},{self.moverobot_topic})"
        )

    def cam_callback(self, msg: CameraInfo):
        self.cam_info = msg

    def depth_callback(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg: Image):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if self.depth is None or self.cam_info is None:
            return

        obb = self.run_yolo_obb(self.rgb)
        if obb is None:
            self._update_debounce(defect_now=False, pose_xyzrpy=None)
            if self.publish_object_marker:
                self.delete_object_marker()
            return

        xc, yc, w, h, theta = obb

        depth_value = self.get_depth(xc, yc)
        if depth_value is None:
            self._update_debounce(defect_now=False, pose_xyzrpy=None)
            if self.publish_object_marker:
                self.delete_object_marker()
            return

        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        # 3D center
        X = (xc - cx) * depth_value / fx
        Y = (yc - cy) * depth_value / fy
        Z = depth_value

        # yaw(연출용 보정 포함)
        corrected_yaw = self._normalize_radian(theta - self.yaw_offset)
        yaw = -corrected_yaw if self.invert_yaw else corrected_yaw

        roll = 0.0
        pitch = 0.0

        # rotation matrix (optional)
        R = self.compute_rotation_matrix(roll, pitch, yaw)
        mat = Float32MultiArray()
        mat.data = R.flatten().astype(np.float32).tolist()
        self.matrix_pub.publish(mat)

        # always publish object pose
        self.publish_pose(self.object_pose_pub, X, Y, Z, roll, pitch, yaw)

        if self.publish_object_marker:
            self.publish_object_marker_cube(X, Y, Z, roll, pitch, yaw, w, h, depth_value)

        yaw_deg = math.degrees(yaw)
        ok_now = (self.minangle < yaw_deg < self.maxangle)
        defect_now = not ok_now

        self._update_debounce(defect_now=defect_now, pose_xyzrpy=(X, Y, Z, roll, pitch, yaw))

    # ------------------------
    # Debounce + state publish
    # ------------------------
    def _update_debounce(self, defect_now: bool, pose_xyzrpy):
        if defect_now:
            self.defect_count += 1
            self.ok_count = 0
        else:
            self.ok_count += 1
            self.defect_count = 0

        want_defect = self.current_defect
        if defect_now and self.defect_count >= self.defect_need:
            want_defect = True
        if (not defect_now) and self.ok_count >= self.ok_need:
            want_defect = False

        if want_defect == self.current_defect:
            return

        # 상태 변경
        self.current_defect = want_defect
        m = Bool()
        m.data = self.current_defect
        self.move_pub.publish(m)

        if self.current_defect:
            # 불량 확정 순간: target_pose latch publish + marker
            if pose_xyzrpy is not None:
                X, Y, Z, roll, pitch, yaw = pose_xyzrpy
                self.publish_pose(self.target_pose_pub, X, Y, Z, roll, pitch, yaw)
                if self.publish_target_marker:
                    self.publish_target_marker_sphere(X, Y, Z)

            self.get_logger().info("DEFECT confirmed -> /moverobot=True (latched /target_pose + /target_marker)")
        else:
            self.get_logger().info("OK confirmed -> /moverobot=False")

    # ------------------------
    # YOLO / geometry helpers
    # ------------------------
    def run_yolo_obb(self, img):
        results = self.model(img, verbose=False)
        if len(results) == 0 or results[0].obb is None or len(results[0].obb) == 0:
            return None
        data = results[0].obb.data.cpu().numpy()[0]
        xc, yc, w, h, theta, conf, cls = data
        return float(xc), float(yc), float(w), float(h), float(theta)

    def get_depth(self, x, y):
        h, w = self.depth.shape
        x = int(x)
        y = int(y)
        if x < 2 or x >= w - 2 or y < 2 or y >= h - 2:
            return None
        patch = self.depth[y-2:y+3, x-2:x+3]
        d = np.median(patch)
        if d == 0 or np.isnan(d):
            return None
        # mm -> m 추정
        if d > 10:
            d = d / 1000.0
        return float(d)

    def _normalize_radian(self, radian):
        radian = radian % (2 * math.pi)
        if radian > math.pi:
            radian -= (2 * math.pi)
        return radian

    def compute_rotation_matrix(self, roll, pitch, yaw):
        cx = np.cos(roll); sx = np.sin(roll)
        cy = np.cos(pitch); sy = np.sin(pitch)
        cz = np.cos(yaw); sz = np.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    # ------------------------
    # Publishers
    # ------------------------
    def publish_pose(self, pub, x, y, z, roll, pitch, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        pub.publish(msg)

    def publish_object_marker_cube(self, x, y, z, roll, pitch, yaw, w, h, depth):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        marker.pose.orientation.x = float(q[0])
        marker.pose.orientation.y = float(q[1])
        marker.pose.orientation.z = float(q[2])
        marker.pose.orientation.w = float(q[3])

        # px -> rough meter scale(데모)
        marker.scale.x = max(0.02, float(w) / 1000.0)
        marker.scale.y = max(0.02, float(h) / 1000.0)
        marker.scale.z = max(0.05, float(depth) / 2.0)

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.object_marker_pub.publish(marker)

    def delete_object_marker(self):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object"
        marker.id = 0
        marker.action = Marker.DELETE
        self.object_marker_pub.publish(marker)

    def publish_target_marker_sphere(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target"
        marker.id = 99
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.06

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.target_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = OrientationEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
