#!/usr/bin/env python3
"""
scripts/vision_core.py — 视觉动态检测节点 (Mode B)

订阅 MuJoCo 顶置相机图像，通过红色通道阈值检测杆件的位置和朝向，
发布 /detected_rod_pose (PoseStamped, frame_id=base_link)。
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion
import cv2
from cv_bridge import CvBridge
import numpy as np


class VisionCoreNode(Node):
    def __init__(self):
        super().__init__('vision_core_node')

        self.bridge = CvBridge()

        # 订阅 MuJoCo 顶置相机图像
        self.sub = self.create_subscription(
            Image,
            '/mujoco_server/cameras/overhead_camera/rgb/image_raw',
            self.image_callback,
            10
        )

        # 重新发布处理后的图像
        self.pub = self.create_publisher(Image, '/overhead_camera/image_raw', 10)

        # 发布检测到的杆件位姿 (base_link 坐标系)
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_rod_pose', 10)

        # 相机内参 —— 从第一帧图像分辨率 + 已知 fovy=60° 计算
        # （MuJoCo 场景相机不发布 camera_info，需要自行推断）
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self._intrinsics_initialized = False

        # 相机外参 —— 顶置相机固定在场景上方
        # 来源: dual_scene.xml <camera name="overhead_camera" pos="0.5 0.0 1.5" xyaxes="0 -1 0 1 0 0" fovy="60"/>
        self.cam_x = 0.5
        self.cam_y = 0.0
        self.cam_z = 1.5
        self.fovy = 60.0  # 垂直视场角 (度)
        # xyaxes: 图像右方向 = world -Y, 图像上方向 = world +X, 视线 = world -Z (看向地面)
        self.world_right = np.array([0.0, -1.0, 0.0])
        self.world_up    = np.array([1.0, 0.0, 0.0])
        self.world_look  = np.array([0.0, 0.0, -1.0])

        # 杆件检测参数 —— 使用红色通道阈值（杆为红色，棋盘为黑白）
        self.red_threshold = 180        # R通道阈值 (红色杆 R≈230, 棋盘灰≈76-204)
        self.green_blue_max = 120       # G和B通道上限 (红色杆G/B≈51, 排除白/灰)
        self.min_contour_area = 500     # 最小轮廓面积 (像素²)
        self.ground_z = 0.02            # 杆件中心高度 (地面)

        self._largest_contour = None    # 缓存最大轮廓用于绘制

        self.get_logger().info('Vision core node started: detecting rod on overhead camera')

    def _init_intrinsics(self, width: int, height: int):
        """从图像分辨率 + fovy 计算针孔相机内参."""
        # 垂直视场角 → 垂直焦距
        fy = (height / 2.0) / math.tan(math.radians(self.fovy / 2.0))
        fx = fy  # 方形像素假设
        cx = width / 2.0
        cy = height / 2.0
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self._intrinsics_initialized = True
        self.get_logger().info(
            f'Camera intrinsics (computed): {width}x{height} '
            f'fovy={self.fovy}° fx={fx:.1f} fy={fy:.1f} '
            f'cx={cx:.1f} cy={cy:.1f}'
        )

    def pixel_to_world(self, u: float, v: float) -> tuple:
        """
        将像素坐标 (u, v) 转换为世界坐标 (x, y, z_ground).
        使用地面平面 z = self.ground_z 的针孔模型.
        """
        # 归一化图像坐标
        xn = (u - self.cx) / self.fx
        yn = (v - self.cy) / self.fy

        # 相机视线方向向量 (在相机坐标系中为 (xn, yn, -1))
        # 转换到世界坐标: dir_world = right*xn + up*yn + look*(-1)
        look_dir = -self.world_look  # world_look = (0, 0, -1), so look_dir = (0, 0, 1)
        dir_world = self.world_right * xn + self.world_up * yn + look_dir

        # 参数化: P = cam_origin + t * dir_world
        # 与 z = ground_z 相交: cam_z + t * dir_world.z = ground_z
        if abs(dir_world[2]) < 1e-9:
            return None

        t = (self.ground_z - self.cam_z) / dir_world[2]

        world_x = self.cam_x + t * dir_world[0]
        world_y = self.cam_y + t * dir_world[1]

        return (world_x, world_y)

    def detect_rod(self, bgr: np.ndarray):
        """
        检测图像中的红色杆件（通过R通道阈值）。
        返回 (center_world_x, center_world_y, yaw_rad) 或 None。
        """
        # 分离 BGR 通道
        b, g, r = cv2.split(bgr)

        # 红色杆: R高 + G和B低 → 排除白/灰棋盘格
        binary = cv2.bitwise_and(
            cv2.threshold(r, self.red_threshold, 255, cv2.THRESH_BINARY)[1],
            cv2.bitwise_and(
                cv2.threshold(g, self.green_blue_max, 255, cv2.THRESH_BINARY_INV)[1],
                cv2.threshold(b, self.green_blue_max, 255, cv2.THRESH_BINARY_INV)[1]
            )
        )

        # 形态学闭运算: 填充杆件内部的小缺口
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        # 开运算: 去除小噪点
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # 选最大轮廓
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_contour_area:
            self._largest_contour = None
            return None

        self._largest_contour = largest

        # 最小外接矩形 → 中心像素 + 角度
        rect = cv2.minAreaRect(largest)
        (cx_px, cy_px), (w_px, h_px), angle_deg = rect

        # 世界坐标转换
        world_xy = self.pixel_to_world(cx_px, cy_px)
        if world_xy is None:
            return None
        world_x, world_y = world_xy

        # minAreaRect 的 angle: OpenCV 返回 [-90, 0), 0° = 长边水平（图像右向）
        # 图像右 = world -Y, 图像下 = world -X
        # 世界方向: (-sin(angle), -cos(angle))
        # yaw = atan2(world_y, world_x) = atan2(-cos(angle_rad), -sin(angle_rad))
        angle_rad = math.radians(angle_deg)
        world_yaw = math.atan2(-math.cos(angle_rad), -math.sin(angle_rad))

        return (world_x, world_y, world_yaw)

    def image_callback(self, msg: Image):
        try:
            # 第一帧：从图像分辨率初始化相机内参
            if not self._intrinsics_initialized:
                self._init_intrinsics(msg.width, msg.height)

            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 检测杆件（使用BGR彩色图像进行红色通道检测）
            result = self.detect_rod(cv_image)

            if result is not None:
                wx, wy, yaw = result

                # 发布检测位姿
                pose_msg = PoseStamped()
                pose_msg.header.stamp = msg.header.stamp
                pose_msg.header.frame_id = "base_link"
                pose_msg.pose.position.x = wx
                pose_msg.pose.position.y = wy
                pose_msg.pose.position.z = self.ground_z
                # yaw → quaternion
                pose_msg.pose.orientation = self.yaw_to_quat(yaw)
                self.pose_pub.publish(pose_msg)

                # 在图像上绘制检测结果
                cv2.drawContours(cv_image, [self._largest_contour], -1, (0, 255, 0), 2)

            # 发布处理后的图像
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Vision processing error: {e}')

    @staticmethod
    def yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = VisionCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
