import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose


class BlobDepthTo3D(Node):
    """
    ROS2 节点：将颜色块 2D 检测结果 + 对齐后的深度图 + 相机内参，转换为 3D 坐标。

    - 输入:
        /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image, 16UC1 或 32FC1)
        /camera/color/camera_info (sensor_msgs/CameraInfo)
        /color_blobs (vision_msgs/Detection2DArray)

    - 输出:
        /color_blobs_3d (vision_msgs/Detection3DArray)
          其中:
            - pose.position 存储 (x, y, z) 相机坐标系
            - results[*].id 继承自 2D 检测中的 id，例如 "blob:red"
    """

    def __init__(self) -> None:
        super().__init__("blob_depth_to_3d")

        self._bridge = CvBridge()

        # 保存相机内参
        self._camera_info: Optional[CameraInfo] = None
        self._has_camera_info = False
        self._camera_info_received = False

        # 参数：话题名称和 patch 半径
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("blobs_2d_topic", "/color_blobs")
        self.declare_parameter("output_topic", "/color_blobs_3d")
        self.declare_parameter("patch_radius", 2)
        # 深度范围过滤（单位：米），用于增强积木/bin 识别的稳定性
        # 你可以根据实际相机安装和实验场景微调，默认先用 [0.05, 0.60]。
        self.declare_parameter("depth_min_m", 0.05)
        self.declare_parameter("depth_max_m", 0.60)
        # 3D 尺寸阈值（单位：米），用于区分 block vs bin
        # 根据你给的尺寸：积木最大约 0.075m，这里给一点裕量到 0.10m；
        # bin 约 0.20m，这里设为 [0.15, 0.35] 区间。
        self.declare_parameter("block_size_max_m", 0.10)
        self.declare_parameter("bin_size_min_m", 0.15)
        self.declare_parameter("bin_size_max_m", 0.35)

        self._depth_topic = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        self._camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self._blobs_2d_topic = (
            self.get_parameter("blobs_2d_topic").get_parameter_value().string_value
        )
        self._output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self._patch_radius = (
            self.get_parameter("patch_radius").get_parameter_value().integer_value
        )
        self._depth_min_m = (
            self.get_parameter("depth_min_m").get_parameter_value().double_value
        )
        self._depth_max_m = (
            self.get_parameter("depth_max_m").get_parameter_value().double_value
        )
        self._block_size_max_m = (
            self.get_parameter("block_size_max_m").get_parameter_value().double_value
        )
        self._bin_size_min_m = (
            self.get_parameter("bin_size_min_m").get_parameter_value().double_value
        )
        self._bin_size_max_m = (
            self.get_parameter("bin_size_max_m").get_parameter_value().double_value
        )

        # 先只订阅 camera_info，确认收到后再订阅其他话题
        self.get_logger().info(
            f"Step 1: Subscribing to camera_info topic: {self._camera_info_topic}"
        )
        self.get_logger().info(
            "Waiting for camera_info... (will subscribe to depth and detections after receiving it)"
        )
        
        self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self.camera_info_callback,
            10,
        )

        # 延迟创建 depth 和 detection 订阅，等 camera_info 收到后再创建
        self._depth_sub: Optional[Subscriber] = None
        self._det_sub: Optional[Subscriber] = None
        self._ts: Optional[ApproximateTimeSynchronizer] = None

        self._pub = self.create_publisher(
            Detection3DArray,
            self._output_topic,
            10,
        )

        # 添加定时器，定期检查是否收到 camera_info
        self._check_timer = self.create_timer(2.0, self._check_camera_info_status)

    # ------------------------------------------------------------------
    def _check_camera_info_status(self) -> None:
        """定期检查 camera_info 状态，如果长时间没收到则提示用户"""
        if not self._camera_info_received:
            self.get_logger().warn(
                f"⚠ Still waiting for camera_info on: {self._camera_info_topic}\n"
                f"   Please check if the topic exists: ros2 topic list | grep camera_info\n"
                f"   If your RealSense uses /camera/camera/color/camera_info instead,\n"
                f"   restart the node with: --ros-args -p camera_info_topic:=/camera/camera/color/camera_info"
            )

    # ------------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo) -> None:
        """收到 camera_info 时，保存内参并创建其他订阅"""
        if self._camera_info_received:
            # 已经收到过，只更新内参
            self._camera_info = msg
            self._has_camera_info = True
            return

        # 第一次收到 camera_info
        self._camera_info = msg
        self._has_camera_info = True
        self._camera_info_received = True

        # 打印确认信息
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        width = msg.width
        height = msg.height

        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"✓ Successfully received camera_info from: {self._camera_info_topic}"
        )
        self.get_logger().info(
            f"  Camera parameters:\n"
            f"    Resolution: {width}x{height}\n"
            f"    fx={fx:.2f}, fy={fy:.2f}\n"
            f"    cx={cx:.2f}, cy={cy:.2f}"
        )
        self.get_logger().info("=" * 60)

        # 现在创建 depth 和 detection 的订阅
        self.get_logger().info(
            f"Step 2: Now subscribing to:\n"
            f"  - depth_topic: {self._depth_topic}\n"
            f"  - blobs_2d_topic: {self._blobs_2d_topic}"
        )

        depth_sub = Subscriber(
            self,
            Image,
            self._depth_topic,
        )
        det_sub = Subscriber(
            self,
            Detection2DArray,
            self._blobs_2d_topic,
        )

        self._ts = ApproximateTimeSynchronizer(
            [depth_sub, det_sub],
            queue_size=10,
            slop=0.1,
        )
        self._ts.registerCallback(self.synced_callback)

        self._depth_sub = depth_sub
        self._det_sub = det_sub

        self.get_logger().info("✓ All subscriptions created. Ready to process 3D coordinates!")

    # ------------------------------------------------------------------
    def synced_callback(
        self, depth_msg: Image, det_array: Detection2DArray
    ) -> None:
        if not self._has_camera_info or self._camera_info is None:
            self.get_logger().warn_throttle(
                2000, "Waiting for camera_info"
            )
            return

        # 将深度图转为 numpy
        try:
            depth_image = self._bridge.imgmsg_to_cv2(depth_msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        # 相机内参
        fx = float(self._camera_info.k[0])
        fy = float(self._camera_info.k[4])
        cx = float(self._camera_info.k[2])
        cy = float(self._camera_info.k[5])

        det3d_array = Detection3DArray()
        det3d_array.header = depth_msg.header

        for det in det_array.detections:
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y

            if math.isnan(u) or math.isnan(v):
                continue

            # 从 2D bbox 中获取平面角度（如果有），作为抓取的 yaw（rz）
            yaw = 0.0
            try:
                # 假设 bbox.center.theta 以弧度存储
                yaw = float(det.bbox.center.theta)
            except AttributeError:
                yaw = 0.0

            x3d, y3d, z3d = self._project_to_3d(
                depth_image, u, v, fx, fy, cx, cy
            )
            if z3d <= 0.0:
                # 无效深度
                continue

            # ---------------- 深度范围过滤：仅保留 [depth_min_m, depth_max_m] 内的目标 ----------------
            if z3d < float(self._depth_min_m) or z3d > float(self._depth_max_m):
                continue

            # ---------------- 基于 3D 尺度的 block/bin 分类 ----------------
            # 使用 2D bbox 像素宽高 + 深度 + 相机内参，估算 3D 中的大致宽/高（单位：米）。
            ww = float(det.bbox.size_x)
            hh = float(det.bbox.size_y)
            if ww <= 0.0 or hh <= 0.0:
                continue

            width_3d = (ww / fx) * z3d
            height_3d = (hh / fy) * z3d
            size_3d = max(abs(width_3d), abs(height_3d))

            obj_type = "unknown"
            # 积木：最大约 7.5cm，这里给到 0.10m 作为上限
            if size_3d < float(self._block_size_max_m):
                obj_type = "block"
            # bin：约 0.20m，这里给 [0.15, 0.35] 区间
            elif (
                size_3d >= float(self._bin_size_min_m)
                and size_3d <= float(self._bin_size_max_m)
            ):
                obj_type = "bin"
            else:
                # 既不是积木也不是 bin（可能是大白板/背景等），直接忽略
                continue

            # 构造 Detection3D
            d3 = Detection3D()
            d3.header = depth_msg.header

            # 将 3D 点和 yaw 放到 pose 中
            d3.results = []
            for hyp2d in det.results:
                hyp3d = ObjectHypothesisWithPose()

                # Jazzy: id/score 在 hypothesis 子字段里
                # 原始 id 形如 "blob:red"；这里基于 3D 尺度将其改为 "block:red" 或 "bin:red"
                orig_id = hyp2d.hypothesis.class_id
                if ":" in orig_id:
                    _, color = orig_id.split(":", 1)
                else:
                    color = orig_id

                hyp3d.hypothesis.class_id = f"{obj_type}:{color}"
                hyp3d.hypothesis.score = hyp2d.hypothesis.score

                # 3D 点写到 pose（PoseWithCovariance）
                hyp3d.pose.pose.position.x = float(x3d)
                hyp3d.pose.pose.position.y = float(y3d)
                hyp3d.pose.pose.position.z = float(z3d)

                # 将 yaw（rz）编码进四元数（只绕 Z 轴旋转）
                half_yaw = yaw * 0.5
                cyaw = math.cos(half_yaw)
                syaw = math.sin(half_yaw)
                hyp3d.pose.pose.orientation.x = 0.0
                hyp3d.pose.pose.orientation.y = 0.0
                hyp3d.pose.pose.orientation.z = syaw
                hyp3d.pose.pose.orientation.w = cyaw

                d3.results.append(hyp3d)

            det3d_array.detections.append(d3)

        self._pub.publish(det3d_array)

    # ------------------------------------------------------------------
    def _project_to_3d(
        self,
        depth_image: np.ndarray,
        u: float,
        v: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> tuple[float, float, float]:
        """
        根据像素坐标 (u, v) 与深度图、相机内参，计算相机坐标系下的 (X, Y, Z)。
        会在 (u, v) 附近取一个 patch 做中值滤波，减少噪声。
        """
        h, w = depth_image.shape[:2]

        u_int = int(round(u))
        v_int = int(round(v))

        r = max(1, int(self._patch_radius))

        u_min = max(0, u_int - r)
        u_max = min(w - 1, u_int + r)
        v_min = max(0, v_int - r)
        v_max = min(h - 1, v_int + r)

        patch = depth_image[v_min : v_max + 1, u_min : u_max + 1]
        if patch.size == 0:
            return 0.0, 0.0, 0.0

        # 根据编码类型处理深度值
        if patch.dtype == np.uint16:
            # RealSense 常用 16UC1，单位 mm
            valid = patch[patch > 0]
            if valid.size == 0:
                return 0.0, 0.0, 0.0
            depth_m = float(np.median(valid)) / 1000.0
        else:
            # 假设是 32FC1，单位 m
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if valid.size == 0:
                return 0.0, 0.0, 0.0
            depth_m = float(np.median(valid))

        if depth_m <= 0.0:
            return 0.0, 0.0, 0.0

        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return x, y, z


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        node = BlobDepthTo3D()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
