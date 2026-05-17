import math
from typing import Dict, List, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


HSVRange = Tuple[int, int, int, int, int, int]


class ColorBlobDetector(Node):
    """
    纯 HSV 颜色块检测节点：

    - 输入:
        彩色图像: /camera/color/image_raw (sensor_msgs/Image, BGR8)

    - 输出:
        颜色块检测: /color_blobs (vision_msgs/Detection2DArray)

        每个 Detection2D:
          - bbox.center.position.(x, y): 颜色块中心像素坐标 (u, v)
          - bbox.size_(x, y): 外接矩形宽高
          - results[0].id: "blob:<color_name>"，例如 "blob:red"
          - results[0].score: 使用归一化面积作为粗略置信度

    颜色阈值与形态学逻辑参考 tools/fast2vision.py。
    """

    def __init__(self) -> None:
        super().__init__("color_blob_detector")

        # ---------------- 参数 ----------------
        self.declare_parameter(
            "yaml_path",
            "/home/student24/robotproject/tools/color_ranges.yaml",
        )
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("output_topic", "/color_blobs")
        self.declare_parameter("min_area", 1200)
        self.declare_parameter("kernel_size", 5)
        self.declare_parameter("resize_factor", 1.0)  # <1.0 可降分辨率加速
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("roi_x_min", 0.0)
        self.declare_parameter("roi_x_max", 1.0)
        self.declare_parameter("roi_y_min", 0.0)
        self.declare_parameter("roi_y_max", 1.0)

        yaml_path = self.get_parameter("yaml_path").get_parameter_value().string_value
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self._min_area = (
            self.get_parameter("min_area").get_parameter_value().integer_value
        )
        self._kernel_size = (
            self.get_parameter("kernel_size").get_parameter_value().integer_value
        )
        self._resize_factor = (
            self.get_parameter("resize_factor").get_parameter_value().double_value
        )
        self._min_score = (
            self.get_parameter("min_score").get_parameter_value().double_value
        )
        self._roi_x_min = (
            self.get_parameter("roi_x_min").get_parameter_value().double_value
        )
        self._roi_x_max = (
            self.get_parameter("roi_x_max").get_parameter_value().double_value
        )
        self._roi_y_min = (
            self.get_parameter("roi_y_min").get_parameter_value().double_value
        )
        self._roi_y_max = (
            self.get_parameter("roi_y_max").get_parameter_value().double_value
        )

        if self._resize_factor <= 0.0 or math.isinf(self._resize_factor):
            self._resize_factor = 1.0

        # ---------------- 颜色区间加载 ----------------
        self._ranges: Dict[str, List[HSVRange]] = self._load_ranges_from_yaml(
            yaml_path
        )
        if not self._ranges:
            self.get_logger().warn(
                f"No HSV ranges loaded from {yaml_path}, detector will publish nothing."
            )

        self._bridge = CvBridge()
        k = max(3, int(self._kernel_size) | 1)  # 确保为奇数且>=3
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

        # ---------------- ROS 通信 ----------------
        self._sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self._pub = self.create_publisher(
            Detection2DArray,
            output_topic,
            10,
        )

        self.get_logger().info(
            f"color_blob_detector started. yaml={yaml_path}, image_topic={image_topic}, "
            f"output_topic={output_topic}, min_area={self._min_area}, kernel={k}, "
            f"resize_factor={self._resize_factor}, min_score={self._min_score}, "
            f"roi=({self._roi_x_min:.2f}, {self._roi_y_min:.2f})-"
            f"({self._roi_x_max:.2f}, {self._roi_y_max:.2f})"
        )

    # ------------------------------------------------------------------
    def _load_ranges_from_yaml(self, path: str) -> Dict[str, List[HSVRange]]:
        """
        适配你的 YAML 格式：
          red:
            - [h1,s1,v1,h2,s2,v2]
          blue:
            - [h1,s1,v1,h2,s2,v2]
          ...
          color_ratio_threshold: ...  # 忽略
        """
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Failed to load YAML {path}: {e}")
            return {}

        if not isinstance(data, dict):
            return {}

        ranges: Dict[str, List[HSVRange]] = {}

        for k, v in data.items():
            if k == "color_ratio_threshold":
                continue
            if not isinstance(v, list):
                continue

            segs: List[HSVRange] = []
            for item in v:
                if not (isinstance(item, list) and len(item) == 6):
                    continue
                h1, s1, v1, h2, s2, v2 = item
                segs.append(
                    (
                        int(h1),
                        int(s1),
                        int(v1),
                        int(h2),
                        int(s2),
                        int(v2),
                    )
                )

            if segs:
                ranges[str(k)] = segs

        self.get_logger().info(
            f"Loaded colors from YAML: {list(ranges.keys()) if ranges else '[]'}"
        )
        return ranges

    # ------------------------------------------------------------------
    def _build_mask(self, hsv: np.ndarray, segs: List[HSVRange]) -> np.ndarray:
        """多段阈值取并集"""
        mask_all: np.ndarray | None = None
        for (h1, s1, v1, h2, s2, v2) in segs:
            lower = np.array([h1, s1, v1], dtype=np.uint8)
            upper = np.array([h2, s2, v2], dtype=np.uint8)
            m = cv2.inRange(hsv, lower, upper)
            mask_all = m if mask_all is None else cv2.bitwise_or(mask_all, m)
        if mask_all is None:
            mask_all = np.zeros(hsv.shape[:2], dtype=np.uint8)
        return mask_all

    # ------------------------------------------------------------------
    def _angle_from_contour(self, cnt):
        """
        使用最小外接矩形计算轮廓的主方向角度（单位：度，范围 [-90, 90]）。
        该角度可用于平面内的抓取朝向（rz）。
        """
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (w, h), raw_angle = rect
        if w <= 1e-6 or h <= 1e-6:
            return None

        # 统一主轴：让 grasp_angle 表示“长边”的方向
        if w < h:
            grasp_angle = raw_angle
        else:
            grasp_angle = raw_angle - 90.0

        # 归一化到 [-90, 90]
        if grasp_angle < -90.0:
            grasp_angle += 180.0
        elif grasp_angle > 90.0:
            grasp_angle -= 180.0

        return float(grasp_angle)

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        h0, w0 = bgr.shape[:2]
        scale = float(self._resize_factor)
        if scale != 1.0:
            new_w = max(1, int(w0 * scale))
            new_h = max(1, int(h0 * scale))
            bgr_proc = cv2.resize(bgr, (new_w, new_h))
        else:
            bgr_proc = bgr

        hsv = cv2.cvtColor(bgr_proc, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        x_min, x_max, y_min, y_max = self._roi_bounds(w, h)

        det_array = Detection2DArray()
        det_array.header = msg.header

        total_pixels = float(h * w)

        for color_name, segs in self._ranges.items():
            mask = self._build_mask(hsv, segs)
            mask = self._apply_roi(mask, x_min, x_max, y_min, y_max)

            # 形态学去噪：开运算 + 闭运算
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                area = float(cv2.contourArea(cnt))
                if area < float(self._min_area):
                    continue
                score = float(area / total_pixels) if total_pixels > 0 else 0.0
                if score < float(self._min_score):
                    continue

                x, y, ww, hh = cv2.boundingRect(cnt)

                m = cv2.moments(cnt)
                if abs(m["m00"]) > 1e-6:
                    u = m["m10"] / m["m00"]
                    v = m["m01"] / m["m00"]
                else:
                    u = x + ww / 2.0
                    v = y + hh / 2.0

                # 缩放回原图坐标
                if scale != 1.0:
                    u *= 1.0 / scale
                    v *= 1.0 / scale
                    x *= 1.0 / scale
                    y *= 1.0 / scale
                    ww *= 1.0 / scale
                    hh *= 1.0 / scale

                # 计算该颜色块的主方向角度（基于整幅图的 contour）
                angle_deg = self._angle_from_contour(cnt)

                # 构造 Detection2D
                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(u)
                det.bbox.center.position.y = float(v)
                det.bbox.size_x = float(ww)
                det.bbox.size_y = float(hh)

                # 将角度写入 bbox.center.theta（以弧度存储，供后续 3D/机械臂使用）
                if angle_deg is not None:
                    try:
                        det.bbox.center.theta = float(np.deg2rad(angle_deg))
                    except AttributeError:
                        # 某些版本的 BoundingBox2D.center 可能没有 theta 字段，忽略
                        pass

                hyp = ObjectHypothesisWithPose()
                # Jazzy: ObjectHypothesisWithPose 没有 id/score，要写进 hypothesis
                hyp.hypothesis.class_id = f"blob:{color_name}"
                hyp.hypothesis.score = score

                det.results.append(hyp)
                det_array.detections.append(det)

        self._pub.publish(det_array)

    # ------------------------------------------------------------------
    def _roi_bounds(self, width: int, height: int) -> tuple[int, int, int, int]:
        x0 = int(max(0.0, min(1.0, self._roi_x_min)) * width)
        x1 = int(max(0.0, min(1.0, self._roi_x_max)) * width)
        y0 = int(max(0.0, min(1.0, self._roi_y_min)) * height)
        y1 = int(max(0.0, min(1.0, self._roi_y_max)) * height)

        if x1 <= x0:
            x0, x1 = 0, width
        if y1 <= y0:
            y0, y1 = 0, height

        return x0, x1, y0, y1

    # ------------------------------------------------------------------
    def _apply_roi(
        self,
        mask: np.ndarray,
        x_min: int,
        x_max: int,
        y_min: int,
        y_max: int,
    ) -> np.ndarray:
        if (
            x_min <= 0
            and y_min <= 0
            and x_max >= mask.shape[1]
            and y_max >= mask.shape[0]
        ):
            return mask

        roi_mask = np.zeros_like(mask)
        roi_mask[y_min:y_max, x_min:x_max] = mask[y_min:y_max, x_min:x_max]
        return roi_mask


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        node = ColorBlobDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
