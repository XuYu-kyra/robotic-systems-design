import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray


class ColorBlobSummary(Node):
    """
    ROS2 节点：订阅 3D 检测结果，汇总并输出格式化的坐标+颜色信息。

    - 输入:
        /color_blobs_3d (vision_msgs/Detection3DArray)

    - 输出:
        在终端打印每个检测到的颜色块的：
          - 颜色（从 id 中解析，例如 "blob:red" -> "red"）
          - 3D 坐标 (x, y, z) 单位：米
          - 置信度 score

    也可以发布到自定义话题（可选，当前版本只打印日志）。
    """

    def __init__(self) -> None:
        super().__init__("color_blob_summary")

        self.declare_parameter("input_topic", "/color_blobs_3d")
        self.declare_parameter("print_interval", 1.0)  # 打印间隔（秒）

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self._print_interval = (
            self.get_parameter("print_interval").get_parameter_value().double_value
        )

        self._last_print_time = 0.0

        self._sub = self.create_subscription(
            Detection3DArray,
            input_topic,
            self.detection_callback,
            10,
        )

        self.get_logger().info(
            f"color_blob_summary started. Subscribing to {input_topic}, "
            f"print_interval={self._print_interval}s"
        )

    # ------------------------------------------------------------------
    def detection_callback(self, msg: Detection3DArray) -> None:
        """
        处理 3D 检测结果，提取颜色和坐标信息并打印。
        """
        import time

        current_time = time.time()
        if current_time - self._last_print_time < self._print_interval:
            return
        self._last_print_time = current_time

        if not msg.detections:
            self.get_logger().debug("No detections in this frame")
            return

        # 汇总所有检测结果
        summaries = []
        for det in msg.detections:
            if not det.results:
                continue

            for hyp in det.results:
                class_id = hyp.hypothesis.class_id
                obj_type, color = self._split_class_id(class_id)

                x = hyp.pose.pose.position.x
                y = hyp.pose.pose.position.y
                z = hyp.pose.pose.position.z
                score = hyp.hypothesis.score

                import math
                qx = hyp.pose.pose.orientation.x
                qy = hyp.pose.pose.orientation.y
                qz = hyp.pose.pose.orientation.z
                qw = hyp.pose.pose.orientation.w

                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                yaw_deg = math.degrees(yaw_rad)

                summaries.append(
                    {
                        "class_id": class_id,
                        "type": obj_type,
                        "color": color,
                        "x": x,
                        "y": y,
                        "z": z,
                        "yaw_deg": yaw_deg,
                        "score": score,
                    }
                )

        # 打印汇总信息
        if summaries:
            self.get_logger().info("=" * 80)
            self.get_logger().info(f"Detected {len(summaries)} object(s):")
            for i, s in enumerate(summaries, 1):
                self.get_logger().info(
                    f"[{i}] class={s['class_id']:12s} | "
                    f"type={s['type']:5s} | color={s['color']:6s} | "
                    f"pos=({s['x']:.3f}, {s['y']:.3f}, {s['z']:.3f}) m | "
                    f"rz={s['yaw_deg']:.1f} deg | "
                    f"score={s['score']:.4f}"
                )
            self.get_logger().info("=" * 80)

    # ------------------------------------------------------------------
    def _split_class_id(self, class_id: str):
        """
        "block:red" -> ("block", "red")
        "bin:white" -> ("bin", "white")
        "blob:red" -> ("blob", "red")
        """
        if ":" in class_id:
            obj_type, color = class_id.split(":", 1)
            return obj_type, color
        return "unknown", class_id


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        node = ColorBlobSummary()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()

