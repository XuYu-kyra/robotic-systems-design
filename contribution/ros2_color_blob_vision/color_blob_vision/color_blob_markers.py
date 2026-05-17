import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class ColorBlobMarkers(Node):
    def __init__(self):
        super().__init__("color_blob_markers")

        self.declare_parameter("input_topic", "/color_blobs_3d")
        self.declare_parameter("output_topic", "/color_blobs_markers")
        self.declare_parameter("marker_scale", 0.03)  # 球半径(米)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._scale = float(self.get_parameter("marker_scale").value)

        self._pub = self.create_publisher(MarkerArray, out_topic, 10)
        self._sub = self.create_subscription(Detection3DArray, in_topic, self.cb, 10)

        self.get_logger().info(f"markers: {in_topic} -> {out_topic}")

    def cb(self, msg: Detection3DArray):
        ma = MarkerArray()
        frame_id = msg.header.frame_id if msg.header.frame_id else "camera_color_optical_frame"

        # 清空旧 marker（用 DELETEALL）
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        mid = 0
        for det in msg.detections:
            for hyp in det.results:
                class_id = hyp.hypothesis.class_id  # "blob:yellow"
                label = class_id.split(":", 1)[1] if ":" in class_id else class_id

                x = hyp.pose.pose.position.x
                y = hyp.pose.pose.position.y
                z = hyp.pose.pose.position.z

                # 球
                m = Marker()
                m.header.frame_id = frame_id
                m.header.stamp = msg.header.stamp
                m.ns = "color_blobs"
                m.id = mid
                mid += 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(x)
                m.pose.position.y = float(y)
                m.pose.position.z = float(z)
                m.pose.orientation.w = 1.0
                m.scale.x = self._scale
                m.scale.y = self._scale
                m.scale.z = self._scale
                m.color.a = 1.0
                # 不指定颜色也能看，但你可以按 label 着色（可选）
                ma.markers.append(m)

                # 文字标签
                t = Marker()
                t.header.frame_id = frame_id
                t.header.stamp = msg.header.stamp
                t.ns = "color_blobs_text"
                t.id = mid
                mid += 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = float(x)
                t.pose.position.y = float(y)
                t.pose.position.z = float(z + 0.04)
                t.pose.orientation.w = 1.0
                t.scale.z = 0.04
                t.color.a = 1.0
                t.text = f"{label}\n({x:.2f},{y:.2f},{z:.2f})"
                ma.markers.append(t)

        self._pub.publish(ma)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ColorBlobMarkers()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()