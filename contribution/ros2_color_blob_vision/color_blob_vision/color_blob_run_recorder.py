import json
import os
import time
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray


class ColorBlobRunRecorder(Node):
    """Record debug images, timestamps, and 3D detections for offline rendering."""

    def __init__(self) -> None:
        super().__init__("color_blob_run_recorder")

        self.declare_parameter("image_topic", "/color_blobs/debug_image")
        self.declare_parameter("white_3d_topic", "/white/color_blobs_3d")
        self.declare_parameter("full_3d_topic", "/full/color_blobs_3d")
        self.declare_parameter("output_dir", "/tmp/color_blob_runs")
        self.declare_parameter("run_name", "")
        self.declare_parameter("video_fps", 30.0)
        self.declare_parameter("write_video", False)
        self.declare_parameter("save_frames", True)
        self.declare_parameter("image_ext", "jpg")

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._white_3d_topic = str(self.get_parameter("white_3d_topic").value)
        self._full_3d_topic = str(self.get_parameter("full_3d_topic").value)
        self._output_dir = str(self.get_parameter("output_dir").value)
        self._run_name = str(self.get_parameter("run_name").value)
        self._video_fps = float(self.get_parameter("video_fps").value)
        self._write_video = bool(self.get_parameter("write_video").value)
        self._save_frames = bool(self.get_parameter("save_frames").value)
        self._image_ext = str(self.get_parameter("image_ext").value).lower()
        if self._image_ext not in {"jpg", "png"}:
            self._image_ext = "jpg"

        if not self._run_name:
            self._run_name = time.strftime("%Y%m%d_%H%M%S")

        self._run_dir = os.path.join(self._output_dir, self._run_name)
        os.makedirs(self._run_dir, exist_ok=True)
        self._frames_dir = os.path.join(self._run_dir, "debug_frames")
        if self._save_frames:
            os.makedirs(self._frames_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_path = os.path.join(self._run_dir, "debug_image.mp4")
        self._jsonl_path = os.path.join(self._run_dir, "detections_3d.jsonl")
        self._frames_jsonl_path = os.path.join(self._run_dir, "debug_frames.jsonl")
        self._jsonl_file = open(self._jsonl_path, "a", encoding="utf-8")
        self._frames_jsonl_file = open(self._frames_jsonl_path, "a", encoding="utf-8")
        self._frame_count = 0

        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._image_cb,
            10,
        )
        self._white_sub = self.create_subscription(
            Detection3DArray,
            self._white_3d_topic,
            lambda msg: self._det3d_cb(msg, "white"),
            10,
        )
        self._full_sub = self.create_subscription(
            Detection3DArray,
            self._full_3d_topic,
            lambda msg: self._det3d_cb(msg, "full"),
            10,
        )

        self.get_logger().info(
            "color_blob_run_recorder started.\n"
            f"  image_topic: {self._image_topic}\n"
            f"  white_3d_topic: {self._white_3d_topic}\n"
            f"  full_3d_topic: {self._full_3d_topic}\n"
            f"  write_video: {self._write_video}\n"
            f"  save_frames: {self._save_frames}\n"
            f"  frames_dir: {self._frames_dir}\n"
            f"  frames_jsonl: {self._frames_jsonl_path}\n"
            f"  video: {self._video_path}\n"
            f"  detections: {self._jsonl_path}"
        )

    def _image_cb(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert debug image: {e}")
            return

        if self._write_video and self._video_writer is None:
            h, w = bgr.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            fps = self._video_fps if self._video_fps > 0.0 else 30.0
            self._video_writer = cv2.VideoWriter(
                self._video_path,
                fourcc,
                fps,
                (w, h),
            )
            if not self._video_writer.isOpened():
                self.get_logger().error(f"Failed to open video writer: {self._video_path}")
                self._video_writer = None
                return

        if self._video_writer is not None:
            self._video_writer.write(bgr)

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        frame_name = f"frame_{self._frame_count:06d}.{self._image_ext}"

        if self._save_frames:
            frame_path = os.path.join(self._frames_dir, frame_name)
            if self._image_ext == "png":
                cv2.imwrite(frame_path, bgr)
            else:
                cv2.imwrite(frame_path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])

        frame_record = {
            "frame_idx": self._frame_count,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "stamp_ns": stamp_ns,
            "frame_id": msg.header.frame_id,
            "image_file": frame_name,
            "height": int(bgr.shape[0]),
            "width": int(bgr.shape[1]),
        }
        self._frames_jsonl_file.write(json.dumps(frame_record, sort_keys=True) + "\n")
        self._frames_jsonl_file.flush()
        self._frame_count += 1

    def _det3d_cb(self, msg: Detection3DArray, stream: str) -> None:
        record = {
            "stream": stream,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "frame_id": msg.header.frame_id,
            "detections": [],
        }

        for det in msg.detections:
            for hyp in det.results:
                pose = hyp.pose.pose
                record["detections"].append(
                    {
                        "class_id": hyp.hypothesis.class_id,
                        "score": float(hyp.hypothesis.score),
                        "position": {
                            "x": float(pose.position.x),
                            "y": float(pose.position.y),
                            "z": float(pose.position.z),
                        },
                        "orientation": {
                            "x": float(pose.orientation.x),
                            "y": float(pose.orientation.y),
                            "z": float(pose.orientation.z),
                            "w": float(pose.orientation.w),
                        },
                    }
                )

        self._jsonl_file.write(json.dumps(record, sort_keys=True) + "\n")
        self._jsonl_file.flush()

    def destroy_node(self) -> bool:
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
        if not self._jsonl_file.closed:
            self._jsonl_file.close()
        if not self._frames_jsonl_file.closed:
            self._frames_jsonl_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorBlobRunRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
