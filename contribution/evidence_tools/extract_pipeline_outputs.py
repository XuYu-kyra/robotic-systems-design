import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quat_to_yaw_deg(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(math.degrees(math.atan2(siny_cosp, cosy_cosp)))


def extract_pipeline_outputs(
    bag_path,
    out_json,
    storage_id="mcap",
    blobs_2d_topic="/full/color_blobs",
    blobs_3d_topic="/full/color_blobs_3d",
    camera_info_topic="/camera/camera/color/camera_info",
):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = [blobs_2d_topic, blobs_3d_topic, camera_info_topic]
    missing = [topic for topic in required if topic not in topics]
    if missing:
        raise RuntimeError(f"Missing topics in bag {bag_path}: {missing}")

    msg_classes = {topic: get_message(topics[topic]) for topic in required}

    output = {
        "bag_path": str(bag_path),
        "storage_id": storage_id,
        "topics": {
            "blobs_2d_topic": blobs_2d_topic,
            "blobs_3d_topic": blobs_3d_topic,
            "camera_info_topic": camera_info_topic,
        },
        "camera_info": [],
        "detections_2d": [],
        "detections_3d": [],
    }

    while reader.has_next():
        topic_name, data, _bag_stamp_ns = reader.read_next()
        if topic_name not in msg_classes:
            continue

        msg = deserialize_message(data, msg_classes[topic_name])

        if topic_name == camera_info_topic:
            output["camera_info"].append(
                {
                    "stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec),
                    "frame_id": msg.header.frame_id,
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "k": [float(x) for x in msg.k],
                }
            )
            continue

        if topic_name == blobs_2d_topic:
            det_items = []
            for det in msg.detections:
                class_id = "blob:unknown"
                score = 0.0
                if det.results:
                    hyp = det.results[0]
                    class_id = hyp.hypothesis.class_id
                    score = float(hyp.hypothesis.score)
                det_items.append(
                    {
                        "bbox": {
                            "cx": float(det.bbox.center.position.x),
                            "cy": float(det.bbox.center.position.y),
                            "sx": float(det.bbox.size_x),
                            "sy": float(det.bbox.size_y),
                        },
                        "class_id": class_id,
                        "score": score,
                    }
                )
            output["detections_2d"].append(
                {
                    "stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec),
                    "frame_id": msg.header.frame_id,
                    "detections": det_items,
                }
            )
            continue

        if topic_name == blobs_3d_topic:
            det_items = []
            for det in msg.detections:
                if not det.results:
                    continue
                hyp = det.results[0]
                pose = hyp.pose.pose
                q = pose.orientation
                det_items.append(
                    {
                        "class_id": hyp.hypothesis.class_id,
                        "score": float(hyp.hypothesis.score),
                        "position": {
                            "x": float(pose.position.x),
                            "y": float(pose.position.y),
                            "z": float(pose.position.z),
                        },
                        "orientation": {
                            "x": float(q.x),
                            "y": float(q.y),
                            "z": float(q.z),
                            "w": float(q.w),
                        },
                        "yaw_deg": quat_to_yaw_deg(q.x, q.y, q.z, q.w),
                    }
                )
            output["detections_3d"].append(
                {
                    "stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec),
                    "frame_id": msg.header.frame_id,
                    "detections": det_items,
                }
            )

    Path(out_json).write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(f"Saved extracted pipeline outputs to {out_json}")
    print(f"camera_info records: {len(output['camera_info'])}")
    print(f"2D detection records: {len(output['detections_2d'])}")
    print(f"3D detection records: {len(output['detections_3d'])}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract real pipeline outputs from a recorded ROS2 output bag."
    )
    parser.add_argument("--bag", required=True, help="Path to the recorded output bag directory")
    parser.add_argument("--out-json", required=True, help="Output JSON path")
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--blobs-2d-topic", default="/full/color_blobs")
    parser.add_argument("--blobs-3d-topic", default="/full/color_blobs_3d")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    return parser.parse_args()


def main():
    args = parse_args()
    extract_pipeline_outputs(
        bag_path=args.bag,
        out_json=args.out_json,
        storage_id=args.storage_id,
        blobs_2d_topic=args.blobs_2d_topic,
        blobs_3d_topic=args.blobs_3d_topic,
        camera_info_topic=args.camera_info_topic,
    )


if __name__ == "__main__":
    main()
