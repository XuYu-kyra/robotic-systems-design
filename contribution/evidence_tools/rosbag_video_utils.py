import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def ensure_dir(path):
    Path(path).mkdir(parents=True, exist_ok=True)


def decode_image_message(msg, msg_type):
    if msg_type == "sensor_msgs/msg/CompressedImage":
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("Failed to decode CompressedImage payload.")
        return image

    encoding = getattr(msg, "encoding", "")
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in {"bgr8", "rgb8"}:
        image = data.reshape((height, step // 3, 3))[:, :width, :]
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)

    if encoding in {"bgra8", "rgba8"}:
        image = data.reshape((height, step // 4, 4))[:, :width, :]
        if encoding == "rgba8":
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return np.ascontiguousarray(image)

    if encoding == "mono8":
        image = data.reshape((height, step))[:, :width]
        return cv2.cvtColor(np.ascontiguousarray(image), cv2.COLOR_GRAY2BGR)

    raise ValueError(f"Unsupported image encoding: {encoding}")


def iter_rosbag_image_frames(
    bag_path,
    topic="/camera/color/image_raw",
    storage_id="sqlite3",
    start_sec=0.0,
    duration_sec=-1.0,
):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise RuntimeError(f"Topic {topic} was not found in bag {bag_path}.")

    msg_type = topics[topic]
    msg_cls = get_message(msg_type)

    first_topic_ns = None
    end_sec = None if duration_sec is None or duration_sec < 0 else start_sec + duration_sec
    frame_idx = 0

    while reader.has_next():
        topic_name, data, stamp_ns = reader.read_next()
        if topic_name != topic:
            continue

        if first_topic_ns is None:
            first_topic_ns = stamp_ns

        rel_sec = (stamp_ns - first_topic_ns) / 1e9
        if rel_sec < float(start_sec):
            continue
        if end_sec is not None and rel_sec > float(end_sec):
            break

        msg = deserialize_message(data, msg_cls)
        image = decode_image_message(msg, msg_type)

        yield {
            "frame_idx": frame_idx,
            "stamp_ns": int(stamp_ns),
            "rel_sec": float(rel_sec),
            "image": image,
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
        }
        frame_idx += 1


def save_jsonl(path, records):
    ensure_dir(Path(path).parent)
    with open(path, "w", encoding="utf-8") as f:
        for record in records:
            f.write(json.dumps(record, ensure_ascii=True) + "\n")


def load_jsonl(path):
    records = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def open_video_writer(path, fps, width, height):
    ensure_dir(Path(path).parent)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, float(fps), (int(width), int(height)))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {path}.")
    return writer


def draw_text_block(
    image,
    lines,
    origin=(20, 30),
    line_height=28,
    text_color=(255, 255, 255),
    bg_color=(0, 0, 0),
    alpha=0.55,
    font_scale=0.7,
    thickness=2,
):
    if not lines:
        return image

    overlay = image.copy()
    x, y = origin
    max_width = 0
    for line in lines:
        (w, _), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        max_width = max(max_width, w)
    box_w = max_width + 20
    box_h = line_height * len(lines) + 10
    cv2.rectangle(overlay, (x - 10, y - 22), (x - 10 + box_w, y - 22 + box_h), bg_color, -1)
    cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0, image)

    yy = y
    for line in lines:
        cv2.putText(
            image,
            line,
            (x, yy),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            text_color,
            thickness,
            cv2.LINE_AA,
        )
        yy += line_height
    return image


def fit_text_lines(summary_lines, max_width=60):
    lines = []
    for line in summary_lines:
        if len(line) <= max_width:
            lines.append(line)
            continue
        words = line.split()
        current = []
        current_len = 0
        for word in words:
            candidate_len = current_len + len(word) + (1 if current else 0)
            if candidate_len > max_width and current:
                lines.append(" ".join(current))
                current = [word]
                current_len = len(word)
            else:
                current.append(word)
                current_len = candidate_len
        if current:
            lines.append(" ".join(current))
    return lines
