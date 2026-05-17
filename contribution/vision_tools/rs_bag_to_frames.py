import argparse
import os
from pathlib import Path
import time

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except Exception as e:
    raise RuntimeError("请先安装 pyrealsense2，例如: pip install pyrealsense2") from e


def export_color_frames(bag_path: str, out_root: str, target_fps: float = 1.0, start: float = 0.0, duration: float = -1.0,
                        class_name: str = "", scene: str = ""):
    bag_path = str(bag_path)
    out_dir = Path(out_root)
    if class_name:
        out_dir = out_dir / class_name
    if scene:
        out_dir = out_dir / scene
    out_dir.mkdir(parents=True, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(bag_path, repeat_playback=False)

    try:
        config.enable_all_streams()
    except Exception:
        pass

    profile = pipeline.start(config)
    playback = profile.get_device().as_playback()
    playback.set_real_time(False)

    start_ts = start if start >= 0 else 0.0
    end_ts = None if duration < 0 else start_ts + duration

    last_save_time = -1e9
    min_interval = 1.0 / max(0.1, target_fps)
    saved = 0

    if start_ts > 0:
        try:
            playback.seek(rs.time_point(start_ts))
        except Exception:
            pass

    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
        except Exception:
            break
        if not frames:
            break
        now_s = frames.get_timestamp() / 1000.0
        if end_ts is not None and now_s > end_ts:
            break

        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        if now_s < start_ts:
            continue
        if (now_s - last_save_time) < min_interval:
            continue

        img = np.asanyarray(color_frame.get_data())
        fmt = color_frame.get_profile().format()
        if fmt == rs.format.rgb8:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        out_path = out_dir / f"frame_{int(now_s*1000):013d}.jpg"
        cv2.imwrite(str(out_path), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        last_save_time = now_s
        saved += 1

    pipeline.stop()
    print(f"导出完成: {bag_path} -> {out_dir}，保存 {saved} 张彩色帧")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="输入 .bag 文件路径")
    ap.add_argument("--out", default="/home/student24/robotproject/datasets/shapes/images_all", help="输出根目录（默认 images_all）")
    ap.add_argument("--class_name", default="", help="形状类别名（如 cube/rectangle_prism/triangle_prism/cylinder/arch）")
    ap.add_argument("--scene", default="", help="场景名（如 scene1/bright/tableA 等）")
    ap.add_argument("--fps", type=float, default=1.0, help="目标导出帧率（降采样，默认1.0）")
    ap.add_argument("--start", type=float, default=0.0, help="起始时间(秒)")
    ap.add_argument("--duration", type=float, default=-1.0, help="持续时长(秒)，-1 表示到文件结束")
    args = ap.parse_args()

    export_color_frames(
        bag_path=args.bag,
        out_root=args.out,
        target_fps=args.fps,
        start=args.start,
        duration=args.duration,
        class_name=args.class_name,
        scene=args.scene,
    )
