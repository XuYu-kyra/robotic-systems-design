import argparse
import json
from pathlib import Path

import cv2

from rosbag_video_utils import ensure_dir, iter_rosbag_image_frames, open_video_writer


def export_rosbag_video(
    bag_path,
    out_dir,
    topic="/camera/color/image_raw",
    storage_id="sqlite3",
    start_sec=0.0,
    duration_sec=-1.0,
    sample_fps=10.0,
    write_frames=True,
    write_video=True,
):
    out_dir = Path(out_dir)
    frames_dir = out_dir / "frames"
    ensure_dir(out_dir)
    if write_frames:
        ensure_dir(frames_dir)

    manifest = {
        "bag_path": str(bag_path),
        "topic": topic,
        "storage_id": storage_id,
        "start_sec": float(start_sec),
        "duration_sec": float(duration_sec),
        "sample_fps": float(sample_fps),
        "frames": [],
    }

    min_interval = 1.0 / max(0.1, float(sample_fps))
    last_saved_rel = -1e9
    writer = None
    saved_count = 0

    for frame in iter_rosbag_image_frames(
        bag_path=bag_path,
        topic=topic,
        storage_id=storage_id,
        start_sec=start_sec,
        duration_sec=duration_sec,
    ):
        if (frame["rel_sec"] - last_saved_rel) < min_interval:
            continue

        image = frame["image"]
        frame_name = f"frame_{saved_count:06d}.jpg"
        frame_path = frames_dir / frame_name
        rel_path = str(frame_path.relative_to(out_dir))

        if write_frames:
            cv2.imwrite(str(frame_path), image, [cv2.IMWRITE_JPEG_QUALITY, 95])

        if write_video:
            if writer is None:
                writer = open_video_writer(
                    out_dir / "raw_video.mp4",
                    fps=sample_fps,
                    width=image.shape[1],
                    height=image.shape[0],
                )
            writer.write(image)

        manifest["frames"].append(
            {
                "frame_idx": saved_count,
                "stamp_ns": int(frame["stamp_ns"]),
                "rel_sec": float(frame["rel_sec"]),
                "width": int(frame["width"]),
                "height": int(frame["height"]),
                "image_path": rel_path,
            }
        )

        last_saved_rel = frame["rel_sec"]
        saved_count += 1

    if writer is not None:
        writer.release()

    manifest["frame_count"] = saved_count
    with open(out_dir / "manifest.json", "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    print(f"Exported {saved_count} frames to {out_dir}")
    print(f"Manifest: {out_dir / 'manifest.json'}")
    if write_video:
        print(f"Raw video: {out_dir / 'raw_video.mp4'}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export a sampled color-video segment from a ROS2 bag for offline evidence rendering."
    )
    parser.add_argument("--bag", required=True, help="Path to the ROS2 bag directory.")
    parser.add_argument("--out-dir", required=True, help="Output directory for frames, video, and manifest.")
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--start-sec", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float, default=-1.0)
    parser.add_argument("--sample-fps", type=float, default=10.0)
    parser.add_argument("--no-frames", action="store_true", help="Skip writing individual JPEG frames.")
    parser.add_argument("--no-video", action="store_true", help="Skip writing raw_video.mp4.")
    return parser.parse_args()


def main():
    args = parse_args()
    export_rosbag_video(
        bag_path=args.bag,
        out_dir=args.out_dir,
        topic=args.topic,
        storage_id=args.storage_id,
        start_sec=args.start_sec,
        duration_sec=args.duration_sec,
        sample_fps=args.sample_fps,
        write_frames=not args.no_frames,
        write_video=not args.no_video,
    )


if __name__ == "__main__":
    main()
