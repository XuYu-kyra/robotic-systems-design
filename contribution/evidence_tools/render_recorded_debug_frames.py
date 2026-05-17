import argparse
import json
from pathlib import Path

import cv2

from rosbag_video_utils import open_video_writer


def load_frame_records(run_dir):
    run_dir = Path(run_dir)
    records_path = run_dir / "debug_frames.jsonl"
    if not records_path.exists():
        raise RuntimeError(f"Missing frame timestamp file: {records_path}")

    records = []
    with open(records_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))

    if not records:
        raise RuntimeError(f"No frame records found in {records_path}")
    return run_dir, records


def render_timed_video(run_dir, out_video, fps=10.0, hold_last_sec=0.3):
    run_dir, records = load_frame_records(run_dir)
    frames_dir = run_dir / "debug_frames"

    first_frame = cv2.imread(str(frames_dir / records[0]["image_file"]))
    if first_frame is None:
        raise RuntimeError("Failed to load the first saved debug frame.")

    writer = open_video_writer(
        out_video,
        fps=float(fps),
        width=first_frame.shape[1],
        height=first_frame.shape[0],
    )

    min_frames_per_image = 1
    target_fps = float(fps)

    for idx, rec in enumerate(records):
        frame_path = frames_dir / rec["image_file"]
        image = cv2.imread(str(frame_path))
        if image is None:
            continue

        if idx + 1 < len(records):
            next_rec = records[idx + 1]
            dt_sec = (int(next_rec["stamp_ns"]) - int(rec["stamp_ns"])) / 1e9
            repeat_count = max(min_frames_per_image, int(round(dt_sec * target_fps)))
        else:
            repeat_count = max(min_frames_per_image, int(round(hold_last_sec * target_fps)))

        for _ in range(repeat_count):
            writer.write(image)

    writer.release()
    print(f"Saved timestamp-aligned debug video to {out_video}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Render a timestamp-aligned mp4 from recorder-saved debug frames."
    )
    parser.add_argument("--run-dir", required=True, help="Recorder output directory containing debug_frames/")
    parser.add_argument("--out-video", required=True, help="Output mp4 path")
    parser.add_argument("--fps", type=float, default=10.0, help="Output video fps")
    parser.add_argument(
        "--hold-last-sec",
        type=float,
        default=0.3,
        help="How long to hold the last frame at the end of the video.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    render_timed_video(
        run_dir=args.run_dir,
        out_video=args.out_video,
        fps=args.fps,
        hold_last_sec=args.hold_last_sec,
    )


if __name__ == "__main__":
    main()
