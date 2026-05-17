import argparse
import json
from pathlib import Path

import cv2

from color_evaluate import detect_color_blobs, load_hsv_ranges


def analyze_color_manifest(
    manifest_path,
    out_json,
    hsv_cfg,
    min_area=1200.0,
    kernel_size=5,
    resize_factor=1.0,
    min_score=0.0,
    kind="block",
):
    manifest_path = Path(manifest_path)
    root_dir = manifest_path.parent
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    hsv_ranges = load_hsv_ranges(hsv_cfg)

    records = []
    color_counts = {}

    for frame_info in manifest["frames"]:
        image_path = root_dir / frame_info["image_path"]
        image = cv2.imread(str(image_path))
        if image is None:
            continue

        detections = detect_color_blobs(
            image=image,
            hsv_ranges=hsv_ranges,
            min_area=min_area,
            kernel_size=kernel_size,
            resize_factor=resize_factor,
            min_score=min_score,
        )

        det_items = []
        for det in detections:
            x1, y1, x2, y2 = det["box"]
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            color_name = det["color"]
            color_counts[color_name] = color_counts.get(color_name, 0) + 1
            det_items.append(
                {
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "center": [float(cx), float(cy)],
                    "color": color_name,
                    "score": float(det["score"]),
                    "kind": kind,
                }
            )

        records.append(
            {
                "frame_idx": int(frame_info["frame_idx"]),
                "rel_sec": float(frame_info["rel_sec"]),
                "image_path": frame_info["image_path"],
                "mode": "color",
                "kind": kind,
                "detections": det_items,
            }
        )

    output = {
        "mode": "color",
        "kind": kind,
        "hsv_cfg": str(hsv_cfg),
        "frame_count": len(records),
        "color_counts": color_counts,
        "frames": records,
    }
    Path(out_json).write_text(json.dumps(output, indent=2), encoding="utf-8")

    print(f"Saved color analysis to {out_json}")
    print(f"Frames analyzed: {len(records)}")
    print(f"Detection counts by color: {color_counts}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the HSV/blob detector on exported ROS2-bag frames for block/bin-color overlays."
    )
    parser.add_argument("--manifest", required=True, help="Manifest JSON from export_rosbag_video.py")
    parser.add_argument("--out-json", required=True, help="Output analysis JSON file")
    parser.add_argument("--hsv-cfg", default="/home/student24/robotproject/tools/color_ranges.yaml")
    parser.add_argument("--min-area", type=float, default=1200.0)
    parser.add_argument("--kernel-size", type=int, default=5)
    parser.add_argument("--resize-factor", type=float, default=1.0)
    parser.add_argument("--min-score", type=float, default=0.0)
    parser.add_argument("--kind", choices=["block", "bin"], default="block")
    return parser.parse_args()


def main():
    args = parse_args()
    analyze_color_manifest(
        manifest_path=args.manifest,
        out_json=args.out_json,
        hsv_cfg=args.hsv_cfg,
        min_area=args.min_area,
        kernel_size=args.kernel_size,
        resize_factor=args.resize_factor,
        min_score=args.min_score,
        kind=args.kind,
    )


if __name__ == "__main__":
    main()
