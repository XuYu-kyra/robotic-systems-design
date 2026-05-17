import argparse
import json
from pathlib import Path

import cv2

from shape_evaluate import (
    contour_features,
    detect_color_blobs,
    load_hsv_ranges,
    load_split,
    train_shape_classifier,
)


def analyze_shape_manifest(
    manifest_path,
    out_json,
    hsv_cfg,
    train_image_dir,
    train_label_dir,
    kernel_size=5,
    detect_min_area=1200.0,
    roi_min_area=50.0,
):
    manifest_path = Path(manifest_path)
    root_dir = manifest_path.parent
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    hsv_ranges = load_hsv_ranges(hsv_cfg)

    train_samples = load_split(train_image_dir, train_label_dir)
    clf, train_count = train_shape_classifier(
        train_samples=train_samples,
        hsv_ranges=hsv_ranges,
        kernel_size=kernel_size,
        roi_min_area=roi_min_area,
    )

    frames = []
    shape_counts = {}

    for frame_info in manifest["frames"]:
        image_path = root_dir / frame_info["image_path"]
        image = cv2.imread(str(image_path))
        if image is None:
            continue

        detections = detect_color_blobs(
            image=image,
            hsv_ranges=hsv_ranges,
            min_area=detect_min_area,
            kernel_size=kernel_size,
        )

        det_items = []
        for det in detections:
            feat = contour_features(det["contour"])
            pred_shape = clf.predict([feat])[0]
            shape_counts[pred_shape] = shape_counts.get(pred_shape, 0) + 1

            x1, y1, x2, y2 = det["box"]
            contour_pts = det["contour"].reshape(-1, 2).tolist()

            det_items.append(
                {
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "color": det["color"],
                    "shape": pred_shape,
                    "area": float(det["area"]),
                    "contour": contour_pts,
                    "features": {
                        "aspect_ratio": float(feat[0]),
                        "extent": float(feat[1]),
                        "solidity": float(feat[2]),
                        "circularity": float(feat[3]),
                        "verts_04": int(round(feat[4])),
                        "verts_02": int(round(feat[5])),
                    },
                }
            )

        frames.append(
            {
                "frame_idx": int(frame_info["frame_idx"]),
                "rel_sec": float(frame_info["rel_sec"]),
                "image_path": frame_info["image_path"],
                "mode": "shape_iou",
                "detections": det_items,
            }
        )

    output = {
        "mode": "shape_iou",
        "hsv_cfg": str(hsv_cfg),
        "training_contours": int(train_count),
        "shape_counts": shape_counts,
        "frame_count": len(frames),
        "frames": frames,
    }
    Path(out_json).write_text(json.dumps(output, indent=2), encoding="utf-8")

    print(f"Saved shape analysis to {out_json}")
    print(f"Frames analyzed: {len(frames)}")
    print(f"Training contours used: {train_count}")
    print(f"Detection counts by predicted shape: {shape_counts}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run contour-based shape analysis on exported ROS2-bag frames."
    )
    parser.add_argument("--manifest", required=True, help="Manifest JSON from export_rosbag_video.py")
    parser.add_argument("--out-json", required=True, help="Output analysis JSON file")
    parser.add_argument("--hsv-cfg", default="/home/student24/robotproject/tools/color_ranges.yaml")
    parser.add_argument(
        "--train-image-dir",
        default="/home/student24/robotproject/datasets/shapes/cube_seperated_dataset/images/train",
    )
    parser.add_argument(
        "--train-label-dir",
        default="/home/student24/robotproject/datasets/shapes/cube_seperated_dataset/labels/train",
    )
    parser.add_argument("--kernel-size", type=int, default=5)
    parser.add_argument("--detect-min-area", type=float, default=1200.0)
    parser.add_argument("--roi-min-area", type=float, default=50.0)
    return parser.parse_args()


def main():
    args = parse_args()
    analyze_shape_manifest(
        manifest_path=args.manifest,
        out_json=args.out_json,
        hsv_cfg=args.hsv_cfg,
        train_image_dir=args.train_image_dir,
        train_label_dir=args.train_label_dir,
        kernel_size=args.kernel_size,
        detect_min_area=args.detect_min_area,
        roi_min_area=args.roi_min_area,
    )


if __name__ == "__main__":
    main()
