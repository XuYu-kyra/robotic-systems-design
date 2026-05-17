import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report


CLASS_NAMES = {
    0: "cube",
    1: "rectangle_prism",
    2: "triangle_prism",
    3: "cylinder",
    4: "arch",
}


def load_hsv_ranges(yaml_path):
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    ranges = {}
    for color_name, value in data.items():
        if color_name == "color_ratio_threshold":
            continue
        if color_name not in {"red", "blue", "yellow"}:
            continue
        if not isinstance(value, list):
            continue

        segs = []
        for item in value:
            if isinstance(item, list) and len(item) == 6:
                segs.append(tuple(map(int, item)))
        if segs:
            ranges[color_name] = segs
    return ranges


def build_mask(hsv, segs):
    mask_all = None
    for h1, s1, v1, h2, s2, v2 in segs:
        lower = np.array([h1, s1, v1], dtype=np.uint8)
        upper = np.array([h2, s2, v2], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask_all = mask if mask_all is None else cv2.bitwise_or(mask_all, mask)
    if mask_all is None:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)
    return mask_all


def detect_color_blobs(image, hsv_ranges, min_area, kernel_size):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (max(3, int(kernel_size) | 1), max(3, int(kernel_size) | 1))
    )

    detections = []
    for color_name, segs in hsv_ranges.items():
        mask = build_mask(hsv, segs)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < float(min_area):
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            detections.append(
                {
                    "color": color_name,
                    "area": area,
                    "box": (x, y, x + w, y + h),
                    "contour": cnt,
                }
            )
    return detections


def contour_features(cnt):
    area = float(cv2.contourArea(cnt))
    perimeter = float(cv2.arcLength(cnt, True))
    x, y, w, h = cv2.boundingRect(cnt)

    hull = cv2.convexHull(cnt)
    hull_area = float(cv2.contourArea(hull)) if hull is not None else 0.0
    rect_area = float(w * h) if w > 0 and h > 0 else 0.0

    extent = area / rect_area if rect_area > 0 else 0.0
    solidity = area / hull_area if hull_area > 0 else 0.0
    aspect_ratio = max(w / h, h / w) if w > 0 and h > 0 else 0.0
    circularity = 4.0 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0

    approx_04 = cv2.approxPolyDP(cnt, 0.04 * perimeter, True)
    approx_02 = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)

    hu = cv2.HuMoments(cv2.moments(cnt)).flatten()
    hu_log = []
    for value in hu[:7]:
        if abs(value) <= 1e-12:
            hu_log.append(0.0)
        else:
            hu_log.append(-math.copysign(1.0, value) * math.log10(abs(value)))

    return np.array(
        [
            aspect_ratio,
            extent,
            solidity,
            circularity,
            float(len(approx_04)),
            float(len(approx_02)),
            area,
            perimeter,
            *hu_log,
        ],
        dtype=float,
    )


def iou(box1, box2):
    xx1 = max(box1[0], box2[0])
    yy1 = max(box1[1], box2[1])
    xx2 = min(box1[2], box2[2])
    yy2 = min(box1[3], box2[3])
    inter = max(0, xx2 - xx1) * max(0, yy2 - yy1)
    area1 = max(0, box1[2] - box1[0]) * max(0, box1[3] - box1[1])
    area2 = max(0, box2[2] - box2[0]) * max(0, box2[3] - box2[1])
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0.0


def load_split(image_dir, label_dir):
    samples = []
    for label_path in sorted(Path(label_dir).glob("*.txt")):
        image_path = Path(image_dir) / f"{label_path.stem}.jpg"
        if not image_path.exists():
            continue

        image = cv2.imread(str(image_path))
        if image is None:
            continue

        h, w = image.shape[:2]
        gt_items = []
        for line in label_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            parts = line.split()
            cls_id = int(parts[0])
            xc, yc, bw, bh = map(float, parts[1:5])
            x1 = max(0, int((xc - bw / 2.0) * w))
            y1 = max(0, int((yc - bh / 2.0) * h))
            x2 = min(w, int((xc + bw / 2.0) * w))
            y2 = min(h, int((yc + bh / 2.0) * h))
            gt_items.append({"box": (x1, y1, x2, y2), "label": CLASS_NAMES[cls_id]})

        samples.append({"image_path": image_path, "gt_items": gt_items})
    return samples


def train_shape_classifier(train_samples, hsv_ranges, kernel_size, roi_min_area):
    features = []
    labels = []

    for sample in train_samples:
        image = cv2.imread(str(sample["image_path"]))
        if image is None:
            continue

        for gt in sample["gt_items"]:
            x1, y1, x2, y2 = gt["box"]
            roi = image[y1:y2, x1:x2]
            if roi.size == 0:
                continue

            roi_detections = detect_color_blobs(
                roi,
                hsv_ranges=hsv_ranges,
                min_area=roi_min_area,
                kernel_size=kernel_size,
            )
            if not roi_detections:
                continue

            best = max(roi_detections, key=lambda item: item["area"])
            features.append(contour_features(best["contour"]))
            labels.append(gt["label"])

    if not features:
        raise RuntimeError("No training contours were extracted from the training split.")

    clf = RandomForestClassifier(
        n_estimators=300,
        random_state=0,
        class_weight="balanced",
    )
    clf.fit(np.array(features), np.array(labels))
    return clf, len(features)


def evaluate(test_samples, hsv_ranges, kernel_size, detect_min_area, iou_threshold, clf):
    total_predictions = 0
    correct_shape_predictions = 0
    matched_pairs_true = []
    matched_pairs_pred = []
    total_gt = 0
    matched_gt = 0
    ious_all_gt = []
    ious_matched = []

    for sample in test_samples:
        image = cv2.imread(str(sample["image_path"]))
        if image is None:
            continue

        gt_items = sample["gt_items"]
        total_gt += len(gt_items)
        detections = detect_color_blobs(
            image,
            hsv_ranges=hsv_ranges,
            min_area=detect_min_area,
            kernel_size=kernel_size,
        )
        used_gt_indices = set()

        for det in sorted(detections, key=lambda item: item["area"], reverse=True):
            total_predictions += 1
            pred_shape = clf.predict([contour_features(det["contour"])])[0]

            best_idx = None
            best_iou = 0.0
            for idx, gt in enumerate(gt_items):
                if idx in used_gt_indices:
                    continue
                score = iou(det["box"], gt["box"])
                if score > best_iou:
                    best_iou = score
                    best_idx = idx

            if best_idx is None or best_iou < float(iou_threshold):
                continue

            used_gt_indices.add(best_idx)
            gt_shape = gt_items[best_idx]["label"]
            matched_pairs_true.append(gt_shape)
            matched_pairs_pred.append(pred_shape)

            if pred_shape == gt_shape:
                correct_shape_predictions += 1

        for gt in gt_items:
            best_iou = max((iou(gt["box"], det["box"]) for det in detections), default=0.0)
            ious_all_gt.append(best_iou)
            if best_iou >= float(iou_threshold):
                matched_gt += 1
                ious_matched.append(best_iou)

    shape_precision = (
        correct_shape_predictions / total_predictions if total_predictions > 0 else 0.0
    )
    bbox_mean_iou_all_gt = sum(ious_all_gt) / len(ious_all_gt) if ious_all_gt else 0.0
    bbox_mean_iou_matched = sum(ious_matched) / len(ious_matched) if ious_matched else 0.0
    bbox_pass_rate = matched_gt / total_gt if total_gt > 0 else 0.0

    print(f"ground-truth instances: {total_gt}")
    print(f"predicted blobs: {total_predictions}")
    print(f"shape precision over all predicted blobs: {shape_precision:.4f}")
    print(f"matched GT at IoU >= {iou_threshold:.2f}: {matched_gt}/{total_gt} ({bbox_pass_rate:.2%})")
    print(f"bbox mean IoU over all GT: {bbox_mean_iou_all_gt:.4f}")
    print(f"bbox mean IoU over matched GT: {bbox_mean_iou_matched:.4f}\n")

    if matched_pairs_true:
        print("shape classification report on IoU-matched blobs:")
        print(
            classification_report(
                matched_pairs_true,
                matched_pairs_pred,
                digits=4,
                zero_division=0,
            )
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Evaluate contour-based shape classification and HSV-blob bbox IoU."
    )
    parser.add_argument(
        "--train-image-dir",
        default="/home/student24/robotproject/datasets/shapes/cube_seperated_dataset/images/train",
    )
    parser.add_argument(
        "--train-label-dir",
        default="/home/student24/robotproject/datasets/shapes/cube_seperated_dataset/labels/train",
    )
    parser.add_argument(
        "--test-image-dir",
        default="/home/student24/robotproject/datasets/shapes/cube_seperated_dataset/images/test",
    )
    parser.add_argument(
        "--test-label-dir",
        default="/home/student24/robotproject/datasets/shapes/cube_seperated_dataset/labels/test",
    )
    parser.add_argument(
        "--hsv-cfg",
        default="/home/student24/robotproject/tools/color_ranges.yaml",
    )
    parser.add_argument("--kernel-size", type=int, default=5)
    parser.add_argument("--detect-min-area", type=float, default=1200.0)
    parser.add_argument("--roi-min-area", type=float, default=50.0)
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    return parser.parse_args()


def main():
    args = parse_args()
    hsv_ranges = load_hsv_ranges(args.hsv_cfg)
    train_samples = load_split(args.train_image_dir, args.train_label_dir)
    test_samples = load_split(args.test_image_dir, args.test_label_dir)

    clf, train_count = train_shape_classifier(
        train_samples=train_samples,
        hsv_ranges=hsv_ranges,
        kernel_size=args.kernel_size,
        roi_min_area=args.roi_min_area,
    )

    print(f"training contours extracted: {train_count}")
    evaluate(
        test_samples=test_samples,
        hsv_ranges=hsv_ranges,
        kernel_size=args.kernel_size,
        detect_min_area=args.detect_min_area,
        iou_threshold=args.iou_threshold,
        clf=clf,
    )


if __name__ == "__main__":
    main()
