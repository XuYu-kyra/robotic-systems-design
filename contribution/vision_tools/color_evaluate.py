import csv
import cv2
from pathlib import Path
from ultralytics import YOLO
from color_utils import HSVColorEstimator
from collections import defaultdict
import numpy as np
import pandas as pd
from sklearn.metrics import confusion_matrix, classification_report

def load_gt(csv_path):
    gt = {}
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            fname = row["filename"]
            x1, y1, x2, y2 = map(int, [row["x1"], row["y1"], row["x2"], row["y2"]])
            color = row["color"].strip()
            gt.setdefault(fname, []).append({
                "box": (x1, y1, x2, y2),
                "color": color
            })
    return gt

def iou(box1, box2):
    x1, y1, x2, y2 = box1
    xx1 = max(x1, box2[0])
    yy1 = max(y1, box2[1])
    xx2 = min(x2, box2[2])
    yy2 = min(y2, box2[3])
    w = max(0, xx2 - xx1)
    h = max(0, yy2 - yy1)
    inter = w * h
    area1 = (x2 - x1) * (y2 - y1)
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0

def evaluate(model_path, img_dir, hsv_cfg, gt_csv):
    model = YOLO(model_path)
    estimator = HSVColorEstimator(hsv_cfg)
    gt = load_gt(gt_csv)

    y_true = []
    y_pred = []

    total = 0
    correct = 0

    for fp in Path(img_dir).glob("*.jpg"):
        img = cv2.imread(str(fp))
        if img is None:
            continue

        results = model.predict(source=img, conf=0.25, imgsz=416, verbose=False)

        pred_boxes = []
        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                xyxy = list(map(int, b.xyxy[0].tolist()))
                pred_boxes.append(xyxy)

        if fp.name not in gt:
            continue

        for gt_item in gt[fp.name]:
            gt_box = gt_item["box"]
            gt_color = gt_item["color"]

            best_iou = 0
            best_pred_box = None
            for pb in pred_boxes:
                score = iou(gt_box, pb)
                if score > best_iou:
                    best_iou = score
                    best_pred_box = pb

            if best_iou < 0.5:
                gt_color_mapped = 'unknown' if gt_color == 'other' else gt_color
                y_true.append(gt_color_mapped)
                y_pred.append("unknown")
                continue

            x1, y1, x2, y2 = best_pred_box
            roi = img[y1:y2, x1:x2]
            pred_color, _, _ = estimator.estimate_from_roi(roi)

            gt_color_mapped = 'unknown' if gt_color == 'other' else gt_color
            pred_color_mapped = 'unknown' if pred_color == 'other' else pred_color

            y_true.append(gt_color_mapped)
            y_pred.append(pred_color_mapped)

            total += 1
            if pred_color_mapped == gt_color_mapped:
                correct += 1

    overall_acc = correct / total if total > 0 else 0
    print(f"total test images: {total}")
    print(f"correct predictions: {correct}")
    print(f"accuracy: {overall_acc:.2%}\n")

    labels = ['red', 'blue', 'yellow', 'unknown']

    cm = confusion_matrix(y_true, y_pred, labels=labels)
    print("confusion matrix:")
    df_cm = pd.DataFrame(cm, index=labels, columns=labels)
    print(df_cm)

    print("\nclassification metrics (Precision / Recall / F1-score):")
    print(classification_report(y_true, y_pred, labels=labels, digits=4, zero_division=0))


def main():
    evaluate(
        "/home/student24/robotproject/runs/detect/train15/weights/best.pt",
        "/home/student24/robotproject/datasets/shapes/images/test",
        "/home/student24/robotproject/tools/color_ranges.yaml",
        "/home/student24/robotproject/color_gt_bin.csv"
    )

if __name__ == "__main__":
    main()
