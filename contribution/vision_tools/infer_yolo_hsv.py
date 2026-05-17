import argparse
import os
import sys
import time
from pathlib import Path
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except Exception as e:
    print("Ultralytics 未安装或导入失败，请先 pip install ultralytics", file=sys.stderr)
    raise

from color_utils import HSVColorEstimator


def draw_box_with_label(img, xyxy, label, color=(0, 255, 0)):
    x1, y1, x2, y2 = map(int, xyxy)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    cv2.putText(img, label, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def infer_on_source(model_path: str,
                    source: str,
                    hsv_cfg: str,
                    conf: float = 0.25,
                    imgsz: int = 416,
                    device: str = "",
                    classes: str = "",
                    save: bool = False,
                    out_dir: str = ""):
    model = YOLO(model_path)
    estimator = HSVColorEstimator(hsv_cfg)

    # 解析 classes
    classes_list = None
    if classes:
        classes_list = [int(x) for x in classes.split(',') if x.strip().isdigit()]

    # 摄像头或视频/图片
    is_cam = source.isdigit()
    if is_cam:
        cap = cv2.VideoCapture(int(source))
        if not cap.isOpened():
            print(f"无法打开摄像头 {source}")
            return
        writer = None
        if save:
            os.makedirs(out_dir, exist_ok=True)
            out_path = os.path.join(out_dir, "camera_out.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = cap.get(cv2.CAP_PROP_FPS) or 30
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            results = model.predict(source=frame, conf=conf, imgsz=imgsz, device=device, classes=classes_list, verbose=False)
            vis = frame.copy()
            for r in results:
                if r.boxes is None:
                    continue
                for b in r.boxes:
                    cls_id = int(b.cls[0]) if b.cls is not None else -1
                    xyxy = b.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, xyxy)
                    roi = frame[y1:y2, x1:x2]
                    color_name, ratio, scores = estimator.estimate_from_roi(roi)
                    name = r.names.get(cls_id, str(cls_id))
                    label = f"{name}:{color_name} {ratio:.2f}"
                    draw_box_with_label(vis, xyxy, label)
            cv2.imshow('yolo+hsv', vis)
            if writer:
                writer.write(vis)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        return

    # 文件/目录
    p = Path(source)
    paths = []
    if p.is_dir():
        for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.mp4", "*.avi"):
            paths.extend(sorted(p.glob(ext)))
    else:
        paths = [p]

    os.makedirs(out_dir, exist_ok=True) if save else None

    for fp in paths:
        if fp.suffix.lower() in {".mp4", ".avi", ".mov", ".mkv"}:
            cap = cv2.VideoCapture(str(fp))
            if not cap.isOpened():
                print(f"打开视频失败: {fp}")
                continue
            writer = None
            if save:
                out_path = os.path.join(out_dir, fp.stem + "_out.mp4")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                fps = cap.get(cv2.CAP_PROP_FPS) or 30
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                results = model.predict(source=frame, conf=conf, imgsz=imgsz, device=device, classes=classes_list, verbose=False)
                vis = frame.copy()
                for r in results:
                    if r.boxes is None:
                        continue
                    for b in r.boxes:
                        cls_id = int(b.cls[0]) if b.cls is not None else -1
                        xyxy = b.xyxy[0].tolist()
                        x1, y1, x2, y2 = map(int, xyxy)
                        roi = frame[y1:y2, x1:x2]
                        color_name, ratio, scores = estimator.estimate_from_roi(roi)
                        name = r.names.get(cls_id, str(cls_id))
                        label = f"{name}:{color_name} {ratio:.2f}"
                        draw_box_with_label(vis, xyxy, label)
                if save:
                    writer.write(vis)
                cv2.imshow('yolo+hsv', vis)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            cap.release()
            if writer:
                writer.release()
        else:
            img = cv2.imread(str(fp))
            if img is None:
                print(f"读取失败: {fp}")
                continue
            results = model.predict(source=img, conf=conf, imgsz=imgsz, device=device, classes=classes_list, verbose=False)
            vis = img.copy()
            for r in results:
                if r.boxes is None:
                    continue
                for b in r.boxes:
                    cls_id = int(b.cls[0]) if b.cls is not None else -1
                    xyxy = b.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, xyxy)
                    roi = img[y1:y2, x1:x2]
                    color_name, ratio, scores = estimator.estimate_from_roi(roi)
                    name = r.names.get(cls_id, str(cls_id))
                    label = f"{name}:{color_name} {ratio:.2f}"
                    draw_box_with_label(vis, xyxy, label)
            if save:
                out_path = os.path.join(out_dir, fp.stem + "_out.jpg")
                cv2.imwrite(out_path, vis)
            cv2.imshow('yolo+hsv', vis)
            cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="YOLO 模型路径（.pt）")
    ap.add_argument("--source", required=True, help="输入：摄像头索引/图片/视频/目录")
    ap.add_argument("--hsv_cfg", default="/home/student24/robotproject/tools/color_ranges.yaml", help="HSV 配置文件路径")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--device", default="", help="'cpu' 或 'cuda:0'")
    ap.add_argument("--classes", default="", help="可选：限制检测类别，如 '0,1'")
    ap.add_argument("--save", action="store_true", help="保存可视化结果")
    ap.add_argument("--out_dir", default="/home/student24/robotproject/outputs", help="输出目录")
    args = ap.parse_args()

    infer_on_source(
        model_path=args.model,
        source=args.source,
        hsv_cfg=args.hsv_cfg,
        conf=args.conf,
        imgsz=args.imgsz,
        device=args.device,
        classes=args.classes,
        save=args.save,
        out_dir=args.out_dir,
    )
