import cv2
import csv
from pathlib import Path
from ultralytics import YOLO

MODEL_PATH = "/home/student24/robotproject/runs/detect/train13/weights/best.pt"
IMG_DIR = "/home/student24/robotproject/datasets/shapes/images_all/images"
OUT_CSV = "color_gt.csv"

COLORS = ["red", "blue", "yellow","other"]

def main():
    model = YOLO(MODEL_PATH)
    img_paths = sorted(Path(IMG_DIR).glob("*.jpg"))

    with open(OUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "x1", "y1", "x2", "y2", "color"])

        for img_path in img_paths:
            img = cv2.imread(str(img_path))
            results = model.predict(img, conf=0.2, verbose=False)

            for r in results:
                if r.boxes is None:
                    continue
                for b in r.boxes:
                    x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                    roi = img[y1:y2, x1:x2]

                    # 展示 ROI
                    while True:
                        display = roi.copy()
                        cv2.putText(display, " | ".join(COLORS),
                                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

                        cv2.imshow("Choose Color (press first letter)", display)
                        key = cv2.waitKey(0)

                        for c in COLORS:
                            if key == ord(c[0]):  # 按首字母
                                writer.writerow([img_path.name, x1, y1, x2, y2, c])
                                print(f"Labeled {img_path.name} -> {c}")
                                break
                        else:
                            continue  # 未按颜色按键，继续等待

                        break  # 颜色选择完毕 → 下一 ROI

        cv2.destroyAllWindows()
    print("标注完成，已保存到:", OUT_CSV)


if __name__ == "__main__":
    main()
