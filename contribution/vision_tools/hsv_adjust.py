import cv2
import numpy as np
import os
import argparse
from collections import defaultdict

def analyze_images(img_list, color_name, mask_func=None):
    h_vals, s_vals, v_vals = [], [], []
    ratios = []

    for path in img_list:
        img = cv2.imread(path)
        if img is None:
            continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if mask_func:
            mask = mask_func(hsv)
        else:
            mask = np.ones(hsv.shape[:2], dtype=np.uint8) * 255

        pixels = hsv[mask > 0]
        if len(pixels) == 0:
            continue

        h_vals.extend(pixels[:, 0])
        s_vals.extend(pixels[:, 1])
        v_vals.extend(pixels[:, 2])

        ratios.append(len(pixels) / (hsv.shape[0] * hsv.shape[1]))

    if len(h_vals) == 0:
        return None

    h_min, h_max = int(np.percentile(h_vals, 5)), int(np.percentile(h_vals, 95))
    s_min, s_max = int(np.percentile(s_vals, 5)), int(np.percentile(s_vals, 95))
    v_min, v_max = int(np.percentile(v_vals, 5)), int(np.percentile(v_vals, 95))

    ratio_thresh = float(np.percentile(ratios, 10))  # 取 10% 作为阈值

    return {
        "hsv_range": [h_min, s_min, v_min, h_max, s_max, v_max],
        "ratio_thresh": ratio_thresh
    }

# ------------------- 摄像头实时调参 -------------------
def camera_mode(color_name):
    print(f"打开摄像头调 {color_name} 的 HSV 范围和阈值...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("摄像头打开失败")
        return

    print("使用滑动条调 HSV")
    cv2.namedWindow("HSV Adjust")
    def nothing(x): pass

    # 初始值
    cv2.createTrackbar("H_min","HSV Adjust",0,179,nothing)
    cv2.createTrackbar("H_max","HSV Adjust",179,179,nothing)
    cv2.createTrackbar("S_min","HSV Adjust",0,255,nothing)
    cv2.createTrackbar("S_max","HSV Adjust",255,255,nothing)
    cv2.createTrackbar("V_min","HSV Adjust",0,255,nothing)
    cv2.createTrackbar("V_max","HSV Adjust",255,255,nothing)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H_min","HSV Adjust")
        h_max = cv2.getTrackbarPos("H_max","HSV Adjust")
        s_min = cv2.getTrackbarPos("S_min","HSV Adjust")
        s_max = cv2.getTrackbarPos("S_max","HSV Adjust")
        v_min = cv2.getTrackbarPos("V_min","HSV Adjust")
        v_max = cv2.getTrackbarPos("V_max","HSV Adjust")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        masked = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("HSV Adjust", masked)
        key = cv2.waitKey(1)
        if key == 27:  # ESC 退出
            break

    print(f"{color_name} HSV范围: H({h_min}-{h_max}) S({s_min}-{s_max}) V({v_min}-{v_max})")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--img_dir", type=str, help="批量图片目录")
    parser.add_argument("--camera", action="store_true", help="实时摄像头调参")
    parser.add_argument("--color", type=str, default="blue", help="颜色名称")
    args = parser.parse_args()

    if args.img_dir:
        imgs = [os.path.join(args.img_dir, f) for f in os.listdir(args.img_dir)
                if f.lower().endswith(('.jpg','.png','.jpeg'))]
        result = analyze_images(imgs, args.color)
        if result:
            print(f"{args.color} HSV范围: {result['hsv_range']}, ratio_thresh: {result['ratio_thresh']:.3f}")
        else:
            print("没有检测到有效像素")
    elif args.camera:
        camera_mode(args.color)
    else:
        print("请使用 --img_dir 或 --camera 参数")
