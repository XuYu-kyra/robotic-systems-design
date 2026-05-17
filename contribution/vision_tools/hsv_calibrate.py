# save as: /home/student24/robotproject/tools/hsv_calibrate.py
import cv2, numpy as np
import yaml

try:
    with open("/home/student24/robotproject/tools/color_ranges.yaml", 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    init = {}
    if "red" in cfg and isinstance(cfg["red"], list) and len(cfg["red"]) >= 2:
        init["red1"] = tuple(cfg["red"][0])
        init["red2"] = tuple(cfg["red"][1])
    for name in ["blue", "yellow"]:
        if name in cfg and isinstance(cfg[name], list) and len(cfg[name]) > 0:
            init[name] = tuple(cfg[name][0])
except Exception:
    init = {
        "red1":   (0, 120, 70, 10, 255, 255),
        "red2":   (170, 120, 70, 180, 255, 255),
        "blue":   (90, 60, 60, 130, 255, 255),
        "yellow": (20, 80, 80, 35, 255, 255),
    }

colors = ["red1", "red2", "blue", "yellow"]

def nothing(x): pass

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头，请检查设备")
    exit(1)

for c in colors:
    cv2.namedWindow(c)
    for n in ["H_min","S_min","V_min","H_max","S_max","V_max"]:
        cv2.createTrackbar(n, c, 0, 255 if n[0]!="H" else 180, nothing)
    if c in init:
        Hmn, Smn, Vmn, Hmx, Smx, Vmx = init[c]
        cv2.setTrackbarPos("H_min", c, Hmn); cv2.setTrackbarPos("S_min", c, Smn); cv2.setTrackbarPos("V_min", c, Vmn)
        cv2.setTrackbarPos("H_max", c, Hmx); cv2.setTrackbarPos("S_max", c, Smx); cv2.setTrackbarPos("V_max", c, Vmx)

print("按 ESC 键退出")
print("调整滑块后，可以手动将结果写入 color_ranges.yaml")

while True:
    ok, frame = cap.read()
    if not ok: break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    vis = frame.copy()
    for c in colors:
        Hmn = cv2.getTrackbarPos("H_min", c); Smn = cv2.getTrackbarPos("S_min", c); Vmn = cv2.getTrackbarPos("V_min", c)
        Hmx = cv2.getTrackbarPos("H_max", c); Smx = cv2.getTrackbarPos("S_max", c); Vmx = cv2.getTrackbarPos("V_max", c)
        lower = np.array([Hmn, Smn, Vmn], dtype=np.uint8)
        upper = np.array([Hmx, Smx, Vmx], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnts, -1, (0,255,0), 2)
        cv2.putText(vis, c, (10, 25+25*colors.index(c)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    cv2.imshow("view", vis)
    if cv2.waitKey(1) & 0xFF == 27: break

cap.release(); cv2.destroyAllWindows()
