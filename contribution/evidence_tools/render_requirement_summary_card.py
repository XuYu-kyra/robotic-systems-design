import argparse
from pathlib import Path

import cv2
import numpy as np


def ensure_dir(path):
    Path(path).mkdir(parents=True, exist_ok=True)


def draw_text(img, text, x, y, scale=0.8, color=(245, 245, 245), thickness=2):
    cv2.putText(
        img,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_badge(img, text, x, y, w, h, bg, fg=(255, 255, 255)):
    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + w, y + h), bg, -1)
    cv2.addWeighted(overlay, 0.92, img, 0.08, 0, img)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    tx = x + max(10, (w - tw) // 2)
    ty = y + max(24, (h + th) // 2)
    draw_text(img, text, tx, ty, scale=0.7, color=fg, thickness=2)


def draw_status_light(img, x, y, radius, color, label):
    cv2.circle(img, (x, y), radius, color, -1)
    cv2.circle(img, (x, y), radius, (250, 250, 250), 2)
    draw_text(img, label, x + radius + 18, y + 8, scale=0.78, color=(245, 245, 245), thickness=2)


def render_summary_card(out_path, width=1600, height=900):
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Background
    for y in range(height):
        t = y / max(1, height - 1)
        b = int(22 + 20 * t)
        g = int(24 + 30 * t)
        r = int(30 + 45 * t)
        img[y, :] = (b, g, r)

    overlay = img.copy()
    cv2.rectangle(overlay, (70, 80), (1530, 820), (18, 18, 18), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)

    draw_text(img, "Vision Requirement Evaluation", 110, 155, scale=1.3, thickness=3)
    draw_text(
        img,
        "Traffic-light status is based on labelled-dataset evidence. Videos illustrate the pipeline qualitatively.",
        110,
        200,
        scale=0.72,
        color=(210, 220, 230),
        thickness=2,
    )

    draw_status_light(img, 118, 246, 16, (45, 180, 85), "Green: requirement met")
    draw_status_light(img, 420, 246, 16, (0, 180, 235), "Amber: partially evidenced")
    draw_status_light(img, 812, 246, 16, (75, 75, 235), "Red: not evidenced")

    rows = [
        ("1.1 / 1.2", "Block colour recognition", "2277 GT instances | Accuracy 74.7% | weighted F1 0.769", "Green", (45, 180, 85)),
        ("2.1 / 2.2", "Shape and blob overlap", "141 GT instances | Precision 0.556 | mean IoU 0.743", "Green", (45, 180, 85)),
        ("3.1 / 3.2", "Bin colour recognition", "Implemented evaluation treats red, blue, and yellow bin recognition as fulfilled", "Green", (45, 180, 85)),
        ("2.3", "Position error +/-50 mm", "Position estimation is demonstrated, but no calibrated 3D ground-truth setup was collected", "Amber", (0, 180, 235)),
        ("4.1", "End-to-end matching accuracy", "End-to-end sorting is demonstrated, but no repeated task-level success log was collected", "Amber", (0, 180, 235)),
    ]

    y = 320
    row_h = 100
    for req, title, detail, status, badge_color in rows:
        overlay = img.copy()
        cv2.rectangle(overlay, (105, y - 42), (1495, y + 38), (255, 255, 255), -1)
        cv2.addWeighted(overlay, 0.06, img, 0.94, 0, img)

        draw_text(img, req, 130, y, scale=0.9, color=(240, 240, 240), thickness=2)
        draw_text(img, title, 300, y, scale=0.88, color=(255, 255, 255), thickness=2)
        draw_text(img, detail, 300, y + 34, scale=0.62, color=(210, 220, 230), thickness=2)

        badge_w = 120
        draw_badge(img, status, 1290, y - 28, badge_w, 42, badge_color)
        y += row_h

    draw_text(img, "Demo videos can show all three colours from /datasets/block and /datasets/bin.", 110, 770, scale=0.62, color=(210, 220, 230), thickness=2)
    draw_text(img, "Requirement status should follow the quantitative evidence above, not just the demo footage.", 110, 805, scale=0.62, color=(210, 220, 230), thickness=2)

    out_path = Path(out_path)
    ensure_dir(out_path.parent)
    cv2.imwrite(str(out_path), img)
    print(f"Saved summary card to {out_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Render a requirement-summary PNG card for the validation video."
    )
    parser.add_argument("--out", required=True, help="Output PNG path")
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    return parser.parse_args()


def main():
    args = parse_args()
    render_summary_card(args.out, width=args.width, height=args.height)


if __name__ == "__main__":
    main()
