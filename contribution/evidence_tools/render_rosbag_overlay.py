import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from rosbag_video_utils import draw_text_block, fit_text_lines, open_video_writer


COLOR_MAP = {
    "red": (50, 50, 255),
    "blue": (255, 120, 40),
    "yellow": (0, 220, 255),
    "white": (235, 235, 235),
    "unknown": (180, 180, 180),
    "cube": (80, 220, 80),
    "rectangle_prism": (255, 180, 50),
    "triangle_prism": (255, 120, 255),
    "cylinder": (80, 180, 255),
    "arch": (255, 80, 80),
}


def color_for_name(name, fallback=(0, 255, 0)):
    return COLOR_MAP.get(name, fallback)


def load_inputs(manifest_path, analysis_path):
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    analysis = json.loads(Path(analysis_path).read_text(encoding="utf-8"))
    root_dir = Path(manifest_path).parent
    frame_map = {int(item["frame_idx"]): item for item in analysis["frames"]}
    return manifest, analysis, root_dir, frame_map


def draw_color_frame(image, frame_result):
    for det in frame_result.get("detections", []):
        x1, y1, x2, y2 = det["bbox"]
        color_name = det["color"]
        draw_color = color_for_name(color_name)
        label = f"{det.get('kind', 'obj')}:{color_name}"
        score = det.get("score")
        if score is not None:
            label += f" {score:.2f}"

        cv2.rectangle(image, (x1, y1), (x2, y2), draw_color, 2)
        cv2.putText(
            image,
            label,
            (x1, max(24, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            draw_color,
            2,
            cv2.LINE_AA,
        )
        center = det.get("center")
        if center:
            cv2.circle(image, (int(center[0]), int(center[1])), 4, draw_color, -1)
    return image


def draw_shape_frame(image, frame_result):
    for det in frame_result.get("detections", []):
        x1, y1, x2, y2 = det["bbox"]
        shape_name = det["shape"]
        color_name = det["color"]
        draw_color = color_for_name(shape_name, fallback=color_for_name(color_name))

        contour = np.array(det["contour"], dtype=np.int32).reshape(-1, 1, 2)
        cv2.drawContours(image, [contour], -1, draw_color, 2)
        cv2.rectangle(image, (x1, y1), (x2, y2), draw_color, 2)

        feats = det.get("features", {})
        label = f"{shape_name} / {color_name}"
        feat_line = (
            f"aspect={feats.get('aspect_ratio', 0.0):.2f} "
            f"extent={feats.get('extent', 0.0):.2f} "
            f"circ={feats.get('circularity', 0.0):.2f}"
        )
        cv2.putText(
            image,
            label,
            (x1, max(24, y1 - 26)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            draw_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            feat_line,
            (x1, max(46, y1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            draw_color,
            2,
            cv2.LINE_AA,
        )
    return image


def render_overlay(
    manifest_path,
    analysis_path,
    out_video,
    title,
    subtitle,
    summary_lines,
    fps=None,
    mode_override=None,
):
    manifest, analysis, root_dir, frame_map = load_inputs(manifest_path, analysis_path)
    render_mode = mode_override or analysis["mode"]

    frames_info = manifest["frames"]
    if not frames_info:
        raise RuntimeError("Manifest contains no frames.")

    first_image = cv2.imread(str(root_dir / frames_info[0]["image_path"]))
    if first_image is None:
        raise RuntimeError("Failed to load the first frame image.")

    video_fps = float(fps or manifest.get("sample_fps", 10.0))
    writer = open_video_writer(out_video, video_fps, first_image.shape[1], first_image.shape[0])

    header_lines = [line for line in [title, subtitle] if line]
    footer_lines = fit_text_lines(summary_lines or [], max_width=48)

    for item in frames_info:
        image_path = root_dir / item["image_path"]
        frame = cv2.imread(str(image_path))
        if frame is None:
            continue

        result = frame_map.get(int(item["frame_idx"]), {})
        if render_mode == "shape_iou":
            frame = draw_shape_frame(frame, result)
        else:
            frame = draw_color_frame(frame, result)

        if header_lines:
            draw_text_block(frame, header_lines, origin=(20, 34), line_height=28)
        if footer_lines:
            draw_text_block(
                frame,
                footer_lines,
                origin=(20, frame.shape[0] - 110),
                line_height=26,
                font_scale=0.62,
            )

        stamp_line = [f"t = {item['rel_sec']:.2f}s | frame {int(item['frame_idx']):04d}"]
        draw_text_block(
            frame,
            stamp_line,
            origin=(frame.shape[1] - 250, 34),
            line_height=24,
            font_scale=0.55,
        )
        writer.write(frame)

    writer.release()
    print(f"Saved overlay video to {out_video}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Render offline overlay videos from exported ROS2-bag frames and analysis JSON."
    )
    parser.add_argument("--manifest", required=True, help="Manifest JSON from export_rosbag_video.py")
    parser.add_argument("--analysis-json", required=True, help="Analysis JSON from an analyze script")
    parser.add_argument("--out-video", required=True, help="Output mp4 path")
    parser.add_argument("--title", default="")
    parser.add_argument("--subtitle", default="")
    parser.add_argument("--summary-line", action="append", default=[])
    parser.add_argument("--fps", type=float, default=0.0)
    parser.add_argument("--mode", choices=["color", "shape_iou"], default="")
    return parser.parse_args()


def main():
    args = parse_args()
    render_overlay(
        manifest_path=args.manifest,
        analysis_path=args.analysis_json,
        out_video=args.out_video,
        title=args.title,
        subtitle=args.subtitle,
        summary_lines=args.summary_line,
        fps=args.fps or None,
        mode_override=args.mode or None,
    )


if __name__ == "__main__":
    main()
