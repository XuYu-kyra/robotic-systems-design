# ROS2 Bag Offline Overlay Workflow

This workflow turns a recorded RealSense ROS2 bag into editable evidence videos.

## Recommended real-pipeline recording workflow

For the final validation video, prefer the real ROS2 pipeline plus recorder:

1. Launch `vision_pipeline_manager.launch.py` with `use_recorder:=true`
2. Play the ROS2 bag through the real pipeline
3. Let the recorder save:
   - `debug_frames/*.jpg`
   - `debug_frames.jsonl`
   - `detections_3d.jsonl`
4. Render the final mp4 from saved frames and timestamps using:
   - `tools/render_recorded_debug_frames.py`

This avoids the severe time compression caused by writing mp4 directly at a fixed fps while frames are dropped during processing.

## Rebuild after recorder changes

The recorder node lives inside `ros_nd2`, so after modifying it you need to rebuild:

```bash
cd /home/student24/robotproject/ros_nd2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select color_blob_vision
source /home/student24/robotproject/ros_nd2/install/setup.bash
```

## Scripts

- `tools/export_rosbag_video.py`
- `tools/analyze_rosbag_block_color.py`
- `tools/analyze_rosbag_shape_iou.py`
- `tools/render_rosbag_overlay.py`

Shared helper:

- `tools/rosbag_video_utils.py`

## 1. Export a video segment from a ROS2 bag

Example:

```bash
./env_robot/bin/python tools/export_rosbag_video.py \
  --bag /path/to/your_rosbag2_dir \
  --out-dir /home/student24/robotproject/output/demo_segment \
  --topic /camera/color/image_raw \
  --start-sec 0 \
  --duration-sec 20 \
  --sample-fps 10
```

Outputs:

- `output/demo_segment/frames/*.jpg`
- `output/demo_segment/raw_video.mp4`
- `output/demo_segment/manifest.json`

## 2. Render block-colour or bin-colour overlays

Analyze:

```bash
./env_robot/bin/python tools/analyze_rosbag_block_color.py \
  --manifest /home/student24/robotproject/output/demo_segment/manifest.json \
  --out-json /home/student24/robotproject/output/demo_segment/block_color_analysis.json \
  --kind block
```

For bins, change `--kind bin`.

Render:

```bash
./env_robot/bin/python tools/render_rosbag_overlay.py \
  --manifest /home/student24/robotproject/output/demo_segment/manifest.json \
  --analysis-json /home/student24/robotproject/output/demo_segment/block_color_analysis.json \
  --out-video /home/student24/robotproject/output/demo_segment/block_color_overlay.mp4 \
  --mode color \
  --title "Requirement 1: Block Colour Recognition" \
  --subtitle "HSV blob detections on recorded ROS2 bag frames" \
  --summary-line "Block colour accuracy: 74.7%" \
  --summary-line "Block colour weighted F1: 0.769" \
  --summary-line "Requirement 1.1 / 1.2: Met"
```

## 3. Render shape overlays

Analyze:

```bash
./env_robot/bin/python tools/analyze_rosbag_shape_iou.py \
  --manifest /home/student24/robotproject/output/demo_segment/manifest.json \
  --out-json /home/student24/robotproject/output/demo_segment/shape_analysis.json
```

Render:

```bash
./env_robot/bin/python tools/render_rosbag_overlay.py \
  --manifest /home/student24/robotproject/output/demo_segment/manifest.json \
  --analysis-json /home/student24/robotproject/output/demo_segment/shape_analysis.json \
  --out-video /home/student24/robotproject/output/demo_segment/shape_overlay.mp4 \
  --mode shape_iou \
  --title "Requirement 2: Shape Capability" \
  --subtitle "Contour-derived features from HSV-segmented blobs" \
  --summary-line "Shape precision: 0.556" \
  --summary-line "Mean IoU: 0.743" \
  --summary-line "Requirement 2.1 / 2.2: Met"
```

## 4. Bin-colour overlay example

```bash
./env_robot/bin/python tools/analyze_rosbag_block_color.py \
  --manifest /home/student24/robotproject/output/demo_segment/manifest.json \
  --out-json /home/student24/robotproject/output/demo_segment/bin_color_analysis.json \
  --kind bin

./env_robot/bin/python tools/render_rosbag_overlay.py \
  --manifest /home/student24/robotproject/output/demo_segment/manifest.json \
  --analysis-json /home/student24/robotproject/output/demo_segment/bin_color_analysis.json \
  --out-video /home/student24/robotproject/output/demo_segment/bin_color_overlay.mp4 \
  --mode color \
  --title "Requirement 3: Bin Colour Recognition" \
  --subtitle "Qualitative bag overlay with quantitative summary card" \
  --summary-line "Repository evidence is complete for the available red/blue subset." \
  --summary-line "Most yellow-bin GT images referenced by color_gt_bin.csv are missing." \
  --summary-line "Requirement 3.1 / 3.2: Partially evidenced"
```

## What these bag scripts are best for

These scripts are best for:

- producing clean qualitative overlays for the validation video
- showing what the detector sees on your recorded demo bag
- adding requirement summary text onto the rendered footage

## What they do not do automatically

These bag scripts do not automatically compute ground-truth metrics from the demo bag itself, because the bag does not inherently contain labelled colour, shape, or IoU annotations.

Use the quantitative results already computed from:

- `tools/color_evaluate.py`
- `tools/shape_evaluate.py`
- `docs/vision_requirement_evidence.md`

Then place those numbers into `--summary-line` in the rendered video.

## Timestamp-aligned rendering from the real pipeline recorder

After running the real pipeline recorder, render a correctly timed video like this:

```bash
./env_robot/bin/python tools/render_recorded_debug_frames.py \
  --run-dir /home/student24/robotproject/output/pipeline_recordings/block_bag_001 \
  --out-video /home/student24/robotproject/output/pipeline_recordings/block_bag_001/debug_image_timed.mp4 \
  --fps 10
```

This uses `debug_frames.jsonl` timestamps instead of assuming a fixed recording fps.
