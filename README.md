# Robotic Systems Design — ROS 2 RGB-D Perception

Portfolio extract of my perception work for **AERO62520 Robotic Systems Design Project**, a year-long University of Manchester MSc Robotics team project built around a physical **Leo Rover**, **myCobot 280 Pi**, and **RealSense RGB-D camera**.

The team project combined perception, navigation, and manipulation for a mobile-manipulation task. My ownership was the **perception subsystem**: task-oriented block/bin detection, RGB-D localisation, temporal stabilisation, ROS 2 interfaces, and target-pose handoff toward downstream robot actions. This repository is a curated view of that contribution, not a claim of sole authorship of the full robot.

- Original team repository: [Picklerick313/AERO62520_Robotic_Systems_Design_Project](https://github.com/Picklerick313/AERO62520_Robotic_Systems_Design_Project)
- Team roles in the original repository: Design — Tom; Vision — XuYu and Wangsiyuan; Manipulator — Lyuxingze; Navigation — Ansber
- Detailed source attribution: [ATTRIBUTION.md](ATTRIBUTION.md)

### Vision-team responsibility split

Within the vision team, my responsibility was the perception subsystem documented in this portfolio: the runtime ROS 2 pipeline, HSV/RGB-D localisation, temporal stabilisation, task-facing target outputs, target-pose/TF integration, downstream handoff, and validation tooling. Wangsiyuan contributed to the early YOLO stage by collecting training images and annotating the dataset in Roboflow. This distinction keeps the team attribution intact while making the ownership of the retained work explicit.

## Hardware & Integration Scope

| Area | Repository-backed scope |
| --- | --- |
| Robot context | Physical Leo Rover + myCobot 280 Pi mobile-manipulation project |
| Sensor path | RealSense-style RGB, aligned depth, and camera intrinsics; recorded RealSense/ROS 2 bag workflows |
| My ownership | Perception package, supporting vision/evidence tools, task-facing target selection, and handoff documentation |
| Spatial output | 2D detections projected to metric 3D; `Detection3DArray` and selected `PoseStamped` target output |
| Frame integration | `perception_manager.py` looks up TF and transforms a selected pose into configurable `target_frame`; offline mode can retain the camera/source frame when TF is unavailable |
| Downstream handoff | `/task/current_target_pose` feeds `/task/arm_target_pose`; semantic arm and navigation command topics are documented |
| Boundary | Perception-to-manipulator interface work is present, but the final physical-arm success/failure loop and end-to-end real grasp were not completed in the original project |

## My Contribution At A Glance

- Built ROS 2 HSV-based object/bin detection for white, red, blue, and yellow task targets.
- Converted 2D blob centres and bounding boxes into 3D camera-frame positions using aligned depth and `CameraInfo` intrinsics.
- Added median depth sampling, metric-size classification, spatial filtering, lightweight tracking, exponential smoothing, and multi-frame confirmation.
- Published task-facing labels, visibility/reachability state, and `geometry_msgs/msg/PoseStamped` targets.
- Implemented configurable TF lookup and coordinate transformation toward a robot frame such as `base_link`.
- Defined manipulator handoff topics (`/task/arm_command`, `/task/arm_target_pose`) and navigation intent (`/task/nav_goal_name`).
- Used RViz/debug images, ROS 2 bag replay, timestamped recordings, and overlay/evaluation tools to inspect system behaviour.
- Explored a learning-based YOLO + HSV detector before selecting an HSV-first runtime pipeline as a resource-aware system trade-off.

## Validated vs Integration Scope

### Demonstrated in the retained artifacts

- ROS 2 perception nodes and launch configuration
- task-oriented block/bin colour detection
- RealSense-style RGB + aligned-depth 2D-to-3D localisation
- temporal smoothing and multi-frame confirmation
- structured `Detection2DArray`, `Detection3DArray`, and `PoseStamped` outputs
- rosbag/offline pipeline validation and visual overlays

### Implemented as integration work

- target selection and target-pose interfaces
- configurable `target_frame` plus TF transformation logic
- perception-to-task-manager data flow
- manipulator and navigation command/handoff contracts

### Not completed in the original project

- calibrated, validated camera-to-robot TF for final hardware execution
- real navigation/manipulator completion feedback consumed by `task_manager`
- perception → physical arm → grasp → real success/failure closed loop
- deployment of YOLO/TensorRT on the original robot

The current `task_manager.py` can publish outward commands and target poses, but its retained implementation advances navigation and arm states through simulated timing. With `simulate_nav:=false` or `simulate_arm:=false`, real result subscribers still need to be implemented. The handoff is therefore **partial integration**, not a completed robot-execution loop.

## System Data Flow

1. RGB image → HSV segmentation, contour filtering, 2D bounding boxes and orientation
2. Aligned depth + camera intrinsics → metric 3D position and approximate object size
3. Size/depth rules → `block:<color>` or `bin:<color>`
4. Multi-frame association + smoothing → stable `Detection3DArray`
5. Target policy + TF lookup → selected `PoseStamped` in the configured frame
6. `task_manager` → semantic navigation/arm commands and arm target pose

Relevant implementation:

- [`color_blob_detector.py`](contribution/ros2_color_blob_vision/color_blob_vision/color_blob_detector.py)
- [`blob_depth_to_3d_smoothed.py`](contribution/ros2_color_blob_vision/color_blob_vision/blob_depth_to_3d_smoothed.py)
- [`perception_manager.py`](contribution/ros2_color_blob_vision/color_blob_vision/perception_manager.py)
- [`task_manager.py`](contribution/ros2_color_blob_vision/color_blob_vision/task_manager.py)
- [`vision_pipeline_manager.launch.py`](contribution/ros2_color_blob_vision/launch/vision_pipeline_manager.launch.py)

## Resource-Aware YOLO → HSV Decision

The earlier work includes a functional YOLO + HSV inference path and dataset/calibration tooling. The move to HSV-first perception was not a failed-model fallback. Navigation, manipulation, and perception had to share compute on the robot, while the task used controlled target colours and benefited from transparent, quickly tunable failure modes. After discussion with the supervisor, I chose the lighter pipeline to reduce runtime dependencies and make integration and debugging more predictable.

The repository does **not** contain device benchmark logs supporting exact YOLO-versus-HSV latency figures, so the migration note treats performance differences as expected characteristics rather than measured results. It also does not include the trained weights needed to claim YOLO deployment on the original robot.

See [`YOLO_TO_HSV_MIGRATION_SUMMARY.md`](contribution/notes/YOLO_TO_HSV_MIGRATION_SUMMARY.md).

## Tooling and Evidence

`contribution/vision_tools/` contains dataset inspection/splitting, RealSense bag frame extraction, HSV calibration, labelling, evaluation, and YOLO + HSV inference utilities.

`contribution/evidence_tools/` contains ROS 2 bag export, recorded-frame rendering, overlay generation, 2D/3D output extraction, and colour/shape analysis utilities.

Selected retained artifacts:

- [Main pipeline overlay](evidence/videos/main_pipeline_overlay_007.mp4)
- [Block pipeline overlay](evidence/videos/block_pipeline_overlay.mp4)
- [Bin pipeline overlay](evidence/videos/bin_pipeline_overlay.mp4)
- [Historical requirement summary](evidence/images/vision_requirement_summary_requirements_videos.png)
- [Evidence provenance and limitations](evidence/docs/vision_requirement_evidence.md)
- [ROS 2 bag overlay workflow](evidence/docs/rosbag_offline_overlay_workflow.md)

Evidence boundary: the videos and scripts are retained, but the labelled datasets, CSV files, model weights, and source bags referenced by local paths are not packaged here. Reported metrics are therefore historical results from the original workspace, not independently reproducible benchmarks from this portfolio repository alone.

## Repository Structure

```text
.
├── README.md
├── ATTRIBUTION.md
├── contribution
│   ├── ros2_color_blob_vision
│   ├── vision_tools
│   ├── evidence_tools
│   └── notes
└── evidence
    ├── docs
    ├── images
    └── videos
```

## Attribution and License

This was a team project. I do not claim ownership of the navigation, manipulation, mechanical design, or the complete system. The curated files here correspond primarily to the original repository's `Xy` perception work and are linked back to the team source.

The original repository's `LICENSE` file is MIT-licensed, and that license is preserved here. The badge in the original team README says Apache 2.0, but the actual license file contains the MIT License; this repository follows the license file.
