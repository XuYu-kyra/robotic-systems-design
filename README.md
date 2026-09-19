# Robotic Systems Design — ROS 2 Perception for Mobile Manipulation

A year-long MSc Robotics team project at the University of Manchester, combining a Leo Rover, myCobot 280 Pi arm, and RealSense RGB-D camera for object collection and colour-based sorting.

I developed the perception subsystem, from early YOLO experiments to a ROS 2 RGB-D pipeline with temporal filtering, target selection, and coordinate-frame transformation for navigation and manipulation interfaces. Wangsiyuan supported the early YOLO work by collecting training images and annotating the dataset in Roboflow.

[Original team project](https://github.com/Picklerick313/AERO62520_Robotic_Systems_Design_Project) · [Source and team attribution](ATTRIBUTION.md)

## My Contribution

- **RGB-D localisation:** combined image detections, aligned depth, and camera intrinsics to estimate object positions and metric dimensions.
- **Task-specific perception:** detected white bins and coloured blocks/bins using HSV segmentation, ROI filtering, and depth-based size rules.
- **Temporal filtering:** implemented median depth sampling, 3D detection association, multi-frame confirmation, and exponential smoothing of position and yaw.
- **ROS 2 integration:** published `Detection2DArray` and `Detection3DArray` messages, selected targets as `PoseStamped`, and implemented TF lookup and pose transformation into a configurable `target_frame`.
- **Manipulator handoff:** connected selected perception targets to task-level arm commands and target poses; documented interfaces for the manipulation and navigation teams.
- **Testing and debugging:** built RViz markers, image overlays, timestamped run recording, and rosbag replay tools to inspect detections and task-state transitions.

## Perception Pipeline

The runtime package processes separate white-bin and full-colour detection streams. Depth provides both a target position and an approximate physical size, allowing the pipeline to distinguish blocks from bins. A target selector switches between white-bin search, block search, and colour-bin search according to the task state.

| Component | Implementation |
| --- | --- |
| Colour segmentation | [`color_blob_detector.py`](contribution/ros2_color_blob_vision/color_blob_vision/color_blob_detector.py): HSV thresholds, morphology, ROI/area filtering, and image-plane orientation |
| RGB-D projection and tracking | [`blob_depth_to_3d_smoothed.py`](contribution/ros2_color_blob_vision/color_blob_vision/blob_depth_to_3d_smoothed.py): depth sampling, metric-size classification, association, and temporal filtering |
| Target selection and frames | [`perception_manager.py`](contribution/ros2_color_blob_vision/color_blob_vision/perception_manager.py): task-dependent selection, TF transformation, and workspace-bound checks |
| Task interface | [`task_manager.py`](contribution/ros2_color_blob_vision/color_blob_vision/task_manager.py): consumes selected targets, sequences task states, and publishes arm/navigation requests |
| Launch configuration | [`vision_pipeline_manager.launch.py`](contribution/ros2_color_blob_vision/launch/vision_pipeline_manager.launch.py): detection streams, managers, optional visualisation/recording, and camera-mount TF parameters |

### Downstream Interfaces

| Output | Type | Purpose |
| --- | --- | --- |
| `/white/color_blobs_3d`, `/full/color_blobs_3d` | `vision_msgs/Detection3DArray` | Filtered 3D detections with `block:<color>` / `bin:<color>` labels |
| `/task/current_target_pose` | `geometry_msgs/PoseStamped` | Selected target in the configured frame |
| `/task/current_target_label`, `/task/current_target_status` | `std_msgs/String` | Target identity and selection/TF status |
| `/task/current_target_visible`, `/task/current_target_reachable` | `std_msgs/Bool` | Visibility and geometric workspace checks |
| `/task/arm_command`, `/task/arm_target_pose` | `std_msgs/String`, `geometry_msgs/PoseStamped` | Semantic arm request and target pose |
| `/task/nav_goal_name` | `std_msgs/String` | Semantic navigation request |

The reachability flag uses coordinate bounds, not inverse kinematics or collision checking. Pose orientation is derived from the image-plane contour angle; grasp planning remains a downstream responsibility.

Integration notes: [manipulator handoff](contribution/ros2_color_blob_vision/manipulator_handoff.md) · [navigation handoff](contribution/ros2_color_blob_vision/navigation_handoff.md) · [offline and real-robot configuration](contribution/ros2_color_blob_vision/offline_vs_real_pipeline.md).

## Engineering Decision: YOLO to HSV-first

I initially explored YOLO for shape detection with HSV colour classification inside each detected region. As system integration progressed, navigation, manipulation, and perception needed to share compute. The task used controlled target colours, making colour segmentation with depth and temporal filtering a practical fit.

After discussing the trade-off with the supervisor, I selected an HSV-first runtime pipeline to reduce inference dependencies and simplify on-robot tuning and debugging. The earlier [YOLO + HSV inference tools](contribution/vision_tools/infer_yolo_hsv.py) remain in the repository alongside dataset preparation and calibration scripts.

See the [design note](contribution/notes/YOLO_TO_HSV_MIGRATION_SUMMARY.md) for the implementation choices and trade-offs.

## Demonstrations and Validation

- [Main pipeline replay](evidence/videos/main_pipeline_overlay_007.mp4)
- [Block detection overlay](evidence/videos/block_pipeline_overlay.mp4)
- [Bin detection overlay](evidence/videos/bin_pipeline_overlay.mp4)

The recorded-data workflows cover perception outputs, RGB-D processing, and task-state debugging. Supporting tools include ROS 2 bag export, 2D/3D message extraction, timestamp-aligned video rendering, and offline colour/shape evaluation.

[Evaluation results and reproduction requirements](evidence/docs/vision_requirement_evidence.md) · [Recording and overlay workflow](evidence/docs/rosbag_offline_overlay_workflow.md)

### Integration Status

The perception nodes, target-pose transformation logic, and downstream command interfaces are implemented. The final perception-to-physical-grasp feedback loop was not completed during the project: the task prototype uses simulated navigation/arm completion timing and still needs real executor-result handling. Camera-mount transforms are configurable assumptions requiring hardware calibration and validation.

Source bags, labelled datasets, and model weights are not bundled here. The evaluation note records the available results and the inputs needed to reproduce them.

## Repository Layout

- `contribution/ros2_color_blob_vision/` — ROS 2 runtime nodes, launch files, and integration notes
- `contribution/vision_tools/` — early YOLO experiments, dataset preparation, and HSV calibration
- `contribution/evidence_tools/` — recording, evaluation, and visualisation scripts
- `contribution/notes/` — perception design decisions
- `evidence/` — demonstration videos and evaluation notes

The launch files retain paths from the original development workspace; update the YAML paths and camera topics for a new setup.

## Team and License

The original team roles were Design — Tom; Vision — XuYu and Wangsiyuan; Manipulator — Lyuxingze; Navigation — Ansber. This repository contains my perception work from the team's `Xy` workspace.

Source mappings are listed in [ATTRIBUTION.md](ATTRIBUTION.md). The original [MIT license](LICENSE) is preserved.
