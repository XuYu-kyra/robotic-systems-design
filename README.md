# Robotic Systems Design

This repository is a portfolio-style showcase of my contribution to the team project **AERO62520 Robotic Systems Design Project**.

It is not a mirror of the full team repository. Instead, it presents:

- a concise summary of the overall project
- the parts I personally worked on
- selected source files and technical notes from my contribution
- a clear link back to the original team repository

## Original Team Repository

Source project:

- https://github.com/Picklerick313/AERO62520_Robotic_Systems_Design_Project

## Project Overview

The original project combines **mobile robotics, perception, navigation, and manipulation** into a single robotic system built around a **Leo Rover** and **myCobot 280 Pi** platform.

At a high level, the project aimed to build a robot that could:

- perform SLAM and autonomous navigation
- detect task-relevant objects and bins using vision
- estimate object positions for downstream robot actions
- support pick-and-place style task flows
- integrate multiple subsystems through ROS 2

The main technology stack described in the team repository includes:

- ROS 2 Jazzy
- Nav2
- MoveIt 2
- Cartographer
- RGB-D perception tools

## Team Context

According to the original repository README, the project work was split across several roles:

- Design: Tom
- Vision: XuYu
- Manipulator: Lyuxingze
- Navigation: Ansber
- State Machine: Wangsiyuan

This repository focuses on the **vision/perception contribution** that I completed.

## My Contribution

My work centered on the `Xy` part of the original repository, especially the perception pipeline and the supporting tooling used to prepare, test, and refine it.

The main areas I contributed were:

- building a ROS 2 color-based perception pipeline for task-relevant targets
- developing HSV-based detection for white bins and colored blocks
- projecting 2D detections into 3D using aligned depth data
- improving stability with smoothing and confirmation logic
- organizing launch files and manager-style pipeline components for integration
- preparing supporting tooling for dataset handling, HSV calibration, and offline evaluation
- writing technical notes to explain migration and integration decisions

## Selected Technical Highlights

### 1. ROS 2 Perception Package

The `contribution/ros2_color_blob_vision/` folder contains the curated ROS 2 package and related notes from my contribution. This part includes:

- `color_blob_detector.py` for HSV-based color segmentation and blob detection
- `blob_depth_to_3d.py` and `blob_depth_to_3d_smoothed.py` for RGB-D based 3D projection
- launch files for different runtime modes
- manager and debugging utilities
- handoff notes for integration with other subsystems

This work was aimed at turning raw image detections into outputs that were more useful for robot behavior and integration.

### 2. Vision Tooling

The `contribution/vision_tools/` folder contains supporting scripts for:

- dataset analysis and splitting
- frame extraction from RealSense recordings
- HSV calibration and adjustment
- color evaluation and combined inference workflows

These tools supported experimentation, debugging, and iteration during development.

### 3. Migration Notes

The file `contribution/notes/YOLO_TO_HSV_MIGRATION_SUMMARY.md` documents a key technical transition from a heavier YOLO-plus-HSV approach toward a more lightweight HSV-first pipeline for this task setting.

## Repository Structure

```text
.
├── README.md
├── ATTRIBUTION.md
├── LICENSE
└── contribution
    ├── ros2_color_blob_vision
    ├── vision_tools
    └── notes
```

## Attribution and Scope

This was a **team project**, and I am not claiming sole authorship of the entire system.

This repository is intended to make my own contribution easier to review by:

- summarizing the full project at a high level
- isolating the files most closely related to my work
- preserving a clear reference to the original project source

For the complete project context, team structure, and the full repository contents, please refer to the original team repository:

- https://github.com/Picklerick313/AERO62520_Robotic_Systems_Design_Project

## License

Selected files in this repository are derived from the original team repository, which includes an MIT license. The original license text is preserved in [`LICENSE`](LICENSE), and additional source attribution is documented in [`ATTRIBUTION.md`](ATTRIBUTION.md).
