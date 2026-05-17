# Robotic Systems Design

This repository is a portfolio-style showcase of my contribution to the team project **AERO62520 Robotic Systems Design Project**.

My goal in this repository is to make my own work easy to review. Rather than presenting the entire team codebase, I focus on the parts I designed, implemented, refined, and documented myself, especially the vision and perception pipeline used to support the robot's task flow.

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

## My Contribution At A Glance

My contribution focused on building a perception subsystem that was not just able to detect colors in an image, but could provide outputs that were much more useful for a full robotics pipeline.

I worked on:

- designing and implementing a ROS 2 perception package for task-oriented color detection
- building HSV-based detection workflows for white bins and colored blocks
- converting 2D image detections into 3D spatial estimates using depth information
- improving reliability through smoothing, filtering, and multi-frame confirmation logic
- structuring launch files and manager-style components to make the pipeline easier to run and integrate
- preparing tooling for dataset inspection, HSV tuning, inference support, and evaluation
- documenting migration decisions and integration handoff details for the wider project

In short, I pushed this part of the project from a basic visual detection idea toward a more usable and integration-ready perception workflow.

## Team Context

According to the original repository README, the project work was split across several roles:

- Design: Tom
- Vision: XuYu
- Manipulator: Lyuxingze
- Navigation: Ansber
- State Machine: Wangsiyuan

This repository focuses on the **vision/perception contribution** that I completed.

## What I Built

My work centered on the `Xy` section of the original repository, especially the perception pipeline and the supporting tooling used to prepare, debug, and refine it.

The most important thing I contributed was a practical perception workflow that could serve real robot behavior rather than remaining an isolated computer vision demo.

That included:

- a ROS 2 package for color-based perception
- RGB-D based 3D position estimation from 2D detections
- task-oriented classification logic for blocks and bins
- launch configurations for different operating stages
- debugging and visualization utilities
- supporting scripts for calibration, dataset handling, and offline experimentation
- technical documentation explaining both implementation and design choices

## My Technical Approach

My design approach was shaped by a practical robotics constraint: perception is only useful if downstream modules can consume it reliably.

Because of that, I did not focus only on detecting colored regions. I focused on making perception outputs:

- lightweight enough to run practically
- interpretable and easy to debug
- stable enough to reduce frame-to-frame flicker
- structured in a way that navigation and manipulation modules could actually use

This is also why part of my work involved moving toward an HSV-first pipeline in this task setting. A simpler and more controllable perception pipeline can be a better engineering choice than a heavier model-based approach when the task is strongly color-driven and integration reliability matters.

## Problems I Helped Solve

The contribution in this repository was built around several concrete robotics problems:

- how to detect task-relevant targets without depending on a heavy inference stack for every step
- how to transform 2D detections into 3D positions meaningful for robot action
- how to reduce unstable detections that would otherwise be difficult for downstream modules to use
- how to distinguish semantically useful objects such as bins and blocks instead of only reporting raw image blobs
- how to package the vision pipeline so it could be launched, inspected, and handed off more cleanly within a team project

These problems shaped both the code and the supporting documentation included here.

## Selected Technical Highlights

### 1. ROS 2 Perception Package

The `contribution/ros2_color_blob_vision/` folder contains the curated ROS 2 package and related notes from my contribution. This part includes:

- `color_blob_detector.py` for HSV-based color segmentation and blob detection
- `blob_depth_to_3d.py` and `blob_depth_to_3d_smoothed.py` for RGB-D based 3D projection
- launch files for different runtime modes
- manager and debugging utilities
- handoff notes for integration with other subsystems

My work here was aimed at turning raw image detections into outputs that were much more useful for robot behavior, integration, and debugging.

The core value of this package is not just that it detects colored objects. It is that it tries to bridge sensing and action by producing structured outputs that better match what the robot actually needs downstream.

### 2. Vision Tooling

The `contribution/vision_tools/` folder contains supporting scripts for:

- dataset analysis and splitting
- frame extraction from RealSense recordings
- HSV calibration and adjustment
- color evaluation and combined inference workflows

These tools supported experimentation, debugging, iteration, and parameter tuning during development. They reflect a part of my workflow that matters in robotics practice: building the surrounding tools needed to improve system quality, not only writing the runtime nodes.

### 3. Migration Notes

The file `contribution/notes/YOLO_TO_HSV_MIGRATION_SUMMARY.md` documents a key technical transition from a heavier YOLO-plus-HSV approach toward a more lightweight HSV-first pipeline for this task setting.

I included this note because it captures an important part of my engineering thinking: choosing an approach that better fits the task, deployment constraints, and debugging needs rather than defaulting to the more complex option.

## Why I Think This Work Is Valuable

What I am most proud of in this contribution is that it goes beyond isolated vision logic.

I contributed work that tried to improve the full engineering usability of perception by:

- connecting image-level detection with spatial reasoning
- improving robustness rather than relying on single-frame outputs
- making the system easier to launch, inspect, and explain
- leaving behind documentation that helps other people understand and reuse the work

From a portfolio perspective, I see this project as a strong example of how I approach robotics software:

- I build for integration, not only for demos
- I care about practical tradeoffs, not only technical complexity
- I try to make systems easier to debug, evaluate, and extend
- I document decisions so the work is usable by others

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
