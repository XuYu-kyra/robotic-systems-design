# Vision Requirement Evidence — Provenance and Limitations

This note separates retained artifacts from claims that can be independently reproduced from this portfolio repository.

## Evidence boundary

The repository contains the evaluation/rendering scripts, three overlay videos, a generated requirement-summary image, and the ROS 2 perception code. It does **not** contain the labelled datasets, ground-truth CSV files, YOLO weights, or source RealSense/ROS 2 bags referenced by the original absolute paths.

Therefore:

- the numerical results below are historical results recorded from the original local workspace;
- they are not independently reproducible from this portfolio repository alone;
- the overlay videos are qualitative demonstrations of recorded-data processing, not proof of physical grasp success;
- the generated summary card contains hard-coded historical metrics and should not be treated as a raw experiment log;
- position accuracy and end-to-end task success remain unsupported.

One preserved inconsistency is also important: `contribution/vision_tools/color_evaluate.py` is a YOLO + HSV evaluator and does not implement the HSV-only command interface previously shown in this note. The HSV/blob logic used by the later evidence scripts is preserved elsewhere in `contribution/evidence_tools/`, but the original datasets required to rerun the reported numbers are absent. The commands previously shown are therefore not presented as reproducible portfolio instructions.

## Historical reported results

| Requirement | Historical result | Portfolio evidence status |
| --- | --- | --- |
| Block colour accuracy ≥ 60% | 2,277 GT instances; 74.70% accuracy | Reported from original workspace; dataset/CSV not packaged |
| Block colour F1 ≥ 0.6 | weighted F1 0.7690 | Reported from original workspace; dataset/CSV not packaged |
| Shape precision ≥ 0.5 | 141 held-out GT instances; precision 0.5563 | Evaluation code retained; train/test data not packaged |
| Blob IoU > 0.5 | 114/141 matched at IoU ≥ 0.50; mean IoU 0.7432 | Evaluation code retained; test data not packaged |
| Bin colour precision ≥ 0.5 | red/blue subset weighted precision 1.0000 | Partial historical subset only; yellow images were already missing |
| Bin colour F1 ≥ 0.6 | red/blue subset weighted F1 0.9981 | Partial historical subset only; not full three-colour evidence |
| Position error within ±50 mm | No calibrated 3D ground truth | Not supported |
| End-to-end matching accuracy > 70% | No repeated task-level success log | Not supported |

## Original-workspace data references

Historical block-colour evaluation referred to:

- `color_gt.csv`
- `datasets/shapes/images_all/images`
- local HSV/blob evaluation tooling

Historical shape/IoU evaluation referred to:

- `datasets/shapes/cube_seperated_dataset/images/train`
- `datasets/shapes/cube_seperated_dataset/labels/train`
- `datasets/shapes/cube_seperated_dataset/images/test`
- `datasets/shapes/cube_seperated_dataset/labels/test`
- `contribution/evidence_tools/shape_evaluate.py`

Historical bin-colour evaluation referred to `color_gt_bin.csv`. The recorded CSV summary was 499 rows / 497 unique filenames, but only 262 referenced red/blue images were locally available at the time; 235 yellow images were missing. The red/blue result must not be presented as full red/blue/yellow validation.

## What the retained artifacts do demonstrate

- HSV-based 2D detections and task labels on recorded imagery
- aligned-depth and camera-intrinsics code paths for 3D localisation
- `Detection3DArray` output extraction and recorded-run tooling
- multi-frame smoothing/confirmation in the ROS 2 node
- debug overlays and pipeline-state visualisation
- a task-state prototype driven by recorded perception inputs

## What they do not demonstrate

- calibrated ±50 mm 3D accuracy
- a validated camera-to-`base_link` transform on the final robot
- real navigation or arm completion feedback
- end-to-end physical grasp success
- deployment-time YOLO/HSV latency on the original hardware

## Retained files

- `evidence/videos/main_pipeline_overlay_007.mp4`
- `evidence/videos/block_pipeline_overlay.mp4`
- `evidence/videos/bin_pipeline_overlay.mp4`
- `evidence/images/vision_requirement_summary_requirements_videos.png`
- `contribution/evidence_tools/`
- `evidence/docs/rosbag_offline_overlay_workflow.md`
