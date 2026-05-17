# Vision Requirement Evidence

This note records the current offline evidence that can be reproduced from the repository.

## Supported requirements

### 1.1 Block colour accuracy >= 60%

Data:
- Ground truth: `color_gt.csv`
- Images: `datasets/shapes/images_all/images`
- Method: current HSV/blob detector in `tools/color_evaluate.py`

Command:

```bash
./env_robot/bin/python tools/color_evaluate.py \
  --gt-csv /home/student24/robotproject/color_gt.csv \
  --img-dir /home/student24/robotproject/datasets/shapes/images_all/images \
  --hsv-cfg /home/student24/robotproject/tools/color_ranges.yaml
```

Result:
- Ground-truth instances: `2277`
- Accuracy: `74.70%`

Conclusion:
- Requirement `1.1` is supported by current offline evidence.

### 1.2 Block colour F1 >= 0.6

Same run as `1.1`.

Result:
- Weighted F1: `0.7690`

Conclusion:
- Requirement `1.2` is supported by current offline evidence.

### 2.1 Shape classification precision >= 0.5

Data:
- Train split: `datasets/shapes/cube_seperated_dataset/images/train`
- Train labels: `datasets/shapes/cube_seperated_dataset/labels/train`
- Test split: `datasets/shapes/cube_seperated_dataset/images/test`
- Test labels: `datasets/shapes/cube_seperated_dataset/labels/test`
- Classes: `cube`, `rectangle_prism`, `triangle_prism`, `cylinder`, `arch`

Method:
- HSV blob detection for candidate contours
- Contour-derived features: aspect ratio, extent, solidity, circularity, polygon vertex counts, Hu moments
- Lightweight contour-feature classifier in `tools/shape_evaluate.py`

Command:

```bash
./env_robot/bin/python tools/shape_evaluate.py
```

Result:
- Training contours extracted: `1377`
- Ground-truth instances in held-out test split: `141`
- Predicted blobs: `160`
- Shape precision over all predicted blobs: `0.5563`

Conclusion:
- Requirement `2.1` is supported by current offline evidence.

### 2.2 Blob bounding-box IoU > 0.5

Same run as `2.1`.

Result:
- Matched GT at IoU >= 0.50: `114/141` (`80.85%`)
- Mean IoU over all GT: `0.7432`
- Mean IoU over matched GT: `0.8683`

Conclusion:
- Requirement `2.2` is supported by current offline evidence.

### 3.1 Bin colour precision >= 0.5

Data currently available in the repository:
- Ground truth: `color_gt_bin.csv`
- Images found locally for this CSV: `262` instances
- Present colours in available images: `red`, `blue`

Command:

```bash
./env_robot/bin/python tools/color_evaluate.py \
  --gt-csv /home/student24/robotproject/color_gt_bin.csv \
  --img-dir /home/student24/robotproject/datasets/shapes \
  --hsv-cfg /home/student24/robotproject/tools/color_ranges.yaml
```

Result on available images:
- Ground-truth instances evaluated: `262`
- Red precision: `1.0000`
- Blue precision: `1.0000`
- Weighted precision: `1.0000`

Conclusion:
- Requirement `3.1` is supported for the available `red/blue` bin subset.
- Full three-colour support is not yet evidenced because the repository is missing the yellow-bin images referenced by `color_gt_bin.csv`.

### 3.2 Bin colour F1 >= 0.6

Same run as `3.1`.

Result on available images:
- Accuracy: `99.62%`
- Red F1: `1.0000`
- Blue F1: `0.9953`
- Weighted F1: `0.9981`

Conclusion:
- Requirement `3.2` is supported for the available `red/blue` bin subset.
- Full three-colour support is not yet evidenced because the repository is missing the yellow-bin images referenced by `color_gt_bin.csv`.

## Not supported by current repository evidence

### 2.3 Position error within +/-50 mm

Missing:
- Reliable 3D ground truth for object position

### 4.1 End-to-end block/bin matching accuracy > 70%

Missing:
- Task-level success records or an end-to-end labelled evaluation protocol

## Repository gap affecting bin-colour evidence

`color_gt_bin.csv` currently contains:
- `499` rows
- `497` unique filenames
- Colour distribution: `yellow 237`, `red 154`, `blue 108`

Images present locally for those filenames:
- `262` unique filenames found
- Found colours: `red 154`, `blue 108`
- Missing colours: `yellow 235`

So the current repository supports a strong bin-colour evaluation for `red/blue`, but cannot yet provide complete evidence for `red/blue/yellow` bins until the missing yellow-bin images are restored.
