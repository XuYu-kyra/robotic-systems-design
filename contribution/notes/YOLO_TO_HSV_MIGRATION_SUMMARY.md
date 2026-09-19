# 从 YOLO + HSV 到 HSV-first 的系统级技术取舍

## 1. 结论与证据边界

本次迁移不是“YOLO 做不出来，所以退回简单视觉”。前期工作保留了可运行的 YOLO + HSV 推理脚本、数据集处理和 HSV 标定工具，说明 learning-based detector 是实际探索过的方向。

后来在真实机器人系统集成阶段，导航、机械臂和感知需要共享计算资源；任务场景中的目标颜色又较受控。与老师讨论后，运行时主线因此改为依赖更少、参数更透明、现场更容易调试的 HSV-first pipeline。这是 **resource-aware / system-level engineering trade-off**。

重要限制：本仓库没有保存原机器人或开发机上的成对延迟 benchmark、完整 profiling 日志或部署记录。因此，下文不把任何具体帧耗时写成项目实测结果，也不声称在原机器人上部署了 YOLO/TensorRT。

## 2. 两条技术路线

### 2.1 前期：YOLO + HSV

保留证据：

- `contribution/vision_tools/infer_yolo_hsv.py` 使用 Ultralytics YOLO 产生目标框。
- `HSVColorEstimator` 在检测框 ROI 内估计颜色。
- `rs_bag_to_frames.py`、数据集拆分、标注和分析脚本支持训练与实验流程。

典型输出把语义类别和颜色组合起来。该方向的优势是语义/形状能力更强，但需要模型权重和推理运行时；本 portfolio 仓库没有包含训练权重，也没有证明它曾在原机器人上正式部署。

### 2.2 后期主线：HSV-first RGB-D perception

运行时流程：

1. 对 RGB 图像做 HSV 多区间分割。
2. 使用开/闭运算去噪并提取轮廓。
3. 通过面积、ROI 和几何信息过滤候选。
4. 将 2D 中心点与对齐深度、相机内参结合，投影到 3D。
5. 根据深度和估算尺寸生成 `block:<color>` / `bin:<color>`。
6. 使用轻量级关联、EMA 平滑和多帧确认提高时序稳定性。
7. 通过 `perception_manager` 选择目标并发布 `PoseStamped`。

主要代码：

- `color_blob_detector.py`
- `blob_depth_to_3d.py`
- `blob_depth_to_3d_smoothed.py`
- `perception_manager.py`
- `task_manager.py`

## 3. 为什么 HSV-first 更适合当时的系统阶段

### 3.1 共享计算与部署约束

YOLO 路线需要模型权重、Ultralytics/PyTorch 等依赖并执行神经网络推理。HSV 路线主要依赖 OpenCV 和 YAML 阈值。一般来说，后者计算路径更轻、依赖更少；但由于仓库内没有同机 benchmark，只能把这写成架构层面的预期，不能写成项目测得的毫秒数字。

### 3.2 任务先验

该任务的目标颜色和场景相对受控，颜色本身就是关键任务特征。在这一条件下，HSV 能以较低的系统复杂度提供可用候选，再利用 RGB-D 尺寸、深度和时序信息补足仅靠颜色的不足。

### 3.3 集成与调试

HSV 阈值、ROI、面积、深度和确认帧数都能直接配置和可视化，适合在 RViz、debug image 和 rosbag replay 中快速定位误检、漏检或深度异常。对共享机器人平台而言，可解释、可调整的失败模式本身就是工程价值。

## 4. Trade-offs

| 维度 | YOLO + HSV | HSV-first RGB-D |
| --- | --- | --- |
| 语义/形状能力 | 可由训练类别提供 | 依赖颜色、几何和尺寸规则 |
| 运行时依赖 | 模型权重 + ML runtime + OpenCV | OpenCV + YAML + ROS 2 RGB-D inputs |
| 预期计算负载 | 通常较高，具体取决于模型和硬件 | 通常较低，具体取决于分辨率和处理配置 |
| 调参方式 | 数据、训练配置、置信度阈值 | HSV/ROI/面积/深度/时序参数 |
| 光照变化 | 通常更有潜力，但取决于训练数据 | 更敏感，需要标定和验证 |
| 背景同色物体 | 语义模型可能更有优势 | 需要 ROI、3D、几何及时序过滤 |
| 本仓库证据 | 推理与数据工具；无权重、无部署 benchmark | ROS 2 runtime nodes、RGB-D projection、smoothing、launch 与离线证据 |

## 5. 已实现的补强

- ROI 与最小面积过滤
- 深度范围和横向工作区过滤
- 由 bbox、深度和内参估算 3D 尺寸
- block/bin 规则分类
- patch median depth
- 多帧确认、miss tolerance、3D 距离关联
- 位置、yaw 和 score 的指数平滑
- target selection、reachability flag、TF lookup 与 `PoseStamped` 输出

## 6. 仍然存在的限制

- HSV 对光照和同色背景敏感。
- 3D 尺寸分类使用经验阈值，不等于经过完整标定的语义分类器。
- 仓库没有保存可以复核 YOLO 与 HSV 性能差异的同机 benchmark。
- TF 安装参数仍含工程假设；没有完成最终手眼标定和真实机械臂闭环验证。
- `task_manager` 的保留版本发布 nav/arm 命令，但没有消费真实执行成功/失败反馈。

## 7. 正确的项目表述

推荐：

> Explored a YOLO + HSV perception path, then selected an HSV-first RGB-D pipeline as a resource-aware system trade-off for a controlled-colour mobile-manipulation task. Implemented ROS 2 2D/3D outputs, temporal stabilisation, target-pose/TF integration logic, and downstream handoff interfaces.

不要表述为：

- failed YOLO model, so switched to simple vision
- benchmarked HSV below 10 ms on the robot
- deployed YOLO/TensorRT on the original robot
- completed end-to-end physical grasping
