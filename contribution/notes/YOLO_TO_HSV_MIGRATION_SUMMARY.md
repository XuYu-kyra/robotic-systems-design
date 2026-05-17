# 从 YOLO 到 HSV 颜色识别的技术迁移总结

## 一、核心改动对比

### 1.1 旧方案：YOLO + HSV 混合检测

**节点文件**：`ros_nd/src/color_vision/color_vision/color_object_detector.py`

**工作流程**：
1. 使用 **YOLO 模型**（`ultralytics`）对整幅图像做**形状/物体检测**（例如检测到“积木块”或“bin”的边界框）
2. 对每个 YOLO 检测框内的 ROI，再用 **HSV 颜色估计器**判断颜色
3. 最终输出格式：`{target_type}:{shape_name}:{color_name}`，例如 `"object:block:red"`

**依赖**：
- `ultralytics`（YOLO 模型库）
- 预训练的 YOLO 权重文件（`.pt`）
- `HSVColorEstimator`（来自 `tools/color_utils.py`）

**输出话题**：`/detected_objects` (Detection2DArray)

---

### 1.2 新方案：纯 HSV 颜色块检测

**节点文件**：`ros_nd2/src/color_blob_vision/color_blob_vision/color_blob_detector.py`

**工作流程**：
1. **直接对整幅图像做 HSV 颜色分割**（使用 `cv2.inRange` + 多段阈值）
2. **形态学去噪**（开运算 + 闭运算）
3. **轮廓检测**（`cv2.findContours`）
4. **面积过滤**（过滤小于 `min_area` 的小块）
5. **计算几何信息**：中心点（图像矩）、外接矩形、朝向角度（最小外接矩形）
6. 输出格式：`"blob:{color_name}"`，例如 `"blob:red"`

**依赖**：
- 仅需 OpenCV (`cv2`)
- YAML 配置文件（`color_ranges.yaml`）定义 HSV 阈值区间

**输出话题**：`/color_blobs` (Detection2DArray)

---

## 二、为什么要做这个改动？

### 2.1 主要动机

1. **简化依赖和部署**
   - 旧方案需要：YOLO 模型权重（通常几十到几百 MB）、PyTorch/Ultralytics 库、GPU 或 CPU 推理时间
   - 新方案只需要：OpenCV（通常已预装）、一个小的 YAML 配置文件

2. **降低计算开销**
   - YOLO 推理：每帧需要神经网络前向传播，CPU 上可能 50-200ms/帧
   - HSV 分割：纯像素级操作，通常 <10ms/帧，实时性更好

3. **更灵活的颜色配置**
   - YOLO 方案：颜色判断在 ROI 内做，但形状检测依赖训练数据，难以快速调整
   - HSV 方案：直接通过 YAML 调整颜色阈值，无需重新训练模型

4. **任务适配性**
   - 对于“按颜色找积木块/bin”这个任务，**颜色是主要区分特征**，形状检测是“过度设计”
   - 如果只需要区分颜色，HSV 分割已经足够

---

## 三、好处（Advantages）

### 3.1 性能优势
- ✅ **速度快**：HSV 分割 + 轮廓检测通常 <10ms/帧，YOLO 需要 50-200ms/帧
- ✅ **资源占用低**：不需要加载大型模型，内存占用小
- ✅ **实时性好**：适合高帧率相机（30fps+）

### 3.2 工程优势
- ✅ **部署简单**：不需要分发模型权重文件，只需 YAML 配置
- ✅ **易于调试**：HSV 阈值直观，可以直接在图像上看到分割效果
- ✅ **跨平台**：OpenCV 比 PyTorch 更容易在嵌入式设备上部署

### 3.3 灵活性优势
- ✅ **快速调整**：修改 YAML 文件即可调整颜色范围，无需重新训练
- ✅ **多段阈值**：可以为一个颜色定义多个 HSV 区间（例如红色跨 0° 和 180°）

---

## 四、缺点（Disadvantages）

### 4.1 鲁棒性问题
- ❌ **背景敏感**：如果背景中有相同颜色的物体，会被误检（例如红色墙壁、红色衣服）
- ❌ **光照敏感**：HSV 阈值在光照变化时可能失效，需要重新标定
- ❌ **无语义理解**：无法区分“这是积木块还是 bin”，只能按颜色分类

### 4.2 功能限制
- ❌ **无法区分物体类型**：只能输出 `"blob:red"`，不能直接区分是积木块还是 bin
- ❌ **形状信息有限**：只有外接矩形和朝向角度，没有更精细的形状特征

---

## 五、改进方法（无需 YOLO 即可显著提升鲁棒性）

### 5.1 ROI（感兴趣区域）过滤

**原理**：利用空间先验，只检测特定区域内的颜色块。

**实现思路**：
- 在 `ColorBlobDetector` 中添加 ROI 参数（例如：`roi_x_min, roi_x_max, roi_y_min, roi_y_max`）
- 在计算轮廓中心后，判断 `(u, v)` 是否在 ROI 内，不在则丢弃

**适用场景**：
- 已知积木块/bin 只出现在图像的下半部分（地面/桌面）
- 已知 bin 总是在画面右侧/左侧的固定区域

**代码示例**（在 `color_blob_detector.py` 中添加）：
```python
# 在 __init__ 中声明参数
self.declare_parameter("roi_x_min", 0.0)  # 归一化坐标 [0, 1]
self.declare_parameter("roi_x_max", 1.0)
self.declare_parameter("roi_y_min", 0.0)
self.declare_parameter("roi_y_max", 1.0)

# 在 image_callback 中过滤
roi_x_min = self.get_parameter("roi_x_min").value * w
roi_x_max = self.get_parameter("roi_x_max").value * w
roi_y_min = self.get_parameter("roi_y_min").value * h
roi_y_max = self.get_parameter("roi_y_max").value * h

if not (roi_x_min <= u <= roi_x_max and roi_y_min <= v <= roi_y_max):
    continue  # 跳过不在 ROI 内的检测
```

---

### 5.2 轮廓几何过滤

**原理**：利用积木块和 bin 的几何特征差异（面积、长宽比、紧凑度等）。

**实现思路**：
- **面积阈值**：积木块面积小（例如 <5000 像素²），bin 面积大（>20000 像素²）
- **长宽比**：积木块可能偏长条（长宽比 >1.5 或 <0.67），bin 更接近方形（长宽比接近 1.0）
- **紧凑度**：`area / (bbox_width * bbox_height)`，积木块可能更紧凑

**代码示例**（在 `color_blob_detector.py` 中）：
```python
# 在 image_callback 中，计算轮廓后
area = float(cv2.contourArea(cnt))
aspect_ratio = float(ww) / float(hh) if hh > 0 else 1.0
compactness = area / (ww * hh) if (ww * hh) > 0 else 0.0

# 过滤规则（可配置）
if area < 500 or area > 50000:  # 太小或太大都丢弃
    continue
if aspect_ratio > 3.0 or aspect_ratio < 0.33:  # 太细长
    continue
```

---

### 5.3 3D 工作空间/深度过滤

**原理**：结合深度图，利用 3D 空间信息过滤不合理的检测。

**实现思路**（在 `blob_depth_to_3d.py` 或新增过滤节点中）：
- **深度范围过滤**：只保留深度在合理范围内的点（例如 0.3m ~ 2.0m）
- **高度过滤**：利用 3D 坐标的 Z 值（高度），过滤掉离地/桌面太远的点（例如 Z > 0.5m 可能是墙上的反光）
- **工作空间过滤**：只保留机器人可达区域内的点（例如 X 在 [-1.0, 1.0]m，Y 在 [0.5, 2.0]m）

**代码示例**（在 `blob_depth_to_3d.py` 的 `synced_callback` 中）：
```python
# 在计算 3D 坐标后
if z3d <= 0.0 or z3d > 2.0:  # 深度不合理
    continue
if z3d < 0.1:  # 太近（可能是噪声）
    continue
if abs(x3d) > 1.0 or y3d < 0.3 or y3d > 2.5:  # 超出工作空间
    continue
```

---

## 六、区分 bin vs 积木块（Rule-based 分类）

### 6.1 推荐方案：3D 位置/深度 + 大小

**核心思路**：利用 bin 和积木块在 3D 空间中的尺寸差异和位置特征。

**判断规则**（在 `blob_depth_to_3d.py` 或新增分类节点中实现）：

1. **3D 尺寸判断**：
   - 从 2D bbox 的宽高和深度值，估算 3D 空间中的实际尺寸
   - 积木块：3D 尺寸通常 < 0.1m × 0.1m
   - bin：3D 尺寸通常 > 0.2m × 0.2m

2. **深度/高度判断**：
   - bin 通常更高（Z 值更大，或者从深度图看高度差明显）
   - 积木块通常放在地面/桌面上，Z 值较小

3. **位置先验**（可选）：
   - bin 可能总是在“起点附近”的固定区域
   - 积木块出现在场地其他区域

**实现代码**（在 `blob_depth_to_3d.py` 中修改 `synced_callback`）：

```python
# 在计算 3D 坐标后，估算 3D 尺寸
# 假设 bbox 中心深度为 z3d，像素尺寸为 ww, hh
# 使用相机内参估算实际尺寸
fx = float(self._camera_info.k[0])
fy = float(self._camera_info.k[4])

# 估算 3D 宽度和高度（米）
width_3d = (ww / fx) * z3d
height_3d = (hh / fy) * z3d
size_3d = max(width_3d, height_3d)

# 分类规则
if size_3d > 0.15:  # 大尺寸 -> bin
    object_type = "bin"
elif size_3d < 0.12:  # 小尺寸 -> 积木块
    object_type = "block"
else:
    object_type = "unknown"  # 中间值，可能需要其他特征

# 修改 class_id
original_class_id = hyp2d.hypothesis.class_id  # 例如 "blob:red"
color_name = original_class_id.split(":", 1)[1] if ":" in original_class_id else original_class_id
new_class_id = f"{object_type}:{color_name}"  # 例如 "bin:red" 或 "block:red"

hyp3d.hypothesis.class_id = new_class_id
```

---

### 6.2 标签格式变更

**旧格式**（YOLO 方案）：
- `"object:block:red"` 或 `"bin:bin:blue"`

**新格式（HSV + 3D 分类）**：
- `"block:red"`：红色积木块
- `"bin:red"`：红色 bin
- `"blob:red"`：未分类的红色块（如果分类规则不确定）

---

## 七、总结对比表

| 维度 | YOLO 方案 | HSV 方案 | HSV + 改进方案 |
|------|-----------|----------|----------------|
| **速度** | 50-200ms/帧 | <10ms/帧 | <15ms/帧 |
| **依赖** | PyTorch + 模型权重 | OpenCV | OpenCV + 深度图 |
| **背景鲁棒性** | 较好（有语义理解） | 弱 | 中等（ROI + 3D 过滤） |
| **光照鲁棒性** | 较好 | 弱 | 中等（可动态调整阈值） |
| **区分 bin/积木** | 可以（需训练） | 不可以 | 可以（rule-based） |
| **部署复杂度** | 高 | 低 | 中 |
| **可调性** | 需重新训练 | 改 YAML | 改 YAML + 参数 |

---

## 八、建议的实施路径

1. **第一阶段**：保持当前纯 HSV 方案，先验证颜色检测的基本功能
2. **第二阶段**：添加 ROI 过滤和轮廓几何过滤，提升背景鲁棒性
3. **第三阶段**：在 `blob_depth_to_3d.py` 中添加 3D 尺寸分类逻辑，区分 bin 和积木块
4. **第四阶段**（可选）：如果仍有误检，考虑添加简单的跟踪/时序过滤（例如：连续 N 帧都检测到才认为是有效目标）

---

## 九、相关文件清单

### 旧方案（YOLO）
- `ros_nd/src/color_vision/color_vision/color_object_detector.py`
- `tools/infer_yolo_hsv.py`
- `tools/color_utils.py`（HSVColorEstimator）

### 新方案（HSV）
- `ros_nd2/src/color_blob_vision/color_blob_vision/color_blob_detector.py`
- `ros_nd2/src/color_blob_vision/color_blob_vision/blob_depth_to_3d.py`
- `ros_nd2/src/color_blob_vision/color_blob_vision/color_blob_debug_image.py`
- `ros_nd2/src/color_blob_vision/color_blob_vision/color_blob_summary.py`
- `ros_nd2/src/color_blob_vision/color_blob_vision/color_blob_markers.py`
- `tools/color_ranges.yaml`（HSV 阈值配置）

