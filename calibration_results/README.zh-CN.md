# 已发布的头部及右腕相机参数

[English](README.md)

于 2026-10-08 从 `/home/dc/tron2_deployment_on_dc/sp_vision/data` 上传。这些是该本地目录中各相机最新可用的结果，并非重新运行标定。四份 JSON 文件均逐字节复制。[manifest.json](manifest.json) 记录来源路径、来源修改时间、SHA-256 校验值及已有独立验证摘要。来源工作区提交仅标识代码版本；原始结果文件被 Git 忽略。

## 文件与来源

下表日期时间均为 Asia/Shanghai（UTC+08:00）的来源文件修改时间，并非 JSON 内嵌的求解时间。头部结果来自实际的 `head-camera-session` 目录（连字符），当前指南示例则使用 `head_camera_session`（下划线）。2026-09-23 的旧腕部 `intrinsics.json` 和 `extrinsics.json` 已被 2026-10-05 的 `wrist_*` 结果取代。本次上传只有右腕的结果。

| 结果 | 发布文件 | 相对部署仓库根目录的本地来源 | 来源最后修改时间 |
| --- | --- | --- | --- |
| 头部内参 | [head_intrinsics.json](head/2026-09-23/head_intrinsics.json) | `sp_vision/data/head-camera-session/intrinsics.json` | 2026-09-23 20:15:24 |
| 头部外参 | [head_extrinsics.json](head/2026-09-23/head_extrinsics.json) | `sp_vision/data/head-camera-session/extrinsics.json` | 2026-09-23 20:16:38 |
| 右腕内参 | [wrist_intrinsics.json](right_wrist/2026-10-05/wrist_intrinsics.json) | `sp_vision/data/wrist_camera_session/wrist_intrinsics.json` | 2026-10-05 19:42:24 |
| 右腕外参 | [wrist_extrinsics.json](right_wrist/2026-10-05/wrist_extrinsics.json) | `sp_vision/data/wrist_camera_session/wrist_extrinsics.json` | 2026-10-05 20:23:37 |

## 已记录的质量与验收状态

| 相机 | 图像尺寸 | 内参 RMS（px） | 内参/外参 `passed` | 独立触点最大误差 | 触点阈值 | 独立验证 `passed` |
| --- | --- | --- | --- | --- | --- | --- |
| 头部 | 640×480 | 0.072941 | true / true | 14.421725 mm | 10 mm | false |
| 右腕 | 640×480 缩放图像 | 0.211810 | true / true | 12.260710 mm | 10 mm | false |

这些数值来自本次发布时检查的已有本地记录，并非新测量的硬件结果。拟合结果的 `passed: true` 不代表独立物理精度达标。两次已记录触点检查均未达到 10 mm 阈值；这些参数仅作为标定证据，尚未通过要求 ≤10 mm 精度的任务验收。来源报告路径和校验值在清单中；图像、原始关节状态样本、TCP 结果、机器人配置和凭据不发布。

## 单位与坐标系

`camera_matrix` 为像素内参；`distortion` 是 OpenCV 的五系数向量 `[k1, k2, p1, p2, k3]`。应使用与标定一致的 640×480 彩色图像几何；不要将这些内参直接用于其他分辨率、裁剪方式或深度流。腕部缩放彩色流为 `/camera/right/color/image_resized/compressed`，而源 CameraInfo 的分辨率不同。

`T_A_B` 将 B 坐标系的坐标转换到 A。齐次变换为 4×4，平移单位为米。光学轴为 +x 向右、+y 向下、+z 向前。头部变换 `T_pitch_camera` 从头部彩色光学坐标系转换到 `head_pitch_Link`。右腕变换 `T_wrist_roll_camera` 从 `right_wrist_camera_color_optical_frame` 转换到 `wrist_roll_R_Link`。采集记录中的 ROS 坐标系为 `right_color_optical_frame`；应检查现场 TF，不要假设已存在相应别名。头部结果没有内嵌相机坐标系 ID；其解释依据头部标定流程。

```text
p_A = T_A_B · p_B
T_base_head_camera(q) = T_base_pitch(q) · T_pitch_camera
T_base_right_wrist_camera(q) = T_base_wrist_roll(q) · T_wrist_roll_camera
```

应使用图像同步的关节角及匹配的运动学模型。`T_base_board` 是各自标定会话的固定棋盘位姿，并非通用物体位姿。`nominal_T_*` 是模型参考值，不是实测相机外参。来源部署仓库的头部指南（来源工作区中的 `sp_vision/head_calib.md`）记录了后续 D455 模型更新；9 月的头部 JSON 保留较早的名义变换和一致性检查，因此其中 `model_consistency` 通过并不代表兼容当前模型。验收前应针对实际使用的模型重新求解并执行独立验证。

桌面任务须遵循[固定桌面坐标系约定](../TABLETOP_FRAME_CONTRACT.zh-CN.md)（[English](../TABLETOP_FRAME_CONTRACT.md)）：物理基座原点高于桌面 0.45 m。本次发布既不改变安装几何，也不验证历史采集是否满足该约定。

## 加载发布文件

从标定仓库根目录运行；仅使用 Python 标准库即可查看文件。加载 JSON 不会连接硬件或发送指令。

```python
import json
from pathlib import Path

root = Path("calibration_results")
head_intrinsics = json.loads((root / "head/2026-09-23/head_intrinsics.json").read_text())
head_extrinsics = json.loads((root / "head/2026-09-23/head_extrinsics.json").read_text())
wrist_intrinsics = json.loads((root / "right_wrist/2026-10-05/wrist_intrinsics.json").read_text())
wrist_extrinsics = json.loads((root / "right_wrist/2026-10-05/wrist_extrinsics.json").read_text())
K_head = head_intrinsics["camera_matrix"]
D_head = head_intrinsics["distortion"]
T_pitch_camera = head_extrinsics["T_pitch_camera"]
K_wrist = wrist_intrinsics["camera_matrix"]
D_wrist = wrist_intrinsics["distortion"]
T_wrist_roll_camera = wrist_extrinsics["T_wrist_roll_camera"]
```

使用结果时保留原始 JSON 结构和数值精度。完整字段解析及重新标定步骤见[头部指南](../sp_vision/head_calib.zh-CN.md)与[腕部指南](../sp_vision/wrist_calib.zh-CN.md)。
