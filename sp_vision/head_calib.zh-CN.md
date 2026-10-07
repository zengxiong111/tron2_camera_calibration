# 头部相机标定

[English](head_calib.md)

按 [README](../README.zh-CN.md) 跑一次 `scripts/install.sh` 后（正是该步骤生成 `.venv/bin/python`），先在仓库根目录执行 `source .venv/bin/activate`，再执行 `cd sp_vision` 运行以下脚本示例；下文命令中的 `python` 即该环境解释器。配置中的相对路径以 JSON 所在目录为基准，数据及结果路径以当前工作目录为基准。命令只读取传感器或执行离线求解，不驱动机器人。

## ROS 2 相机单帧诊断

`sp-vision-capture` 诊断工具通过 SSH 从 ROS 2 Foxy 主机读取一张彩色图像。它与头部主标定流程不同：头部默认 RGB-D 后端通过本机 ROS 1 Noetic 订阅；右腕标定则有独立的 ROS 2 图像和关节状态采集路径。该单帧诊断不保存深度或同步头部状态，因此不能把结果当作头部标定数据。检查 ROS 2 数据流：

```bash
python capture_ros2_image.py
```

脚本通过 SSH 使用传感器数据 QoS 订阅 `sensor_msgs/msg/CompressedImage`，保存到 `data/right-camera-latest.jpg`。加 `--camera top` 可读取对应的头部彩色话题 `/camera/top/color/image_raw/compressed`。两种方式都只读取图像；单帧快照不包含深度或关节状态，也不是经过标定的观测结果。

头部工作流负责标定：

1. 头部彩色相机内参；
2. 彩色光学相机坐标系到 `head_pitch_Link` 的刚体外参 `T_pitch_camera`；
3. 使用棋盘格内角点和机械臂触点进行独立验证。

采集时固定棋盘格并移动头部的 yaw、pitch。程序不控制头部、机械臂、灵巧手或夹爪；所有运动都通过机器人已有且经过审核的界面人工完成。代码和生成配置均使用 JSON，不使用 YAML。

## 模型快照与坐标约定

以下随包模型快照定义标定的运动学链：

- `configs/assembly.urdf`：运动学真值；
- `configs/scene.xml`：运动学链交叉检查；未随包提供引用的 mesh；
- `head_camera_color_optical_frame`：被标定的彩色光学坐标系，OpenCV 约定为 +x 向右、+y 向下、+z 向前；
- `head_pitch_Link`：与相机最近的可动 pitch 轴坐标系。

模型中还保留了一个直接挂在 `base_Link` 下的旧 `d435_Link`。本流程明确忽略它，只使用 `head_pitch_Link` 下的 `head_camera_color_optical_frame`。求解外参时，URDF 中的相机固定变换仅作为名义值对照，不作为求解约束。

所有矩阵采用 `T_A_B` 表示“把 B 坐标转换到 A 坐标”。头部完整链为：

```text
T_base_pitch(q_yaw, q_pitch)
  = T_base_headbase
  · T_headbase_yaw(q_yaw)
  · T_yaw_pitch_origin
  · R_pitch(q_pitch)
```

yaw→pitch 的固定偏移是 `[0.051, 0.03, 0.097] m`。因此 yaw 旋转会带着 pitch 轴原点移动；pitch 只绕自己的原点旋转，不会改变 yaw，也不会移动自身原点。虽然采集 JSON 的顺序是 `[head_pitch_Joint, head_yaw_Joint]`，程序先按关节名称建立映射，再依照 URDF 的父子链执行 yaw→pitch，不依赖数组顺序猜测。

每次外参求解都会检查 `configs/assembly.urdf` 和 `configs/scene.xml` 在三个不同头姿下的 `base→pitch` 与 `pitch→color optical` 变换；两者不一致时直接停止。

## 棋盘格与依赖

示例配置的棋盘为横向 7、纵向 10 个**内角点**，方格边长 `0.021 m`。即印刷图案应有 8×11 个方格，并在四周留至少约一个方格宽的白边。修改标定板后必须同步修改 JSON 中的三个参数。

棋盘参数也可以直接写在命令中：`--pattern` 始终表示“列数×行数”的内角点，`--square-m` 表示实测方格边长（米）。命令行值优先于 JSON；内参、外参和验证参数不一致时程序会拒绝继续。

从 `sp_vision/` 目录准备本地配置：

```bash
cp configs/head_config.example.json configs/head_config.json
cp configs/robot_profile.example.json configs/robot_profile.json
```

若本机配置选择 WebSocket 桥接后端而非 ROS Noetic，请在此目录运行 `python -m pip install -e '..[live]'` 安装该可选 Python 依赖。

将本地配置保留在示例旁边，使模型相对路径正确解析。实时采集前，将 `head_config.json` 的 `capture.profile` 改为 `robot_profile.json`，并填写该 profile 的相机内参、畸变、深度内参、深度到彩色变换和所选 ROS/bridge 连接参数。示例中的占位符和 null 不能直接用于采集。离线处理已有图像无需连接相机。记录控制器状态前，还要在该 profile 中填写 `robot.host` 和 `robot.port`（默认端口 `5000`），并让 `capture.state_profile` 指向它。路径示例：

```json
{
  "capture": {"profile": "robot_profile.json", "state_profile": "robot_profile.json", "timeout_s": 8},
  "robot": {
    "urdf": "assembly.urdf",
    "model_xml": "scene.xml",
    "pitch_link": "head_pitch_Link",
    "camera_frame": "head_camera_color_optical_frame"
  }
}
```

相对路径均相对于该配置 JSON 所在目录。Python 依赖通过上述安装命令提供；ROS 系统依赖和可选厂商传输环境需要另行准备。`state` 子命令读取控制器反馈，需要兼容的外部 `tron2_env` 环境；只做离线求解时无需安装。

## 一、采集约 40 个头姿

把棋盘刚性固定在相机和机械臂都能看到、触及的位置。整个内参和外参数据集内不得移动棋盘。运行：

```bash
python calibration.py \
  --config configs/head_config.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session data/head_camera_session --count 40
```

默认情况下，`--count` 是本轮重新采集的总张数。全部采完后，新图像将替换同一 `--session` 中原来的 `view-*` 目录，并从 `view-001` 重新编号；中途退出则保留原数据。如需有意续采，显式传入 `--append --count N`；例如已有 30 张时用 `--append --count 10` 追加到 40 张。两种采集方式结束后都需重新求解内参和外参，因为原有 JSON 结果仍对应之前的图像。

程序直接复用现有 profile 的 bridge 或 ROS 相机配置，同时读取与图像对应的 `[pitch, yaw]`。它不会沿用旧固定头姿检查。同步状态回退、关节/图像时间差超限或未完整检测 70 个角点的帧不能保存。

窗口按键：

- `s`：仅保存当前通过检查的图像和 JSON；
- `r`、空格或其他普通按键：丢弃当前帧并重新采集，不写照片；
- `f`：将角点顺序旋转 180°；当 `(0,0)` 没有落在与其他视角相同的实体角点时使用；
- `q` 或 Esc：退出。

预览使用彩色逐行连线，并标出四角的 `(row,column)`。每帧保存为：

```text
data/head_camera_session/
  view-001/
    color.png
    corners.png
    frame.json
  ...
```

保持棋盘不动，通过已有控制界面改变头部。采样应同时覆盖 yaw 和 pitch 的正负方向、不同组合，并在每次保存前等待头部完全停止。尽量覆盖不同的棋盘画面位置、倾角和成像大小；新增大量近乎相同的画面帮助很小。采满 40 张且通过质量检查时，默认将最后 6 张图像留作验证；采集时应使这些视角也有充分差异，其余 34 张参与拟合；质量剔除可能减少实际拟合张数。

检测器使用：

```text
findChessboardCornersSB(
  CALIB_CB_NORMALIZE_IMAGE |
  CALIB_CB_EXHAUSTIVE |
  CALIB_CB_ACCURACY
)
```

该检测器直接返回亚像素内角点，不再额外调用 `cornerSubPix`。程序还检查角点拓扑、棋盘画面覆盖率、边界距离和可选清晰度阈值。`corners.png` 是必须保留的人工复核证据。

## 二、求解内参

```bash
python calibration.py \
  --config configs/head_config.json intrinsics \
  --pattern 7x10 --square-m 0.021 \
  --session data/head_camera_session \
  --output data/head_camera_session/head_intrinsics.json
```

程序只用训练帧拟合 `K` 和五参数畸变，并用逐帧重投影误差剔除明显异常视角。剔除记录和原因写入 JSON，不会静默消失。结果还检查整个图像范围内的径向畸变映射是否保持单调；非单调结果视为标定失败。

查看重投影误差：
```bash
python - <<'PY'
import json

path = "data/head_camera_session/head_intrinsics.json"
result = json.load(open(path))

print("总体重投影 RMS:", result["rms_px"], "px")
for view, error in result["training_view_rms_px"].items():
    print(f"{view}: {error:.4f} px")
PY
```
重点检查：

- `passed` 必须为 `true`；
- `training_view_rms_px` 不应出现单帧明显突增；
- `radial_monotonicity.minimum_radial_derivative` 必须大于零；
- `diagnostics/intrinsics/*.png` 中所有角点顺序正确。

RMS 小并不能弥补姿态覆盖不足。焦距或主点与相机出厂值相差异常大时，应先检查棋盘尺寸、屏幕摩尔纹、角点方向和视角分布，不要继续求外参。优先使用平整的实体印刷板，不要把电脑屏幕上的棋盘作为最终标定板。

## 三、求解相机到 pitch 轴外参

```bash
python calibration.py \
  --config configs/head_config.json extrinsics \
  --pattern 7x10 --square-m 0.021 \
  --session data/head_camera_session \
  --intrinsics data/head_camera_session/head_intrinsics.json \
  --output data/head_camera_session/head_extrinsics.json
```

每帧先用 IPPE PnP 求 `T_camera_board`，再从最终 URDF 按该帧同步的 yaw、pitch 求 `T_base_pitch`。固定棋盘满足：

```text
T_base_pitch(i) · T_pitch_camera · T_camera_board(i)
  = T_base_board
```

程序比较 OpenCV 的多种 hand-eye 初值，再对 `T_pitch_camera` 和固定的 `T_base_board` 做联合鲁棒优化。它要求 yaw、pitch 都有足够角度跨度，并用最后 6 帧检查未参与求解的棋盘位姿是否仍保持固定。

输出中的关键字段为：

- `T_pitch_camera`：实际标定结果，运行时应使用它；
- `nominal_T_pitch_camera`：最终 URDF 中的名义安装值，仅供比较；
- `calibrated_from_nominal`：实测与名义安装的差异；
- `training`、`holdout`：每帧 PnP、平移和旋转残差；
- `model_consistency`：`configs/assembly.urdf` 与 `configs/scene.xml` 的链一致性检查。

运行时相机在基座中的动态位姿为：

```text
T_base_camera(q_yaw, q_pitch)
  = T_base_pitch(q_yaw, q_pitch) · T_pitch_camera
```

所以完成本标定后，执行任务时的 yaw、pitch 不必与标定中的某一个姿态相同；必须使用任务当时的实时关节角做上述动态组合。

## 四、标定机械臂触点 TCP

最终验证需要一个相对腕部刚性不动、能重复接触内角点的尖端。带灵巧手时建议安装刚性探针；也可以使用固定姿态的指尖，但整个 TCP 标定和验证期间所有手指关节必须保持完全相同。当前状态文件只记录 `arm_q14` 和头部关节，不记录手指关节，因此手指一旦移动，验证立即失效。

用同一个右臂尖端抵住同一个固定点，改变至少四种明显不同的腕部朝向。每次稳定后，用 `state` 子命令只读取控制器反馈并保存 `data/tcp/pose-01.json` 至 `pose-04.json`；该命令不发送任何运动指令：

```bash
python calibration.py \
  --config configs/head_config.json state \
  --output data/tcp/pose-01.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/tcp/pose-02.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/tcp/pose-03.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/tcp/pose-04.json
```

每份 JSON 包含 `arm_q14`（按配置左臂再右臂顺序排列的十四个有限实测关节角，单位 rad）、同步 `head_q2` 和 `timestamp_s`。下方 TCP 拟合与验证均使用同一右臂探针。

离线拟合腕部坐标中的尖端位置：

```bash
python calibration.py \
  --config configs/head_config.json pivot --side right \
  --states data/tcp/pose-{01,02,03,04}.json \
  --output data/head_camera_session/tcp/head_tcp_pivot.json
```

`passed` 必须为 `true`。该步骤只标定触点位置，不标定工具朝向。

## 五、选择三个角点并触碰验证

外参求解结束后，下面是拍一张不参与求解的新图像：

```bash
python calibration.py \
  --config configs/head_config.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session data/touch-validation --count 1
```

选择三个分散且不共线的内角点。省略 `--corner` 时可在窗口中点击，程序会吸附到最近的已检测角点；下面示例直接按行列指定：

```bash
python calibration.py \
  --config configs/head_config.json select-validation \
  --pattern 7x10 --square-m 0.021 \
  --frame data/touch-validation/view-001 \
  --intrinsics data/head_camera_session/head_intrinsics.json \
  --extrinsics data/head_camera_session/head_extrinsics.json \
  --corner 0,0 --corner 0,6 --corner 9,3 \
  --output data/head_camera_validation/head_selection.json
```

先打开 `head_selection.png`，确认编号 1、2、3 与将要触碰的实体角点完全一致。保持棋盘不动，通过机器人已有的受审核控制界面，让同一个已标定尖端依次接触 1、2、3，并按同一顺序用 `state` 子命令保存实测关节角至 `data/touch-validation/state-01.json` 至 `state-03.json`。预测使用拍照时同步保存的头姿；拍照后的头部运动仅作为诊断记录。

```bash
python calibration.py \
  --config configs/head_config.json state \
  --output data/touch-validation/state-01.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/touch-validation/state-02.json
python calibration.py \
  --config configs/head_config.json state \
  --output data/touch-validation/state-03.json
```

程序不会发送任何运动命令。现场必须有人监护，使用低速和可重复定位的尖端，避免碰撞或推动棋盘。

执行比较：

```bash
python calibration.py \
  --config configs/head_config.json validate \
  --selection data/head_camera_validation/head_selection.json \
  --side right --tcp data/head_camera_session/tcp/head_tcp_pivot.json \
  --states data/touch-validation/state-{01,02,03}.json \
  --output data/head_camera_validation/head_validation.json
```

相机链预测值为：

```text
p_base_camera
  = T_base_pitch(head_q2)
  · T_pitch_camera
  · T_camera_board
  · p_board_corner
```

机械臂接触实测值为：

```text
p_base_touch
  = T_base_wrist(arm_q14) · p_wrist_tip
```

最终比较的是两者在 `base_Link` 下的三维欧氏距离，不比较关节角。同一个空间点可能对应多组关节角，因此用关节角作为误差指标不成立。`head_validation.json` 给出三点各自误差、平均误差和最大误差；默认最大允许误差为 10 mm，必须根据 TCP 重复性、棋盘固定误差和实际任务间隙预算重新确定。

## 换一个头姿复核之前的触点

接受右臂 TCP 固定点标定结果、完成第一次棋盘触点选择并保存三份触碰状态后，保持棋盘在基座中的位置和朝向不变，尖端也保持同一刚性安装；头部相机可以移动。重新拍一张带同步头姿的彩色图，按原触碰顺序复用上次选中的三个实体角点，再用旧的机械臂状态计算触点基座坐标并比较距离。这是离线检查，不驱动机器人，也不需要重新触碰。

```bash
python calibration.py \
  --config configs/head_config.json capture \
  --pattern 7x10 --square-m 0.021 \
  --session data/touch-recheck --count 1

python calibration.py \
  --config configs/head_config.json select-validation \
  --pattern 7x10 --square-m 0.021 \
  --frame data/touch-recheck/view-001 \
  --intrinsics data/head_camera_session/head_intrinsics.json \
  --extrinsics data/head_camera_session/head_extrinsics.json \
  --reuse-selection data/head_camera_validation/head_selection.json \
  --output data/head_camera_validation/head_recheck_selection.json

python calibration.py \
  --config configs/head_config.json validate \
  --selection data/head_camera_validation/head_recheck_selection.json \
  --side right --tcp data/head_camera_session/tcp/head_tcp_pivot.json \
  --states data/touch-validation/state-{01,02,03}.json \
  --output data/head_camera_validation/head_recheck_validation.json
```

检查 `head_recheck_selection.png`，确认标号对应上次实际触碰的三个实体角点。新图只求一次棋盘 PnP 位姿，旧状态给出三个基座系触点坐标。如果棋盘在第一次触碰后移动过，或尖端安装改变，比较就无效。报告记录各点三维距离（米）；超过配置门限时程序以退出码 1 结束。首次触点流程见本指南。

## 离线测试

```bash
python -m pytest -q test_calibration.py
```

测试覆盖 7×10 SB 角点检测、hand-eye 数学方向、最终 URDF 的头部 FK、yaw 引起 pitch 原点移动而 pitch 不移动自身原点，以及 `configs/assembly.urdf`/`configs/scene.xml` 的头部相机链一致性。测试不连接相机或机器人。

## 常见失败

- `expected 70 inner corners`：核对内角点数量，避免模糊、反光、遮挡和拍摄显示器产生的摩尔纹。
- `(0,0)` 在不同视角跳到棋盘对角：采集时按 `f` 修正 180° 顺序后再保存。
- `joint/image header skew exceeded`：该帧不是严格同步样本，重新等待头部稳定后采集。
- `insufficient head excitation`：yaw 或 pitch 覆盖不足，补采两个方向的组合姿态。
- URDF/XML 一致性失败：最终模型的关节轴、偏移或相机链已变化，先修复模型，不能用旧外参补偿。
- 触点误差大但重投影误差小：检查 TCP、手指姿态、棋盘是否被碰动、触点顺序及图像/关节状态同步。
