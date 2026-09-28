# 左/右腕相机标定

[English](README_wrist.md)

这是右手腕彩色相机的独立只读实验。从本目录运行命令，图像和结果保存于 `data/wrist_camera_session/`；该目录不会进入 Git。模型文件随本目录提供，离线标定不再依赖外部仓库。程序不驱动机器人。

控制器负载辨识值不参与这项几何标定。若用拖动示教调整手臂，应单独在控制器中设置并回读当前负载，不要将这些值写入相机配置。

## 坐标系与几何关系

`configs/assembly.urdf` 中，相机支架固定在 `wrist_roll_R_Link`。它与 `wrist_pitch_R_Link` 之间还有可动的 `wrist_roll_R_Joint`：roll 的局部 X 轴与 pitch 的局部 Y 轴正交。因此应求解的常量是 **`T_wrist_roll_camera`**，即从 `right_wrist_camera_color_optical_frame` 到 `wrist_roll_R_Link` 的变换。URDF 中的数值只是名义安装参考，不是实测标定结果。对任一实测手臂状态：

```text
T_base_camera(q) = T_base_wrist_roll(q) · T_wrist_roll_camera
T_base_board = T_base_camera(q) · T_camera_board
T_wrist_pitch_camera(q_roll)
  = T_wrist_pitch_wrist_roll(q_roll) · T_wrist_roll_camera
```

`T_base_wrist_roll` 需要右臂七个实测关节角；头部与左臂关节角不在这条 FK 链上。采集仍按名称保存双臂全部 14 个角，供审计及后续触点验证使用。

相机内参不取决于可动关节数量，但必须针对当前 **640×480 的腕部缩放图像** 单独标定。当前 ROS 2 `CameraInfo` 报告的是原始流的 848×480，不能直接原样用于缩放话题。

## 准备配置并核对关节映射

```bash
cp configs/wrist_config.example.json configs/wrist_config.json
```

相机和 `/joint_states` 位于 `guest@10.192.1.4` 的 ROS 2 Foxy 环境。脚本通过 SSH 连接，仅接收时间戳相差不超过 100 ms 的新图像与关节状态。ROS 2 的前四个关节名是 `abad`、`hip`、`yaw`、`knee`，URDF 对应位置写作 `proximal_pitch`、`proximal_roll`、`proximal_yaw`、`elbow`。配置中的名称列表把它们排成**头部触点验证读取的控制器 `arm_q14` 的同一 14 维顺序**：名称不同，向量顺序并未改变。采集同时保存 ROS 2 原始名称和值、以及排好顺序的向量。FK 的实物精度仍需用留出集棋盘图像和独立触碰检查。

```bash
.venv/bin/python calibration_wrist.py --config configs/wrist_config.json probe
```

`probe` 不要求画面中有棋盘，会把同步图像和带名称的关节 JSON 保存到 `data/wrist_camera_session/`。它还会在拍照前后读取头部验证所用的同一控制器 `arm_q14`；只有手臂保持静止且 ROS 2 映射值逐项相差不超过 0.005 rad，`mapping_check.passed` 才为 `true`。控制器比较失败或不可用，不代表图像获取失败。图像与 ROS 2 关节状态的 `state_skew_ms` 是另一项检查，阈值为 100 ms。

把平整的 7×10 内角点棋盘刚性固定，实测方格边长为 0.021 m。在图像采集及触点验证期间保持其位置不变。只通过机器人已有的审核接口调整手臂，并在稳定后采样。采集不同的位置和方向，让棋盘覆盖画面不同区域，并使腕部至少绕两个不同方向转动。每张图应能看到完整棋盘；标定期间不要改变图像缩放设置。

## 采样、拟合与检查

```bash
.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  capture --session data/wrist_camera_session --count 40

.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  intrinsics --session data/wrist_camera_session

.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  extrinsics --session data/wrist_camera_session
```

采集窗口中，`s` 保存通过检查的图像，`f` 将棋盘角点顺序旋转 180°，`r` 或空格重新取图，`q` 退出。不加 `--append` 时，完整采集成功后覆盖该 session 的旧 `view-*`；中途退出会保留旧图。需要追加视角时加 `--append`。内参拟合复用头部流程的棋盘检测、畸变拟合、质量阈值及留出集划分。外参拟合复用 PnP 与手眼求解，但每张图都使用同步的 `T_base_wrist_roll(q)`。程序检查 URDF/MJCF 中的名义相机链，并用未参与拟合的图像检查训练所得棋盘位姿。要求结果 `passed: true`；同时检查 `training`、`holdout`、被拒绝视角及相对名义安装值的偏差。

## 标定右臂触点 TCP

若要重新标定 TCP，用同一支相对右腕刚性不动的尖端抵住同一个固定点，在至少四个明显不同的腕部朝向下读取实测状态。灵巧手指尖只有在所有手指关节始终保持同一姿态时才能作为尖端；状态文件不记录手指关节。每次由已有的受审核界面调整姿态，稳定后运行一次只读状态命令：

```bash
.venv/bin/tron2-deploy state --profile configs/robot_profile.example.json \
  --output data/wrist_camera_session/tcp/pose-01.json
.venv/bin/tron2-deploy state --profile configs/robot_profile.example.json \
  --output data/wrist_camera_session/tcp/pose-02.json
.venv/bin/tron2-deploy state --profile configs/robot_profile.example.json \
  --output data/wrist_camera_session/tcp/pose-03.json
.venv/bin/tron2-deploy state --profile configs/robot_profile.example.json \
  --output data/wrist_camera_session/tcp/pose-04.json
```

腕部程序直接复用头部流程的固定点拟合器，输出 `wrist_roll_R_Link` 中的尖端位置：

```bash
.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  pivot --states data/wrist_camera_session/tcp/pose-{01,02,03,04}.json \
  --output data/wrist_camera_session/tcp/wrist_tcp_pivot.json
```

要求结果 `passed: true`，并检查 `max_residual_m`、姿态跨度和 `condition_number`。四姿态拟合只确定尖端**位置**，不确定工具朝向。若实体尖端和安装从头部验证时起完全未变，也可跳过重采样，改用已复制的 `data/wrist_camera_session/tcp-pivot-from-head.json`；它的内部拟合通过，但先前头部相机的独立三点验证仍有 13–14 mm 误差，不能视为整条测量链通过。

如果已通过头部流程重新拟合并保存为 `data/head_camera_session/tcp/head_tcp_pivot.json`，无需再采集四姿态；将这份新结果复制到上述腕部 `wrist_tcp_pivot.json` 路径，或在下方验证命令中直接把它作为 `--tcp` 参数。不要误用更早的 `tcp-pivot-from-head.json`。

## 独立触碰验证

外参拟合后，保持棋盘在基座中固定，用腕部相机拍摄一张**不参与求解**的新图像。拍照后右臂可以移动；预测使用图像保存时同步的关节状态。选择三个分散、不共线且能用尖端接触的实体内角点：

```bash
.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  capture --session data/wrist_camera_validation --count 1

.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  select-validation --frame data/wrist_camera_validation/view-001 \
  --output data/wrist_camera_validation/wrist_selection.json
```

这条 `capture --count 1` 每次完整保存后都会用新的 `view-001` 覆盖旧验证图像；按 `q` 退出则保留旧图。重拍后必须重新运行 `select-validation`，使 `wrist_selection.json` 和 `selection.png` 对应新图；旧触点状态是否还能使用，取决于棋盘和实体角点是否保持不动。

先检查 `data/wrist_camera_validation/wrist_selection.png` 的编号是否对应将要触碰的实体角点。保持棋盘固定，用与 TCP 拟合时相同的尖端和手指姿态，按编号 1、2、3 触碰；每次稳定后读取一次状态：

```bash
.venv/bin/tron2-deploy state --profile configs/robot_profile.example.json \
  --output data/wrist_camera_validation/state-01.json
.venv/bin/tron2-deploy state --profile configs/robot_profile.example.json \
  --output data/wrist_camera_validation/state-02.json
.venv/bin/tron2-deploy state --profile configs/robot_profile.example.json \
  --output data/wrist_camera_validation/state-03.json

.venv/bin/python calibration_wrist.py --config configs/wrist_config.json \
  validate --selection data/wrist_camera_validation/wrist_selection.json \
  --side right --tcp data/wrist_camera_session/tcp/wrist_tcp_pivot.json \
  --states data/wrist_camera_validation/state-{01,02,03}.json \
  --output data/wrist_camera_validation/wrist_validation.json
```

如果复用先前 TCP，仅把上述 `--tcp` 路径改为 `data/wrist_camera_session/tcp-pivot-from-head.json`。验证在 `base_Link` 中比较相机预测的三个角点和尖端实测位置；报告的 `passed` 才是这次独立检查的结果。TCP 内部残差和外参留出集通过都不能替代它。

外参 JSON 的 `training`、`holdout` 中，`translation_m` 以米为单位；终端汇总中的 `max_training_mm`、`max_holdout_mm` 将其换算为毫米，与头部命令一致。若三点触碰误差接近棋盘角点间距，先核对 `wrist_selection.png` 上的 1、2、3 与实际触碰顺序。只有确认物理触点后，才用 `select-validation --corner 行,列` 按真实触碰顺序重新选点并验证；不要仅根据触碰状态反推角点后宣称独立验证通过。
