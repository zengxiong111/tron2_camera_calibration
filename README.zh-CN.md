# TRON2 相机标定

[English](README.md)

用于 TRON2 头部和右腕彩色相机的采集、内参标定、手眼外参求解与独立触点验证。离线求解不需要 ROS；采集和状态读取只读传感器，不发送运动指令。

## 安装

需要 Python 3.10+（含 `venv`）。Ubuntu 如缺少 OpenCV 共享库，先安装 `libgl1 libglib2.0-0`。

```bash
git clone https://github.com/zengxiong111/tron2_camera_calibration.git
cd tron2_camera_calibration
TRON2_PYTHON=python3 bash scripts/install.sh
source .venv/bin/activate
```

脚本创建 `.venv`，安装依赖并检查三个命令入口。已有环境会检查 Python 版本，不会重建。可用 `TRON2_PYTHON` 指定解释器、`TRON2_PIP_INDEX_URL` 指定软件源；脚本不安装 ROS 或 `tron2_env`。

## 使用流程

1. 将 `sp_vision/configs/` 中所需的 `*.example.json` 复制为对应的本地 JSON，填写连接配置。
2. 自行前往[棋盘格生成器](https://calib.io/zh/pages/camera-calibration-pattern-generator)生成、下载并打印棋盘格；清点内角点数量并**实测打印后的方格边长**，再填写配置。示例为 7×10 内角点，即 8×11 个方格。
3. 固定棋盘，采集约 40 个不同姿态；依次求解内参、外参。
4. 标定刚性触针 TCP，再用三个角点做独立触碰验证。

命令从 `sp_vision/` 运行；配置内路径相对 JSON 文件，数据路径相对当前目录。用 `sp-vision-head --help`、`sp-vision-wrist --help`、`sp-vision-capture --help` 查看参数。

### 实测棋盘格尺寸

使用最终打印并固定好的实体棋盘，用卡尺或直尺沿横向和纵向分别测量多个连续方格的总长度，再除以方格数；在不同位置重复测量，检查打印缩放和横纵尺寸是否一致。测的是**单个方格的边长**，不是整张纸、白边或内角点间隔总长。

将实测毫米数除以 1000，填入本地 `head_config.json` / `wrist_config.json` 的 `board.square_m`；`board.columns` 和 `board.rows` 填内角点数。头部示例的 `0.021` 和腕部示例的 `0.015` 都只是示例，不能当作实测值。若横纵缩放不同或棋盘不平整，应重新制作后测量。采集、内参、外参和验证必须使用同一组实测尺寸；修改尺寸后需重新求解和验证。

### 采集与求解

```bash
cd sp_vision
cp configs/head_config.example.json configs/head_config.json
cp configs/wrist_config.example.json configs/wrist_config.json
cp configs/robot_profile.example.json configs/robot_profile.json
```

先编辑本地配置：填入实测棋盘尺寸、连接地址和话题；将 `capture.profile`（头部）及 `capture.state_profile` 指向 `robot_profile.json`。头部选择 ROS 或桥接后端；腕部先配置 SSH 公钥登录，必要时设置 `capture.ssh_identity_file`。

```bash
python calibration.py --config configs/head_config.json capture --session data/head --count 40
python calibration.py --config configs/head_config.json intrinsics --session data/head --output data/head/head_intrinsics.json
python calibration.py --config configs/head_config.json extrinsics --session data/head --intrinsics data/head/head_intrinsics.json --output data/head/head_extrinsics.json

python calibration_wrist.py --config configs/wrist_config.json capture --session data/wrist --count 40
python calibration_wrist.py --config configs/wrist_config.json intrinsics --session data/wrist --output data/wrist/wrist_intrinsics.json
python calibration_wrist.py --config configs/wrist_config.json extrinsics --session data/wrist --intrinsics data/wrist/wrist_intrinsics.json --output data/wrist/wrist_extrinsics.json
```

采集时 `s` 保存、`f` 翻转角点顺序、`r`/空格重拍、`q` 退出。固定棋盘，变化头部或腕部姿态，等待稳定后再保存。完整采集会替换旧视图，追加需加 `--append`。拟合结果要求 `passed: true`；再通过 `state` 记录触针姿态、`pivot` 求 TCP、`select-validation` 选三个非共线角点，按编号触碰并保存状态，最后运行 `validate`。各子命令的参数见 `--help`。

| 功能 | 额外运行条件 |
|---|---|
| 离线内外参、TCP 与验证 | 安装后的 Python 环境 |
| 头部采集 | 本机 ROS 1 Noetic 或 WebSocket 桥接；只需彩色图像和同步头部关节状态，不需要深度图或已有内参 |
| 右腕采集 | 相机主机上的 ROS 2 Foxy、图像和关节状态话题，以及 SSH 公钥登录；可用 `capture.ssh_identity_file` 指定私钥 |
| `state` / 腕部 `probe` 控制器对比 | 另行安装兼容的 `tron2_env`，填写 `robot.host` / `robot.port` |
| 单帧诊断 | ROS 2 主机及 SSH；图像不含同步关节状态，不能直接用于标定 |

## 准备资料

- [棋盘格生成器](https://calib.io/zh/pages/camera-calibration-pattern-generator)：棋盘格需自行生成、下载并打印；打印后实测方格边长并更新配置。
- [TRON2 SDK 开发指南](https://www.limxdynamics.com/zh/documents/847884267345285120)：开发连接说明及 SSH 登录密码。腕部脚本使用免密登录，请先按指南登录并授权公钥。
- [TRON2 用户手册](https://www.limxdynamics.com/zh/documents/844648486841487360)：联网步骤及 Wi-Fi 密码。
- [TRON2 官方机器人模型](https://github.com/limxdynamics/tron2-robot-description)：双臂＋头部等配置的 URDF/xacro 与模型资源；选择与实机一致的型号。仓库自带模型是求解器使用的运动学快照，不随官方仓库自动更新。

## 结果与坐标约定

已发布[头部与右腕内外参](calibration_results/README.zh-CN.md)。它们通过内部拟合检查，但独立触点误差为头部 **14.42 mm**、右腕 **12.26 mm**，均未达到 10 mm 阈值。复用前应按目标模型重新求解并验证。

`T_A_B` 表示从 B 到 A 的变换，平移单位为米、关节角为弧度。桌面任务中，物理基座原点固定高于桌面顶部 **0.45 m**，即 `z_physical_base_world - z_tabletop_world = 0.45 m`；不能用 URDF root 或腕部原点代替物理基座原点。模型快照不包含网格，只用于运动学求解。内外参 `passed: true` 不代表独立触碰验收通过。

本地配置、图像和新生成的报告放在忽略目录中（如 `data/`），不提交密码、私钥或采集数据。

## 开发与许可证

```bash
python -m pip install -e '.[test]'
python -m pytest -q
```

[MIT 许可证](LICENSE)，保留 Jerry Lo 的原始版权声明。感谢 [AcSg999](https://github.com/AcSg999)、[Shukashuki](https://github.com/Shukashuki)、[peasgo](https://github.com/peasgo) 和 [Wangbaiyue007](https://github.com/Wangbaiyue007)。

来源：标定工具与运动学快照引入自 [tron2_deployment_on_dc](https://github.com/AcSg999/tron2_deployment_on_dc) 的 `sp_vision/`（`dbd6c2f2d0718f38c86fac34dc86788ad43c60b5`）；桥接适配器源于 [dexpipe](https://github.com/Shukashuki/dexpipe) 的 `hardware/tron2_rgbd.py`（`1e4a810de477bda7f621db553ecd496bb0884f2f`）；只读控制器适配器参考 [tron2_env](https://github.com/limxdynamics/tron2_env)（`03e39d3f7d66dcb096b41f03612fdab2f8c10ea5`）。
