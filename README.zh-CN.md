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
2. 打印并实测棋盘格：默认 **7×10 内角点、方格边长 0.021 m**，即 8×11 个方格。
3. 固定棋盘，采集约 40 个不同姿态；依次求解内参、外参。
4. 标定刚性触针 TCP，再用三个角点做独立触碰验证。

详细命令见[头部指南](sp_vision/head_calib.zh-CN.md)和[右腕指南](sp_vision/wrist_calib.zh-CN.md)。示例命令从 `sp_vision/` 运行；配置内路径相对 JSON 文件，数据路径相对当前目录。用 `sp-vision-head --help`、`sp-vision-wrist --help`、`sp-vision-capture --help` 查看参数。

| 功能 | 额外运行条件 |
|---|---|
| 离线内外参、TCP 与验证 | 安装后的 Python 环境 |
| 头部采集 | 本机 ROS 1 Noetic 或 WebSocket 桥接；只需彩色图像和同步头部关节状态，不需要深度图或已有内参 |
| 右腕采集 | 相机主机上的 ROS 2 Foxy、图像和关节状态话题，以及 SSH 公钥登录；可用 `capture.ssh_identity_file` 指定私钥 |
| `state` / 腕部 `probe` 控制器对比 | 另行安装兼容的 `tron2_env`，填写 `robot.host` / `robot.port` |
| 单帧诊断 | ROS 2 主机及 SSH；图像不含同步关节状态，不能直接用于标定 |

## 准备资料

- [棋盘格生成器](https://calib.io/zh/pages/camera-calibration-pattern-generator)：生成打印图案；打印后实测方格边长并更新配置。
- [TRON2 SDK 开发指南](https://www.limxdynamics.com/zh/documents/847884267345285120)：开发连接说明及 SSH 登录密码。腕部脚本使用免密登录，请先按指南登录并授权公钥。
- [TRON2 用户手册](https://www.limxdynamics.com/zh/documents/844648486841487360)：联网步骤及 Wi-Fi 密码。
- [TRON2 官方机器人模型](https://github.com/limxdynamics/tron2-robot-description)：双臂＋头部等配置的 URDF/xacro 与模型资源；选择与实机一致的型号。仓库自带模型是求解器使用的运动学快照，不随官方仓库自动更新。

## 结果与坐标约定

已发布[头部与右腕内外参](calibration_results/README.zh-CN.md)。它们通过内部拟合检查，但独立触点误差为头部 **14.42 mm**、右腕 **12.26 mm**，均未达到 10 mm 阈值。复用前应按目标模型重新求解并验证。

`T_A_B` 表示从 B 到 A 的变换，平移单位为米、关节角为弧度。桌面任务遵守[固定 45 cm 安装约束](TABLETOP_FRAME_CONTRACT.zh-CN.md)。模型快照不包含网格，只用于运动学求解。内外参 `passed: true` 不代表独立触碰验收通过。

本地配置、图像和新生成的报告放在忽略目录中（如 `data/`），不提交密码、私钥或采集数据。

## 开发与许可证

```bash
python -m pip install -e '.[test]'
python -m pytest -q
```

[更新记录](CHANGELOG.zh-CN.md) · [贡献者与来源](CONTRIBUTORS.zh-CN.md) · [MIT 许可证](LICENSE)
