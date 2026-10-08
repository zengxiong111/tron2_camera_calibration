# TRON2 相机标定

固定桌面高度：[坐标系约束](TABLETOP_FRAME_CONTRACT.zh-CN.md) · [English](TABLETOP_FRAME_CONTRACT.md)


[English](README.md)

本项目提供独立的 Python 工具，用于离线标定 TRON2 可动头部彩色相机和右腕彩色相机。软件包包含求解器所需的代码和运动学模型快照；不包含机器人控制器、ROS 安装、相机录像或本机机器人配置。

## 已发布的标定结果

本地最新头部（2026-09-23）和右腕（2026-10-05）内外参 JSON 已发布到 [calibration_results](calibration_results/README.zh-CN.md)，附来源校验值及坐标系、单位说明。两组拟合均通过内部检查，但已记录的独立触点验证未达到 10 mm 阈值（头部 14.42 mm，右腕 12.26 mm）。复用前请阅读结果说明；发布不代表硬件验收通过。

## 安装

需要 Python 3.10 或更新版本。克隆仓库后在根目录跑一次安装脚本，**这一步生成 `.venv/bin/python`**，以及 `sp-vision-head`、`sp-vision-wrist`、`sp-vision-capture` 三个命令行入口；本 README 和标定指南中的所有命令都使用该解释器：

```bash
git clone https://github.com/zengxiong111/tron2_camera_calibration.git
cd tron2_camera_calibration
bash scripts/install.sh
```

`scripts/install.sh` 依次执行：`python3.10 -I -m venv .venv` 创建虚拟环境 → 用 `.venv/bin/python` 升级 pip → 安装本包及 `test`、`live` 辅助依赖（即 `'.[test,live]'`）→ `pip check` → 校验三个 `sp-vision-* --help` 入口。需要手工建环境时，等价命令是：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install '.[test,live]'
```

安装脚本的边界：

- 需要 `PATH` 上有 Python 3.10 或更新版本。Ubuntu 20.04 的系统 Python 3.8 不行，也不要改写 `/usr/bin/python3`；用 `TRON2_PYTHON=/path/to/python3.10 bash scripts/install.sh` 指定其他解释器。
- 该解释器必须带 `venv`/`ensurepip` 模块（Debian/Ubuntu 上通常是 `python3.10-venv` 软件包）。
- OpenCV 需要系统共享库：Ubuntu 上先执行 `sudo apt install -y libgl1 libglib2.0-0`，否则 `import cv2` 会因缺少 `libGL.so.1` 失败。
- 需要能访问软件源，默认 `https://pypi.org/simple`；用 `TRON2_PIP_INDEX_URL` 可换源。该选择只对脚本自身及其构建子进程生效，不改写全局 pip 配置。
- `.venv` 已存在时脚本**只校验解释器版本，不会重建**；版本低于 3.10 会直接报错退出（请保留原环境，另建一份检出）。要强制重建需先手动删除 `.venv`。
- 脚本**不安装 ROS，也不安装可选的 `tron2_env` 运行时**（见下）。

开发和测试时把可编辑安装装进同一个环境：

```bash
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -m pytest -q
```

下文命令中出现的 `python` 均指该虚拟环境的解释器：先执行一次 `source .venv/bin/activate` 后继续用 `python`，或在仓库根目录显式调用 `.venv/bin/python`（例如 `.venv/bin/python sp_vision/calibration.py ...`）。`.venv` 已被 Git 忽略，也不进入 wheel 包，每个检出创建一次即可。不要与提供 `opencv-python` 或 headless OpenCV wheel 的环境混用，因为本项目以 `opencv-contrib-python` 作为唯一的 `cv2` 提供者。

基础依赖也列在 `requirements.txt`，供使用 requirements 文件的环境安装。

离线标定只需要这个虚拟环境：有 NumPy、SciPy 和带 contrib 的 OpenCV，就足以求解头部与腕部内外参、TCP pivot 与触点验证。头部相机通过桥接器实时采集还需要 `websockets` 客户端（已随 `'.[live]'` 装好）。默认的头部相机 ROS 后端在本机使用 ROS 1 Noetic，需要系统安装 `rospy`、`message_filters`、`sensor_msgs`，并能连接 ROS master。独立的右腕采集流程和 `sp-vision-capture` 工具使用配置相机主机上的 ROS 2 Foxy，需要该主机安装 `rclpy` 和 `sensor_msgs`；这些 ROS 系统包不会由本 Python 包或安装脚本安装。可选的控制器状态命令（`state` 记录姿态、`probe` 做腕部映射对比）需要单独提供的 `tron2_env` 运行环境及其传输依赖，离线标定不依赖它们。

## 命令

安装后提供三个 CLI；传入 `--help` 可查看子命令和参数：

```bash
sp-vision-head --help
sp-vision-wrist --help
sp-vision-capture --help
```

源码目录中仍可直接运行原脚本：

```bash
python sp_vision/calibration.py --help
python sp_vision/calibration_wrist.py --help
python sp_vision/capture_ros2_image.py --help
```

打包的示例配置和模型快照位于 `sp_vision/configs/`。使用前应把对应的 `*.example.json` 复制为本地 JSON，再填入本机相机主机、话题或配置路径。Git 和 wheel 包均不包含本机配置及标定会话；请将录像、拟合结果和报告放到可写工作目录，例如 `data/`。

## 文档导航

| 文档 | 用途 |
|---|---|
| [头部相机标定](sp_vision/head_calib.zh-CN.md) | 配置、采集、内外参求解及触点验证 |
| [右腕相机标定](sp_vision/wrist_calib.zh-CN.md) | 关节映射核对、腕部采集及相机到腕部外参验证 |
| [贡献者](CONTRIBUTORS.zh-CN.md) | 项目贡献者与版权归属 |
| [更新记录](CHANGELOG.zh-CN.md) | 版本历史和兼容说明 |
| [来源清单](SOURCE_MANIFEST.json) | 源提交、原始及修改后文件哈希、模型资产范围 |

标定指南的脚本示例从 `sp_vision/` 目录运行。触点验证读取 `state` 子命令生成的 JSON，该命令只采样控制器反馈，不发送运动指令。仓库不分发标定数据集或历史拟合结果。

## 工作流与证据范围

- **离线标定**读取保存的图像、关节状态记录、JSON 设置和随包运动学模型。它可拟合相机内参、外参、固定点 TCP，并执行几何一致性检查；不会打开相机、连接 ROS 或控制机器人。
- **实时采集**与求解相互独立。头部工作流默认通过本机 ROS 1 Noetic 读取 RGB-D 数据，也可使用现有 WebSocket RGB-D 桥接后端；独立的右腕工作流通过相机主机上的 ROS 2 读取右侧彩色图像和同步关节状态。`sp-vision-capture` 是 ROS 2 单帧诊断工具。使用时须区分 ROS 版本和数据流。这些接口都不会发送运动指令。
- **模型快照仅用于运动学。**随包 URDF/MJCF 快照引用了未随仓库分发的网格文件。求解器只使用关节变换，不加载视觉或碰撞几何。因此这些快照仅支持运动学正向计算和模型链一致性检查，不能用于网格渲染、碰撞检查、物理仿真或机器人几何避障结论。
- **求解通过不等于实机验证。**离线拟合和留出数据检查不能证明实时相机同步正确、物理安装正确、机器人触碰验证成功或 Sim2Real 表现。请按头部和腕部指南完成独立触碰检查，并按报告所定义的范围解读 `passed` 字段。

数据采集与验证步骤见[头部相机标定](sp_vision/head_calib.zh-CN.md)和[右腕相机标定](sp_vision/wrist_calib.zh-CN.md)。

## 许可证

贡献者：[AcSg999](https://github.com/AcSg999)、[Shukashuki](https://github.com/Shukashuki)。来源署名见[贡献者说明](CONTRIBUTORS.zh-CN.md)。

MIT。原始版权声明保留在 [LICENSE](LICENSE) 中。
