# TRON2 相机标定

[English](README.md)

本项目提供独立的 Python 工具，用于离线标定 TRON2 可动头部彩色相机和右腕彩色相机。软件包包含求解器所需的代码和运动学模型快照；不包含机器人控制器、ROS 安装、相机录像或本机机器人配置。

## 安装

需要 Python 3.10 或更新版本。从源码目录安装：

```bash
python -m pip install .
```

开发和测试环境：

```bash
python -m pip install -e '.[test]'
python -m pytest -q
```

基础依赖也列在 `requirements.txt`，供使用 requirements 文件的环境安装。一般建议使用 `pip install -e .` 安装项目。

离线求解需要 NumPy、SciPy 和包含 contrib 模块的 OpenCV。头部相机通过桥接器实时采集还需要执行 `pip install '.[live]'`（即 `websockets` 客户端）。默认的头部相机 ROS 后端在本机使用 ROS 1 Noetic，需要系统安装 `rospy`、`message_filters`、`sensor_msgs`，并能连接 ROS master。独立的右腕采集流程和 `sp-vision-capture` 工具使用配置相机主机上的 ROS 2 Foxy，需要该主机安装 `rclpy` 和 `sensor_msgs`；这些 ROS 系统包不会由本 Python 包安装。可选的控制器状态探测需要单独提供的 `tron2_env` 运行环境及其传输依赖，离线标定不依赖它。

## 命令

安装后提供三个无需参数的命令入口；传入 `--help` 可查看各自帮助：

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

## 工作流与证据范围

- **离线标定**读取保存的图像、关节状态记录、JSON 设置和随包运动学模型。它可拟合相机内参、外参、固定点 TCP，并执行几何一致性检查；不会打开相机、连接 ROS 或控制机器人。
- **实时采集**与求解相互独立。头部工作流默认通过本机 ROS 1 Noetic 读取 RGB-D 数据，也可使用现有 WebSocket RGB-D 桥接后端；独立的右腕工作流通过相机主机上的 ROS 2 读取右侧彩色图像和同步关节状态。`sp-vision-capture` 是 ROS 2 单帧诊断工具。使用时须区分 ROS 版本和数据流。这些接口都不会发送运动指令。
- **模型快照仅用于运动学。**随包 URDF/MJCF 快照引用了未随仓库分发的网格文件。求解器只使用关节变换，不加载视觉或碰撞几何。因此这些快照仅支持运动学正向计算和模型链一致性检查，不能用于网格渲染、碰撞检查、物理仿真或机器人几何避障结论。
- **求解通过不等于实机验证。**离线拟合和留出数据检查不能证明实时相机同步正确、物理安装正确、机器人触碰验证成功或 Sim2Real 表现。请按头部和腕部指南完成独立触碰检查，并按报告所定义的范围解读 `passed` 字段。

数据采集与验证步骤见[头部相机标定](sp_vision/head_calib.zh-CN.md)和[右腕相机标定](sp_vision/wrist_calib.zh-CN.md)。

## 许可证

贡献者：[AcSg999](https://github.com/AcSg999)、[Shukashuki](https://github.com/Shukashuki)。来源署名见[贡献者说明](CONTRIBUTORS.zh-CN.md)。


MIT。原始版权声明保留在 [LICENSE](LICENSE) 中。
