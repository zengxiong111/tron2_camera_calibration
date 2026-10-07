# 更新日志

本文件记录版本级变更。[English](CHANGELOG.md)

## [Unreleased]

### 新增
- 新增 `scripts/install.sh`（由部署仓库的安装脚本改写）：创建 `.venv/bin/python` 与三个 `sp-vision-*` 入口，安装 `'.[test,live]'`，执行 `pip check`，并用 `--help` 逐个校验入口。

### 修复
- 恢复 `sp-vision-head` 和 `sp-vision-wrist` 可运行的 `state` 子命令：它把实测控制器 `arm_q14`/`head_q2` 反馈保存为 JSON，供 TCP pivot 拟合和触点验证使用，替代原先指向不可用外部记录工具的文档说明。该命令只读取反馈，不发送任何运动指令。
- 在 `head_config.example.json` 及头/腕标定指南中补充 `capture.state_profile` 说明。

### 文档
- 把 `scripts/install.sh` 写成文档化的安装步骤：它生成 `.venv/bin/python`，README 与两个标定指南都调用该解释器，离线标定只需要这个环境。README 同时记录脚本的边界（Python 3.10+ 且带 `venv`/`ensurepip`、OpenCV 共享库、软件源可达、已存在 `.venv` 的处理方式，以及不安装 ROS 与 `tron2_env`）。
- 在两个标定指南中逐个说明 `head_intrinsics.json`、`wrist_intrinsics.json`、`head_extrinsics.json`、`wrist_extrinsics.json` 的结果字段，并新增“如何获得更好的标定效果”一节，涵盖标定板选择、姿态覆盖、激励门限与使结果失效的情形。

### 注意事项
- `state` 命令与腕部 `probe` 对比一样需要外部 `tron2_env` 运行环境；离线求解仍不依赖它。

## [0.1.0] - 2026-09-28

### 功能
- 将 TRON2 头部和右腕相机离线标定工具打包为 `tron2-camera-calibration`。
- 在构建产物中包含示例 JSON 配置及 URDF/MJCF 运动学快照。
- 添加命令行入口，并说明实时采集所需的可选依赖。

### 设计理由
- 离线求解可在没有 ROS 或控制器软件的环境中安装。硬件相关适配器和实时采集保持可选并延迟加载。
- 仅打包求解器正向运动学需要的模型文件；离线求解不依赖外部网格资产。

### 注意事项
- 实时 ROS 采集需要单独安装 ROS 运行环境，并能连接配置的相机主机。厂商控制器状态探测还依赖外部 `tron2_env` 环境。
- 随包 URDF/MJCF 快照不含所引用的网格，不能据此声称完成渲染、碰撞或物理仿真验证。
- 离线求解通过不等同于独立实机触碰验证或 Sim2Real 成功。
