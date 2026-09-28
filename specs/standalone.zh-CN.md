# 独立相机标定仓库拆分

从 `AcSg999/tron2_deployment_on_dc` 的 `dbd6c2f2d0718f38c86fac34dc86788ad43c60b5` 提取 `sp_vision`，新仓库名为 `tron2_camera_calibration`。

- 保留现有离线标定算法、测试、配置结构及 URDF/XML 运动学快照。
- 提供可安装的 `sp_vision` Python 包、依赖声明、CLI、中英文文档及 CI。
- 移除对 `tron2_deployment` 的运行时导入，迁移必要的相机采集及只读控制器反馈适配器。明确说明可选 ROS 和厂商传输环境依赖。
- 导入及构造不连接硬件，不增加运动命令接口。
- 主机、ROS setup 路径和 domain 可配置；保留采集同步及质量门限。
- 保留 MIT 许可证及来源信息；排除本机配置、采集数据、生成结果、日志和凭据。
- 验证原有离线测试及适配器测试；在源码目录之外验证 wheel 的配置、模型资产及 CLI help。不得宣称完成硬件验证。
- 在贡献者文档中署名 `AcSg999` 和 `Shukashuki`，发布后邀请两位为仓库协作者，保留原版权信息。
- 授权账号可用后，将验证后的结果发布为 Public 的 `zengxiong111/tron2_camera_calibration`。

拆分以固定源版本导入提交开始新历史，不保留父仓库全部历史提交。原仓库不修改。
