# 更新日志

本项目采用语义版本号；ROS 包版本为 0.2.1，发布标签为 v0.2.1-alpha。

## v0.2.1-alpha - 2026-07-12

### 安全与数据链路

- 增加源点云时间戳、序号和年龄的端到端校验。
- 修正过近障碍、过滤后空点云和近距离几何膨胀的 fail-closed 语义。
- cloud 与 sender 增加 WARN/FAIL 两级超时和连续健康帧恢复滞回。
- MAVLink 连接区分端口打开、心跳等待、飞控已连接、发送错误和断开。
- 强制 MAVLink 2、BODY_FRD(12)、目标 sysid/compid 过滤和有上限退避重连。
- PointCloud2 增加结构、字段、字节、点数、endian 和 NaN/Inf 比例校验。

### 部署与配置

- 实现 base、profile、installation 和显式 CLI 的配置分层。
- 分离 Mission Planner 拆桨演示与低速飞行候选 profile。
- 新增只读硬件预检、systemd 模板安装、独立 cloud/sender/Livox 服务与健康定时器。
- ROS 安装源固定为 ros-apt-source 1.1.0，并校验 SHA-256。
- 新增 X7 + MID360 未标定安装示例和四方向自动校验工具。

### 测试与开源发布

- 新增看门狗抖动/恢复、配置分层、PointCloud2 异常和 replay 错误路径测试。
- 新增合成点云集成、时延/资源基准工具和 HIL 故障矩阵模板。
- 新增 GitHub Actions、静态检查、可复现 tarball、manifest 和 SHA-256 sidecar。
- 使用完整 Apache License 2.0 文本，统一版本和中文部署文档。

### 尚未完成

TF、安装偏航、自遮挡、真实 UART 波特率、30 分钟 Livox 连续性、一小时 soak、P99 sensor-to-MAVLink、Loiter/AUTO/GUIDED 故障行为和真实飞行仍需 HIL 验证，全部阻塞飞行。
