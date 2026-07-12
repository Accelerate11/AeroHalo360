# 版本路线图

## v0.1.0-alpha

- 建立 ROS 2 Humble 工程和 72 扇区点云压缩。
- 实现基础滤波、膨胀、Watchdog 与 MAVLink sender。

## v0.2.0-alpha

- 完成首次 Jetson、MID360、CUAV X7+ 与 Mission Planner 拆桨台架链路。
- 验证 PointCloud2 输入、扇区输出和 OBSTACLE_DISTANCE 可视化。

## v0.2.1-alpha

当前版本，目标是形成可公开审查和继续 HIL 的安全基线：

- 配置分层、原子参数校验和 demo/flight 隔离。
- 两级 watchdog、恢复滞回和源数据新鲜度校验。
- MAVLink 心跳身份、重连和 diagnostics。
- PointCloud2 资源上限、完整 Apache-2.0、CI 与可复现发布包。
- indoor_low_speed、outdoor_low_speed 和 flight_low_speed 均为未认证实验配置。

## v0.3.0

- 完成 base_link TF、安装方向、自遮挡和 UART 实机标定。
- 建立静态墙、快速接近/移开、窄柱、噪点与地面的 rosbag 回归数据集。
- 完成目标功耗模式的一小时 soak、P99 时延、温度和资源门限。

## v0.4.0

- 完成 Loiter 低速空旷 HIL 与全部故障矩阵。
- 固化按飞控固件版本区分的 ArduPilot 参数和人工接管流程。

## v0.5.0

- 在前序门禁全部通过后评估 AUTO、GUIDED、RTL 与 BendyRuler。
- 任何高速或复杂外场能力都需要独立风险评估和验证。
