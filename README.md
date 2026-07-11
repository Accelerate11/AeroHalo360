# AeroHalo360

AeroHalo360 是面向 ArduPilot 飞控的 ROS2 近距避障模块。它把 Livox MID360 等 3D 激光雷达发布的 `PointCloud2` 点云转换为 MAVLink `OBSTACLE_DISTANCE` 消息，将机体周围 360 度空间划分为 72 个 5 度扇区，供 ArduPilot 的 Proximity、Simple Avoidance 与 BendyRuler 使用。

当前版本定位为 `v0.2-alpha`：适合台架验证、室内低速联调和 Mission Planner 可视化检查，不建议直接用于高速或复杂外场飞行。

## 安全边界

AeroHalo360 不是飞控，也不是完整自主避障系统。它只提供“周围障碍物距离”的感知输入，最终避让动作仍由 ArduPilot 模式、参数和任务逻辑决定。

使用前请确认：

- 已保留人工遥控接管通道。
- 已在台架、空桨或安全约束条件下完成验证。
- 已确认雷达坐标系、机体坐标系和安装方向正确。
- 已确认 Loiter、AUTO、GUIDED 等目标模式已启用对应避障参数。
- 已按场地风险选择 `watchdog.degraded_mode`，默认建议使用 fail-closed 虚拟墙。

## 当前能力

- 订阅 Livox MID360 或兼容设备发布的 `sensor_msgs/msg/PointCloud2`。
- 支持距离、高度、机体自遮挡、体素降采样、半径离群点过滤。
- 将点云压缩为 72 个 MAVLink 兼容扇区，默认每扇区 5 度。
- 支持时间滤波、障碍膨胀和近中远不同点数阈值。
- Watchdog 可检测点云超时、TF 异常、原始空点云、处理频率过低等状态。
- MAVLink sender 默认使用 MAVLink2，发送 `OBSTACLE_DISTANCE` 和 heartbeat。
- 在启动未收到扇区数据或扇区数据超时时，默认发送虚拟墙，避免误报“全向安全”。
- 支持 launch 参数覆盖、可选静态 LiDAR TF、调试点云、RViz marker 和诊断开关。

## 工程位置

源码目录：

```bash
/home/accelerate/AeroHalo360_ws/src/aero-halo-360
```

ROS2 工作空间目录：

```bash
/home/accelerate/AeroHalo360_ws
```

`build/`、`install/`、`log/` 是本地构建产物，不需要提交到代码仓库。

## 编译

```bash
cd /home/accelerate/AeroHalo360_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 快速运行

启动点云处理节点：

```bash
ros2 launch aero_halo_360 aero_halo_360.launch.py
```

发布一个前方 2 米虚拟墙，用于台架检查：

```bash
ros2 run aero_halo_360 synthetic_cloud_generator.py --ros-args \
  -p wall_direction:=front \
  -p wall_distance_m:=2.0
```

查看扇区距离输出：

```bash
ros2 topic echo /aero_halo_360/sector_distances
```

默认坐标约定下，四个方向对应的关键扇区如下：

| 方向 | 扇区 | 说明 |
| --- | ---: | --- |
| 前方 | `0` | 机体 `+X` 方向 |
| 右方 | `18` | 机体 `-Y` 方向 |
| 后方 | `36` | 机体 `-X` 方向 |
| 左方 | `54` | 机体 `+Y` 方向 |

## 常用 launch 参数

```bash
ros2 launch aero_halo_360 aero_halo_360.launch.py \
  input_topic:=/livox/lidar \
  target_frame:=base_link \
  publish_filtered_cloud:=true \
  publish_markers:=true \
  publish_diagnostics:=true
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `config_file` | 包内 `config/default.yaml` | 主配置文件路径 |
| `input_topic` | `/livox/lidar` | 输入点云话题 |
| `target_frame` | `base_link` | 点云转换后的目标坐标系 |
| `start_mavlink` | `false` | 是否随 launch 一起启动 MAVLink sender |
| `mavlink_connection` | `udpout:127.0.0.1:14550` | MAVLink 连接字符串 |
| `mavlink_baud` | `921600` | 串口波特率 |
| `publish_filtered_cloud` | `true` | 是否发布过滤后的点云 |
| `publish_markers` | `true` | 是否发布 RViz marker |
| `publish_diagnostics` | `true` | 是否发布诊断消息 |
| `use_static_lidar_tf` | `false` | 是否发布静态 LiDAR TF |

## MAVLink 台架验证

通过 UDP 发送到本机 Mission Planner 或 MAVLink 检查工具：

```bash
ros2 run aero_halo_360 mavlink_obstacle_sender.py --ros-args \
  -p mavlink.connection:=udpout:127.0.0.1:14550 \
  -p mavlink.baud:=921600
```

通过串口连接飞控或数传设备：

```bash
ros2 run aero_halo_360 mavlink_obstacle_sender.py --ros-args \
  -p mavlink.connection:=/dev/ttyTHS1 \
  -p mavlink.baud:=921600
```

如果当前环境没有安装 MAVLink Python 库，先安装：

```bash
python3 -m pip install pymavlink
```

MAVLink sender 的默认安全行为：

- 启动后尚未收到 `/aero_halo_360/sector_distances` 时，发送 `80 cm` 虚拟墙。
- 扇区数据超过 `mavlink.input_timeout_ms` 未更新时，继续发送虚拟墙。
- 默认 `source_system=1`，`source_component` 优先使用 `MAV_COMP_ID_ONBOARD_COMPUTER`。
- 默认设置 `MAVLINK20=1`，确保发送 MAVLink2 消息。
- 诊断话题为 `/aero_halo_360/mavlink_diagnostics`。

## 配置文件

常用配置文件：

- `aero_halo_360/config/default.yaml`：默认台架配置。
- `aero_halo_360/config/flight_low_speed.yaml`：低速飞行联调配置，默认关闭部分调试发布。
- `aero_halo_360/config/indoor_low_speed.yaml`：室内低速配置。
- `aero_halo_360/config/outdoor_low_speed.yaml`：室外低速配置。
- `aero_halo_360/config/self_masks_x7_mid360.yaml`：CUAV X7+ 与 MID360 的自遮挡示例。

常改参数：

| 参数 | 说明 |
| --- | --- |
| `input_topic` | Livox 驱动发布的点云话题 |
| `target_frame` | 机体系，通常为 `base_link` |
| `radar.ip` / `radar.host_ip` | MID360 与机载计算机网口配置记录 |
| `vehicle.radius_m` / `vehicle.safety_extra_m` | 机体半径和安全余量 |
| `height_filter.z_min_m` / `height_filter.z_max_m` | 参与避障的高度范围 |
| `self_mask.*` | 机体、脚架、雷达支架等自遮挡区域 |
| `watchdog.*` | 超时、空点云、降级策略和虚拟墙距离 |
| `mavlink.connection` / `mavlink.baud` | MAVLink UDP 或串口连接 |

## 静态 LiDAR TF

如果 Livox 驱动发布的点云 frame 不是 `base_link`，并且系统内没有其它 TF 发布器，可以在 launch 中临时发布静态 TF：

```bash
ros2 launch aero_halo_360 aero_halo_360.launch.py \
  use_static_lidar_tf:=true \
  lidar_parent_frame:=base_link \
  lidar_child_frame:=livox_frame \
  lidar_xyz:="0.10 0.00 0.05" \
  lidar_rpy:="0.00 0.00 0.00"
```

`lidar_xyz` 单位为米，`lidar_rpy` 单位为弧度。正式上机前请用 RViz 或 `tf2_echo` 核对方向，尤其是前后、左右和偏航角。

## 测试

运行单元测试：

```bash
cd /home/accelerate/AeroHalo360_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select aero_halo_360
colcon test-result --verbose
```
