# 硬件与网络

## 推荐分工

- Jetson Orin Nano：运行 livox_ros_driver2 与 AeroHalo360。
- Livox MID-360：通过独立以太网连接 Jetson。
- CUAV X7+：运行 ArduPilot。
- USB：只用于拆桨台架。
- 空闲 TELEM UART：正式飞行候选链路。

## MID-360 网络

AeroHalo360 不配置雷达网络。所有设备网络参数位于 livox_ros_driver2 的 MID360_config.json。

MID-360 出厂地址候选规则：

~~~text
192.168.1.1XX
~~~

XX 为 SN 末两位。若 Livox Viewer 2 修改过静态 IP，必须以当前实际地址为准。

JSON 字段含义：

- host_net_info.cmd_data_ip：Jetson 雷达网口地址。
- host_net_info.point_data_ip：点云目标 Jetson 地址。
- host_net_info.imu_data_ip：IMU 目标 Jetson 地址。
- lidar_configs[].ip：LiDAR 自身地址。

主机与 LiDAR 必须同子网、不同地址。任何示例 IP 都不能直接当生产默认值。

源码与安装副本：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
ros2 pkg prefix livox_ros_driver2
find "$HOME/ws_livox/src/livox_ros_driver2/config" -type f -iname '*MID360*.json' -print
find "$HOME/ws_livox/install" -type f -iname '*MID360*.json' -print
~~~

修改源码配置后重新构建：

~~~bash
source /opt/ros/humble/setup.bash
cd "$HOME/ws_livox"
colcon build --packages-select livox_ros_driver2
source "$HOME/ws_livox/install/setup.bash"
~~~

## PointCloud2 模式

使用官方驱动中发布 PointCloud2 的 rviz_MID360_launch.py。msg_MID360_launch.py 发布 Livox 自定义消息，不能直接输入 AeroHalo360。

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
ros2 topic type /livox/lidar
ros2 topic info /livox/lidar
ros2 topic hz /livox/lidar
~~~

## 飞控 USB 台架

稳定路径必须从本机发现：

~~~bash
find /dev/serial/by-id -maxdepth 1 -type l -print
groups
fuser -v /dev/serial/by-id/REPLACE_WITH_STABLE_DEVICE
~~~

不得让 Mission Planner 与 sender 同时直接打开同一串口。

## TELEM UART 飞行候选

- 根据 CUAV X7+ 文档核对物理 TELEM 与逻辑 SerialN。
- TX/RX 交叉连接并共地。
- 核对 3.3 V 电平、RTS/CTS 和 Jetson pinmux。
- 不从 TELEM 5 V 给 Jetson 供电。
- 使用稳定 udev 别名或明确的稳定设备路径。
- 在实机比较 115200、460800、921600 的错误率和丢包。

ArduPilot 官方串口说明：https://ardupilot.org/copter/docs/common-serial-options.html


### UART 稳定别名与波特率验收

Jetson 板载 UART 不一定出现在 `/dev/serial/by-id`。先按目标载板 pinmux 和设备树确认实际 tty，再创建仅匹配该硬件路径的 udev 规则；不要照抄 `/dev/ttyTHS1`。规则示意：

~~~text
SUBSYSTEM=="tty", KERNELS=="REPLACE_WITH_VERIFIED_DEVICE_PATH", SYMLINK+="aero-halo-fcu", GROUP="dialout", MODE="0660"
~~~

重新加载规则后冷启动验证十次，并确认 `/dev/aero-halo-fcu` 始终指向同一 UART。USB 台架优先使用系统生成的 `/dev/serial/by-id/...`，不需要自建模糊规则。

在真实飞控上分别以 115200、460800、921600 运行至少 30 分钟，记录 heartbeat 丢失、sender 发送错误、ArduPilot MAVLink 丢包统计和重连次数。选择错误率稳定且 CPU 占用可接受的最低充分波特率，并将结果写入本机部署记录。
