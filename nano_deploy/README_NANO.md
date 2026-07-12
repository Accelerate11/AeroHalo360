# Jetson Orin Nano 部署指南

本指南适用于 aarch64、Ubuntu 22.04、ROS 2 Humble。旧款 Jetson Nano、JetPack 4/5 或 Ubuntu 20.04 不得直接运行 Humble 安装脚本。

## 1. 解压与校验

~~~bash
cd /media/"$USER"/YOUR_USB
sha256sum -c AeroHalo360_nano_v0.2.1-alpha_20260712.tar.gz.sha256
mkdir -p "$HOME/AeroHalo360_ws/src"
tar -xzf AeroHalo360_nano_v0.2.1-alpha_20260712.tar.gz \
  -C "$HOME/AeroHalo360_ws/src"
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
~~~

## 2. ROS 2 与依赖

ROS 2 Humble 尚未安装时：

~~~bash
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
bash nano_deploy/00_install_ros2_humble.sh
source /opt/ros/humble/setup.bash
~~~

默认依赖安装：

~~~bash
source /opt/ros/humble/setup.bash
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
bash nano_deploy/01_install_dependencies.sh
~~~

APT 可用但 rosdep GitHub 源不可达时：

~~~bash
source /opt/ros/humble/setup.bash
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
bash nano_deploy/01_install_dependencies.sh --skip-rosdep
~~~

脚本先安装显式 APT 依赖和用户级 pymavlink，再处理 rosdep。默认 best-effort 模式只有在必需 ROS 包与 pymavlink 实际验证通过时才成功；--strict-rosdep 要求 rosdep 成功。

## 3. 编译

在线严格模式：

~~~bash
source /opt/ros/humble/setup.bash
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
bash nano_deploy/02_build_release.sh --strict-rosdep
~~~

显式依赖已装好的离线模式：

~~~bash
source /opt/ros/humble/setup.bash
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
bash nano_deploy/02_build_release.sh --skip-rosdep
~~~

新 shell：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 pkg prefix aero_halo_360
~~~

## 4. Livox 外部依赖

AeroHalo360 不配置 MID-360 网络。必须先让 livox_ros_driver2 发布 PointCloud2。

MID-360 出厂地址候选规则为 192.168.1.1XX，XX 为 SN 末两位；设备被 Livox Viewer 2 改过后不再适用。

MID360_config.json 中 host_net_info 的 cmd_data_ip、point_data_ip、imu_data_ip 都是 Jetson 雷达网口地址；lidar_configs[].ip 才是 LiDAR 地址。

源码与安装副本检查：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
ros2 pkg prefix livox_ros_driver2
find "$HOME/ws_livox" -type f -iname '*MID360*.json' -print
~~~

使用发布 PointCloud2 的 rviz_MID360_launch.py，不使用发布 Livox 自定义消息的 msg_MID360_launch.py。

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
ros2 topic type /livox/lidar
ros2 topic info /livox/lidar
ros2 topic hz /livox/lidar
~~~

## 5. 只读预检

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/AeroHalo360_ws/install/setup.bash"
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
bash nano_deploy/04_preflight.sh \
  --interface YOUR_LIDAR_INTERFACE \
  --host-ip YOUR_HOST_IP \
  --lidar-ip YOUR_LIDAR_IP \
  --livox-workspace "$HOME/ws_livox" \
  --serial-device /dev/serial/by-id/REPLACE_WITH_STABLE_DEVICE
~~~

## 6. 拆桨 demo

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 launch aero_halo_360 mission_planner_demo.launch.py \
  start_mavlink:=false
~~~

若缺少 base_link TF，只能在拆桨可视化时显式使用：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 launch aero_halo_360 mission_planner_demo.launch.py \
  target_frame:=livox_frame \
  start_mavlink:=false
~~~

不得将此命令复制到 flight profile。

## 7. USB heartbeat 台架

~~~bash
find /dev/serial/by-id -maxdepth 1 -type l -print
sudo usermod -aG dialout "$USER"
~~~

重新登录后：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 run aero_halo_360 check_mavlink.py \
  --connection /dev/serial/by-id/REPLACE_WITH_STABLE_DEVICE \
  --heartbeat-only \
  --target-system 1 \
  --target-component 1 \
  --timeout 10
~~~

USB 仅用于拆桨台架。正式飞行候选链路为经过 pinmux、电平、SerialN 映射和误码验证的空闲 TELEM UART，不从 TELEM 5 V 给 Jetson 供电。

## 8. 启动 AeroHalo360

拆桨 Mission Planner 可视化：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 launch aero_halo_360 mission_planner_demo.launch.py \
  start_mavlink:=true \
  mavlink_connection:=/dev/serial/by-id/REPLACE_WITH_STABLE_DEVICE \
  wait_heartbeat_on_connect:=true
~~~

低速飞行候选，只有完成全部 HIL 门禁后才允许：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 launch aero_halo_360 aero_halo_360.launch.py \
  profile:=flight_low_speed \
  start_mavlink:=true \
  mavlink_connection:=/dev/serial/by-id/REPLACE_WITH_STABLE_DEVICE
~~~

## 9. 状态检查

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 topic hz /aero_halo_360/sector_distances
ros2 topic echo /aero_halo_360/diagnostics --once
ros2 topic echo /aero_halo_360/mavlink_diagnostics --once
~~~

MAVLink diagnostics 必须为 AUTOPILOT_CONNECTED 才代表目标飞控 heartbeat 新鲜。PORT_OPEN 只表示设备文件或 UDP socket 已打开。

## 10. systemd

仓库只提交模板，不提交真实用户名、串口 ID 或 IP。使用以下命令生成 /etc/aero-halo-360/env 与实际 units：

~~~bash
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
bash nano_deploy/06_install_systemd.sh \\
  --livox-workspace "$HOME/ws_livox" \\
  --mavlink-device /dev/serial/by-id/REPLACE_WITH_STABLE_DEVICE
~~~

确认环境文件后，再加 `--enable` 启用 Livox、cloud、sender 与健康定时器。

进程死亡检查：

~~~bash
sudo systemctl status aero-halo-360-cloud aero-halo-360-sender
sudo journalctl -u aero-halo-360-cloud -u aero-halo-360-sender -f
systemctl status aero-halo-360-health.timer
~~~

sender、串口、Jetson 或供电完全失效时无法发送虚拟墙，必须同时验证 ArduPilot 侧 timeout、模式降级和遥控接管。
