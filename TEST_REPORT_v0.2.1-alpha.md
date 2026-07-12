# AeroHalo360 v0.2.1-alpha 测试报告

测试日期：2026-07-12  
目标发布：v0.2.1-alpha  
源码环境：Ubuntu 22.04 WSL / ROS 2 Humble  
目标硬件：Jetson Orin Nano / aarch64

## 原始包与基线

- 原始附件：`AeroHalo360_nano_v0.2.0_20260711.tar.gz`
- 已核对 SHA-256：`0b50bf97b3be667c1678714203b23584b5b423309b7daf5fec902405d5d9c8b4`
- 修改前 Release 构建通过。
- 修改前报告口径不一致：CTest 入口、C++/Python 逻辑测试和 colcon XML 汇总不能混为一个数字。

## 最终自动测试

执行命令：

~~~bash
source /opt/ros/humble/setup.bash
cd "$HOME/AeroHalo360_ws"
colcon build --packages-select aero_halo_360 \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select aero_halo_360
colcon test-result --all --verbose
~~~

结果：

- CTest 入口：10，全部通过。
- C++ 逻辑测试：33，全部通过。
- Python 逻辑测试：27，全部通过。
- 逻辑测试总数：60，全部通过。
- `colcon test-result` XML 汇总：46 tests，0 errors，0 failures，0 skipped。

覆盖范围包括扇区方向、过近障碍、细杆全角度、时间滤波、膨胀、自遮挡、两级 watchdog、332 ms 抖动、恢复滞回、源时间戳、PointCloud2 字段/大小/endian/NaN/Inf、MAVLink 2/heartbeat/reconnect、配置分层和 replay 错误路径。

## 合成集成

Mission Planner demo profile、合成前方 2 m 障碍、cloud 与 UDP sender 同时运行：

- `source_sequence=46`
- `source_age_ms=33.58`
- `status_text=OK`
- sender profile：`mission_planner_demo`
- sender observed send rate：`10.0 Hz`
- sender state：`PORT_OPEN`，demo 明确关闭 heartbeat 要求
- `source_to_mavlink_ms=83.2`
- 扇区发布者数量：1
- 恢复迁移：`DEGRADED_SECTOR_RECOVERY -> OK`
- 发送错误计数：0

cloud diagnostics 已确认包含 input topic/frame/publisher/QoS、profile、安全阈值、前一状态、状态迁移、输入间隔 P95/最大值、处理时延、source-to-publish、TF 错误和拒绝帧原因。

## 配置与错误路径

flight + installation 配置查询：

- `target_frame=base_link`
- `debug.publish_markers=false`
- `runtime.profile_name=flight_low_speed`
- cloud warn/fail：400/800 ms
- sender warn/fail：600/1000 ms

不存在的 rosbag 路径启动 replay launch 时明确报告 `rosbag 路径不存在` 并非零退出。

## 静态检查

已通过：

- Python compileall；
- Bash `bash -n`；
- ShellCheck 0.8.0 warning 级别；
- YAML/JSON/XML 解析；
- UTF-8 无 BOM、LF 行尾；
- Markdown 本地链接；
- shebang 可执行位；
- 个人路径、真实飞控 ID 和示例实机 LiDAR IP 搜索；
- systemd-analyze verify。

systemd verify 仅报告 WSL 自带 snap/netplan 和测试替换用户 `nobody` 的环境警告，AeroHalo360 unit 无语法错误。

## 发布包 smoke test

首版归档已解压到独立 `/tmp/aero_smoke_20260712_1` 工作空间，未引用开发源码树。静态检查通过，Release 构建通过，10 个 CTest 入口全部通过，`colcon test-result` 为 46 tests、0 errors、0 failures、0 skipped。更新本报告后重新生成最终确定性归档；报告内容变化不影响已验证的源码、构建图或测试集合。

## HIL 阻塞项

以下项目未在本地自动测试中完成，全部阻塞飞行：

- MID360 到 `base_link` TF、安装偏航和四方向实机确认；
- 自遮挡、地面、机臂、脚架和近盲区标定；
- 9 至 11 Hz 连续 30 分钟和时间戳/最大间隔报告；
- 115200/460800/921600 UART 误码与稳定设备路径比较；
- kill cloud/sender、停 Livox、断 TF、拔 UART/USB、Jetson 掉电；
- Loiter、AUTO、GUIDED 的 proximity timeout、模式降级和人工接管；
- 一小时 soak、内存/FD、CPU、温度和 P99 sensor-to-MAVLink；
- 低反射、斜面与真实飞行场景。

sender 或 Jetson 完全失效时无法继续发送虚拟墙，不能把内部 watchdog 宣传为整机级 fail-closed。
