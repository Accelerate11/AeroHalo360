# 性能与回放基准

本仓库提供采集工具，不附带实机 rosbag。发布前自动测试只能验证工具和合成链路；真实 P50/P95/P99、CPU、内存、温度和一小时 soak 仍是飞行阻塞项。

## 场景集

在目标 Jetson 功耗模式下分别录制或回放：

1. 静态墙；
2. 快速接近；
3. 快速移开；
4. 0.3 至 1.5 m 窄柱；
5. 孤立噪点与低反射目标；
6. 地面和机体固定回波。

每个 bag 必须记录 `/livox/lidar`、TF、扇区和 diagnostics，保存雷达频率、最大间隔和时间戳模式。

## 采集命令

先启动 replay 或真实链路，再在新 shell 运行：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
mkdir -p "$HOME/aero_halo_reports"
ros2 run aero_halo_360 benchmark_latency.py \
  --scenario fast_approach \
  --duration 60 \
  --output "$HOME/aero_halo_reports/fast_approach.json"
~~~

输出包括 cloud 处理时延、source-to-publish、source-to-MAVLink、输入间隔、处理 Hz、单进程 CPU 百分比和结束 RSS 的 P50/P95/P99/最大值。若没有收到处理时延 diagnostics，命令以非零状态退出。

## 参数解释

10 Hz 下 `clear_frames=3` 约为 0.3 秒净空确认；`receding_alpha=0.4` 会让远离变化平滑但产生拖尾。demo 可关闭 temporal/inflation 用于检查原始方向，flight 必须保留安全滤波并以回放结果确定最终值。
