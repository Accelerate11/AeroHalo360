# 测试 SOP

每次改变 TF、安装、自遮挡、profile、飞控版本或串口参数，都要从 Stage A 重新执行。所有飞控参数先备份，并保持遥控人工接管。

## Stage A：拆桨可视化

飞控保持：

~~~text
PRX1_TYPE=2
AVOID_ENABLE=0
OA_TYPE=0
~~~

启动 demo 后，确认 `OBSTACLE_DISTANCE` 为 MAVLink 2、BODY_FRD(12)、72 个 5 度扇区且约 9 至 11 Hz。不得让 Mission Planner 和 sender 同时直接打开同一个串口。

## Stage B：方向与失效

分别放置前、右、后、左唯一近障碍：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
for direction in front right rear left; do
  ros2 run aero_halo_360 check_sector_directions.py --direction "$direction" || exit 1
done
~~~

验证 0/18/36/54 扇区后，依次执行：停 Livox、杀 cloud、杀 sender、断 TF、拔 UART/USB、Jetson 掉电。记录进入虚拟墙或飞控侧 proximity timeout 的时间、恢复滞回、飞控模式和人工接管结果。sender 或 Jetson 已退出时，软件不能继续发送虚拟墙。

## 过滤与边界场景

必须覆盖：

- 0.10、0.20、0.24、0.30、0.40 m 接近和撤离；
- 0.3 至 1.5 m 细杆的全方向；
- 原始空点云；
- 非空点云被高度或错误 self-mask 全部过滤；
- 低反射面、斜面、地面、噪点；
- 零、重复、冻结、过旧和倒退时间戳；
- 缺字段、错误 endian、截断、NaN/Inf 和超限 PointCloud2。

flight profile 中过滤后空必须是 `DEGRADED_EMPTY_CLOUD`，不能按全向净空处理。

## Stage C：约束或空桨 Loiter

只有 Stage A/B 全部通过后才评估 Loiter Simple Avoidance。自动避障参数只作为起点，先验证接近、撤离、悬停、遥控覆盖和所有故障注入。

## Stage D：低速空旷飞行

初始速度不高于 0.5 m/s，障碍起始距离不小于 1.5 m，现场具备隔离区、观察员和立即接管条件。每次只改变一个变量。

## Stage E：任务模式

最后才评估 AUTO、GUIDED、RTL 与 BendyRuler。Loiter 通过不代表任务模式通过，必须分别记录各模式的传感器超时、路径规划和人工接管行为。
