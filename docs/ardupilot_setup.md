# ArduPilot 与 Mission Planner 设置

所有参数都只是验证起点。修改前必须备份完整参数，并核对飞控固件版本、目标板卡 SerialN 映射和遥控接管。

## Stage A：仅可视化

~~~text
PRX1_TYPE   = 2
AVOID_ENABLE = 0
OA_TYPE      = 0
~~~

目标：

- MAVLink Inspector 能看到 MAVLink 2 OBSTACLE_DISTANCE。
- frame=12，即 MAV_FRAME_BODY_FRD。
- distances 长度为 72，increment=5°。
- Proximity 中前右后左与实物一致。
- 不让飞控自动产生避障动作。

## Stage B：拆桨故障测试

依次测试：

- 停 Livox；
- 缺失 TF；
- 冻结、重复和倒退点云时间戳；
- 杀 cloud；
- 杀 sender；
- 拔 USB/UART；
- 重启 Jetson。

必须区分：上游失效但 sender 仍运行时可发送虚拟墙；sender、串口、Jetson 或供电失效时不能发送虚拟墙。

## Stage C：约束或空桨 Loiter

在 Stage A/B 全部通过后，才评估 Simple Avoidance。起点示例：

~~~text
AVOID_ENABLE = 7
AVOID_MARGIN = 2.0
AVOID_BEHAVE = 1
~~~

保持人工接管，先 Stop，再评估 Slide。

## Stage D：低速空旷飞行

- 空旷场地；
- 单方向障碍；
- 低速开始；
- 持续保留遥控接管；
- 记录飞控日志与 telemetry log。

## Stage E：AUTO/GUIDED/RTL

最后才评估 BendyRuler。起点示例：

~~~text
OA_TYPE          = 1
OA_BR_TYPE       = 1
OA_BR_LOOKAHEAD  = 5
OA_MARGIN_MAX    = 2
OA_DB_SIZE       = 100
OA_DB_EXPIRE     = 2
OA_DB_OUTPUT     = 3
~~~

## MAVLink 端口参数版本差异

ArduPilot 4.7+ 使用 MAVx_* 与 MAVx_OPTIONS。较早版本常见 SRx_* 和 SERIALx_OPTIONS。串口实例编号按“启用 MAVLink 的端口顺序”映射，不能只凭物理 TELEM 标签猜测。

MAVx_EXTRA3 控制飞控生成的 DISTANCE_SENSOR、RANGEFINDER 等流率，不是接收或转发 OBSTACLE_DISTANCE 的开关。

MAVx_OPTIONS 的 no-forward 位会阻止跨 MAVLink 通道转发。MAV_OPTIONS 还可能限制接受的 sysid。不要建议无条件清零整个 OPTIONS 位掩码，应先读取、记录并只修改经过理解的位。

官方资料：

- https://ardupilot.org/copter/docs/common-mavlink-configuration.html
- https://ardupilot.org/copter/docs/common-serial-options.html
- https://ardupilot.org/copter/docs/common-simple-object-avoidance.html
- https://ardupilot.org/copter/docs/common-proximity-landingpage.html

## Mission Planner 路由

Mission Planner 和 sender 不能同时直接打开同一个串口。

可选拓扑：

1. sender 使用飞控 USB，Mission Planner 使用独立遥测链路，由飞控路由。
2. 正式 UART 接飞控，Mission Planner 使用另一个 MAVLink 通道。
3. 经过验证的 mavlink-router 独占串口并向多个 UDP 端点转发。
4. 纯 demo 将 sender 直接输出 Mission Planner UDP；这只证明可视化，不证明飞控收到消息。

在 Inspector 检查：

- OBSTACLE_DISTANCE 约 9～11 Hz；
- sysid/compid 来源符合预期；
- frame=12；
- increment=5；
- min/max 与 ROS 扇区消息一致；
- 72 个方向值随实物变化。
