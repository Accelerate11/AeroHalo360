# 安全说明

AeroHalo360 v0.2.1-alpha 只提供障碍距离输入，当前未完成飞行认证。

## 软件能保护的范围

只在以下条件同时成立时，sender 内部 fail-closed 才能发送虚拟墙：

- Jetson 仍供电并运行；
- sender 进程仍运行或已被 supervisor 恢复；
- MAVLink 设备与线路可写；
- 飞控仍接收该 MAVLink 通道；
- ArduPilot proximity 处理未超时或禁用。

## 软件无法覆盖的故障

- Jetson 掉电或系统崩溃；
- sender 持续崩溃；
- UART/USB 物理断线；
- 飞控端口失效；
- 飞控拒收 sysid/compid；
- MAVLink no-forward 或路由配置错误；
- 飞控模式不使用 proximity；
- 传感器本身未覆盖目标。

这些故障必须依赖 ArduPilot 侧 timeout、模式降级、独立 failsafe 与人工接管，不能宣传为“虚拟墙仍有效”。

## 当前阻塞飞行项目

- base_link TF 与安装偏航未标定；
- MID-360 的 +X、接口方向和机体前向未完成四方向确认；
- self-mask 示例未标定，默认不加载；
- 地面、机臂、脚架和近盲区未完成覆盖率测试；
- 0.10～0.40 m 过近目标、细杆、低反射、斜面未完成 HIL；
- P99 sensor 到 MAVLink 延时尚无实机数据；
- UART 波特率和误码尚未比较；
- 一小时 soak、温度、CPU、内存和 FD 增长尚未验收；
- Loiter/AUTO/GUIDED 的实机故障行为尚未记录。

## 强制顺序

1. 合成点云；
2. 拆桨 Mission Planner；
3. 故障注入；
4. 约束或空桨；
5. 低速空旷；
6. 最后评估 AUTO/GUIDED/RTL。

任何 TF、安装、自遮挡、profile 或飞控参数变化后都必须重新执行方向与失效测试。
