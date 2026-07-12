# 系统架构

AeroHalo360 位于 ROS 2 点云与 ArduPilot 之间，只负责近距离障碍感知和协议桥接，不负责 SLAM、定位、轨迹规划或飞行控制。

## 数据流

1. Livox MID360 通过 livox_ros_driver2 发布 sensor_msgs/msg/PointCloud2。
2. cloud_processor_node 使用 TF 将点云转换到 base_link。
3. 节点依次执行有限值检查、距离过滤、高度过滤、自遮挡剔除、体素降采样和半径离群点过滤。
4. Sectorizer 将水平面划分为 72 个 5 度扇区，并计算每个扇区的保守距离。
5. TemporalFilter 对障碍接近与离开采用不同策略，减少跳变和闪烁。
6. Inflation 根据机体半径和安全余量向相邻扇区扩张障碍。
7. Watchdog 检查点云超时、TF 连续失败、原始空点云、处理频率过低等异常。
8. mavlink_obstacle_sender.py 使用 MAVLink2 发送 OBSTACLE_DISTANCE 和 heartbeat。

## 坐标约定

ROS base_link 使用 FLU：

~~~text
x 向前
y 向左
z 向上
~~~

MAVLink MAV_FRAME_BODY_FRD 使用 FRD：

~~~text
x 向前
y 向右
z 向下
~~~

因此水平角按下面的公式换算：

~~~cpp
theta_deg = atan2(-y, x)
~~~

角度从机头开始顺时针增加：

- 前方：sector 0
- 右方：sector 18
- 后方：sector 36
- 左方：sector 54

## v0.2 安全设计

- cloud processor 与 MAVLink sender 分别维护独立 Watchdog，避免任一进程失效时继续发送旧数据。
- 启动无数据和输入超时默认采用 fail-closed 虚拟墙。
- 原始点云为空视为传感器异常；非空点云经过过滤后为空可表示有效净空，并发布警告状态。
- MAVLink sender 独立检查扇区消息新鲜度，不依赖 cloud processor 的诊断状态。
- SectorDistances.header.stamp 保留原始点云时间戳，诊断消息使用实际发布时间。
