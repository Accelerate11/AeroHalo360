# 调参指南

所有飞行候选参数都必须从已记录的 base、profile 和安装参数文件组合生成。修改源码树中的 YAML 后，非 symlink 安装需要重新构建；也可以通过 launch 的绝对配置路径加载外部文件。

## TF 与安装方向

ROS `base_link` 使用 FLU：X 前、Y 左、Z 上。MAVLink `MAV_FRAME_BODY_FRD(12)` 使用 X 前、Y 右、Z 下。AeroHalo360 将 FLU 平面角转换为顺时针 BODY_FRD 扇区：

| 障碍方向 | ROS 点坐标符号 | MAVLink 扇区 |
| --- | --- | --- |
| 前 | +X | 0 |
| 右 | -Y | 18 |
| 后 | -X | 36 |
| 左 | +Y | 54 |

MID-360 外壳的 +X 标记与 M12 接口方向相反。安装偏航初值只能来自实物测量，不能只看接口方向。复制 `config/install_x7_mid360.example.yaml` 为本机文件，填写 `xyz/rpy`，将 `calibration_status` 改为真实记录状态后，再显式加载：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 launch aero_halo_360 aero_halo_360.launch.py \
  profile:=flight_low_speed \
  installation_config:="$HOME/aero_halo_config/install_x7_mid360.yaml"
~~~

找不到 TF 必须保持 fail-closed。拆桨后分别在前、右、后、左放置唯一近障碍并运行：

~~~bash
ros2 run aero_halo_360 check_sector_directions.py --direction front
ros2 run aero_halo_360 check_sector_directions.py --direction right
ros2 run aero_halo_360 check_sector_directions.py --direction rear
ros2 run aero_halo_360 check_sector_directions.py --direction left
~~~

四项未全部通过时禁止进入飞行阶段。

## 高度、地面与近盲区

`height_filter` 以 `base_link` 为参考。先在 RViz 同时显示原始点云和过滤点云，记录地面、机臂、脚架、支架和雷达近盲区。不得为了消除地面回波而把真实低矮障碍一并过滤。对 0.10 至 0.40 m 障碍逐步接近和移开，确认过近点保持在最小距离，而不是变成净空。

## 自遮挡

`self_masks_x7_mid360.yaml` 是未标定示例，默认不会加载。标定步骤：

1. 拆除螺旋桨并固定机体。
2. 记录原始点云 30 至 60 秒。
3. 识别随机构固定的机臂、脚架和支架点。
4. 每次只调整一个包围盒，并保存尺寸与日期。
5. 从包围盒边缘外侧放置细杆，验证真实障碍没有被误删。
6. 全部点被过滤时，flight profile 必须进入 `DEGRADED_EMPTY_CLOUD`。

## 机体膨胀

`vehicle.radius_m + safety_extra_m` 决定相邻扇区膨胀角。默认 `max_inflate_bins=36` 允许最近距离覆盖所需的几何上限；参数校验会拒绝不足的上限。演示配置可以关闭膨胀，flight profile 不得照搬。

## 时间滤波

10 Hz 下，`clear_frames=3` 约等于连续 0.3 秒净空确认。接近障碍立即更新，远离使用 `receding_alpha=0.4` 平滑，因此画面会拖尾。所有更改都要用静态墙、快速接近、快速移开、窄柱、噪点和地面 rosbag 重跑时延基准。

## Watchdog

默认关系为 cloud `warn=400 ms`、`fail=800 ms`，sender `warn=600 ms`、`fail=1000 ms`。sender 的失效阈值晚于 cloud，避免无上游原因地抢先降级。单次 332 ms 间隔不触发虚拟墙；超过 warning 阈值会诊断告警，超过 fail 阈值进入虚拟墙。退出虚拟墙需要连续 3 个新健康源序号。

这些值是基于已见 332 ms 抖动的低速候选起点，不是最终飞行认证值。最终阈值必须由目标功耗模式下的 P99 sensor-to-MAVLink 时延和制动风险预算确定。任何 fail-open 都必须显式配置并承担风险。
