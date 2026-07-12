# 快速开始

## 新 shell 环境

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
~~~

## Mission Planner 拆桨演示

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 launch aero_halo_360 mission_planner_demo.launch.py \
  start_mavlink:=false
~~~

另一个新 shell 发布前方 2 米合成墙：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 run aero_halo_360 synthetic_cloud_generator.py --ros-args \
  -p wall_direction:=front \
  -p wall_distance_m:=2.0
~~~

第三个新 shell 查看扇区：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 topic echo /aero_halo_360/sector_distances --once
~~~

预期方向：

- front：sector 0。
- right：sector 18。
- rear：sector 36。
- left：sector 54。

## 配置优先级

~~~text
base_config < profile/profile_config < 显式 CLI
~~~

示例：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 launch aero_halo_360 aero_halo_360.launch.py \
  profile:=flight_low_speed \
  input_topic:=/livox/lidar \
  target_frame:=base_link \
  start_mavlink:=false
~~~

未提供的 CLI 参数不会覆盖 YAML。

## 临时 LiDAR TF

只允许在拆桨检查中使用测量后的值：

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/AeroHalo360_ws/install/setup.bash"
ros2 launch aero_halo_360 mission_planner_demo.launch.py \
  use_static_lidar_tf:=true \
  lidar_parent_frame:=base_link \
  lidar_child_frame:=livox_frame \
  lidar_xyz:="X Y Z" \
  lidar_rpy:="ROLL PITCH YAW"
~~~

单位分别为米和弧度。正式 profile 必须使用 base_link，并完成前右后左实测。

## 只读预检

~~~bash
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/AeroHalo360_ws/install/setup.bash"
cd "$HOME/AeroHalo360_ws/src/aero-halo-360"
bash nano_deploy/04_preflight.sh --help
~~~
