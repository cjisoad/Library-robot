# Library-robot

面向 Ubuntu 24.04 + ROS 2 Jazzy 的实机机器人工作空间，包含：

- 雷神激光驱动 `lslidar_driver`
- 雷神激光消息定义 `lslidar_msgs`
- 底盘控制、IMU 驱动与里程计 `imu_car_ros2`
- Nav2 导航与建图启动整合包 `mobile_robot_nav_bringup`
- 语音模块命令读取与交互桥接 `mobile_robot_voice_interaction`
- Cartographer 实车建图启动整合包 `mobile_robot_cartographer_bringup`

当前版本已经按 Raspberry Pi 5 实机运行做过参数和启动策略调整，默认工作流面向差速底盘、2D 激光雷达、IMU 和静态地图导航。

这个仓库本身就是一个 ROS 2 工作空间。
克隆下来后，仓库根目录就是工作空间根目录，不需要你再额外新建一层工作空间。

## 系统依赖

基础环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- `colcon`
- `python3-serial`
- `libpcap-dev`
- `libpcl-dev`

建议先安装 ROS 2 Jazzy 桌面版，或至少安装以下核心包：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-serial \
  libpcap-dev \
  libpcl-dev \
  ros-jazzy-cartographer-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-opennav-docking
```

如果机器已经装好了 `rosdep`，也可以在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 工作空间结构

```text
Library-robot/
├── archive/
│   └── Turntable_ctrl/
│       ├── COLCON_IGNORE
│       ├── README.md
│       └── src/
├── maps/
│   ├── my_map.yaml
│   └── my_map.pgm
└── src/
    ├── imu_car_ros2/
    │   ├── config/car_params.yaml
    │   └── imu_car_ros2/
    ├── lslidar_driver/
    │   ├── params/lsx10.yaml
    │   └── rviz/lslidar.rviz
    ├── lslidar_msgs/
    ├── mobile_robot_cartographer_bringup/
    │   ├── config/mobile_robot_2d.lua
    │   ├── launch/cartographer_mapping.launch.py
    │   └── launch/save_map.launch.py
    ├── mobile_robot_voice_interaction/
    │   ├── config/speech_interaction_example.yaml
    │   ├── launch/speech_interaction.launch.py
    │   └── mobile_robot_voice_interaction/
    └── nav2_minimal_bringup/
        ├── config/nav2_params.yaml
        ├── launch/full_navigation.launch.py
        ├── launch/navigation.launch.py
        ├── launch/slam.launch.py
        └── launch/save_map.launch.py
```

各模块职责：

- `imu_car_ros2`
  负责底盘串口控制、IMU 串口读取、轮速与 IMU 融合里程计发布。
- `lslidar_driver`
  负责雷达驱动和 `/scan` 发布。
- `lslidar_msgs`
  提供雷达驱动依赖的消息定义。
- `mobile_robot_nav_bringup`
  负责导航、定位、建图、保存地图以及整车一键启动入口。
- `mobile_robot_voice_interaction`
  负责语音模块串口接入、命令词识别结果发布，以及“听到什么发什么信号 / 收到什么信号播什么话”的桥接接口。
- `mobile_robot_cartographer_bringup`
  负责 Cartographer 实车建图启动、占据栅格发布以及地图保存入口。
- `archive/Turntable_ctrl`
  仅保存原始升降台控制相关源码，目录名沿用历史命名，供后续参考，不参与 ROS 工作空间构建。

## 默认硬件接口

当前默认串口和接口配置如下，如设备编号不同请先改配置文件：

- 底盘串口：`/dev/chassis_serial_port`
- IMU 串口：`/dev/imu_port`
- 雷达串口：`/dev/laser_port`
- 语音模块：`/dev/speech_port`
- 升降台串口：`/dev/lift_port`

对应配置文件：

- 底盘/IMU/里程计：[src/imu_car_ros2/config/car_params.yaml](src/imu_car_ros2/config/car_params.yaml)
- 雷达：[src/lslidar_driver/params/lsx10.yaml](src/lslidar_driver/params/lsx10.yaml)
- 导航参数：[src/nav2_minimal_bringup/config/nav2_params.yaml](src/nav2_minimal_bringup/config/nav2_params.yaml)
- 语音交互：[src/mobile_robot_voice_interaction/config/speech_interaction_example.yaml](src/mobile_robot_voice_interaction/config/speech_interaction_example.yaml)
- Cartographer：[src/mobile_robot_cartographer_bringup/config/mobile_robot_2d.lua](src/mobile_robot_cartographer_bringup/config/mobile_robot_2d.lua)

## 克隆与编译

先克隆仓库：

```bash
git clone git@github.com:cjisoad/Library-robot.git
cd Library-robot
```

仓库根目录已经是工作空间根目录，直接在这里编译，不需要再额外创建新的工作空间：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

说明：`archive/` 下的归档源码不在 `src/` 中，且带有 `COLCON_IGNORE`，不会参与 `colcon build`，也不会向当前工作空间导出 ROS 包环境。

## 基本启动

整车导航一键启动：

```bash
cd Library-robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_robot_nav_bringup full_navigation.launch.py
```

如果需要同时打开 Nav2 RViz：

```bash
ros2 launch mobile_robot_nav_bringup full_navigation.launch.py use_nav_rviz:=true
```

如果只想启动导航栈，不带底盘和雷达驱动：

```bash
ros2 launch mobile_robot_nav_bringup navigation.launch.py use_rviz:=true
```

## 语音模块交互

先单独检查语音模块能否读出命令码：

```bash
cd Library-robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run mobile_robot_voice_interaction voice_cmd_reader --ros-args -p port:=/dev/speech_port
```

如果要做“听到什么话发什么信号 / 收到什么信号说什么话”，直接启动语音交互桥：

```bash
cd Library-robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_robot_voice_interaction speech_interaction.launch.py port:=/dev/speech_port
```

这个包默认发布：

- `/speech_cmd`
- `/speech_cmd_name`
- `/speech_heard_signal`

并订阅：

- `/speech_say_cmd`
- `/speech_say_name`
- `/speech_say_signal`

详细接口、参数和示例规则见：

- [src/mobile_robot_voice_interaction/README.md](src/mobile_robot_voice_interaction/README.md)

## Cartographer 建图

如果你想用 Cartographer，而不是 `slam_toolbox`，可以使用新增的 Cartographer 功能包：

```bash
cd Library-robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_robot_cartographer_bringup cartographer_mapping.launch.py
```

这套默认对齐当前实车的：

- `/scan`
- `/odom`
- `base_link`
- `laser_link`

并复用当前工作空间已有的底盘、IMU、里程计与雷达参数。

如果只想在驱动已经运行时单独起 Cartographer：

```bash
ros2 launch mobile_robot_cartographer_bringup cartographer_mapping.launch.py \
  start_robot_drivers:=false
```

如果要保存 Cartographer 当前生成的 `/map`：

```bash
ros2 launch mobile_robot_cartographer_bringup save_map.launch.py map_name:=cartographer_map
```

默认会保存到工作空间根目录：

- `maps/cartographer_map.yaml`
- `maps/cartographer_map.pgm`

详细说明见：

- [src/mobile_robot_cartographer_bringup/README.md](src/mobile_robot_cartographer_bringup/README.md)

## 建图与保存地图

建图：

```bash
cd Library-robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_robot_nav_bringup slam.launch.py
```

保存地图：

```bash
cd Library-robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mobile_robot_nav_bringup save_map.launch.py map_name:=my_map
```

默认地图保存在：

- `maps/my_map.yaml`
- `maps/my_map.pgm`

## 导航运行前提

运行导航前，需要保证：

- `/scan` 正常发布
- `/odom` 正常发布
- TF 中存在 `odom -> base_link`
- 启动定位后，在 RViz 中给出初始位姿
- 底盘能够接收 `/cmd_vel`

## 关键启动文件

- [src/nav2_minimal_bringup/launch/full_navigation.launch.py](src/nav2_minimal_bringup/launch/full_navigation.launch.py)
  整车一键启动，包含底盘、IMU、里程计、雷达、定位与导航。
- [src/nav2_minimal_bringup/launch/navigation.launch.py](src/nav2_minimal_bringup/launch/navigation.launch.py)
  只负责 Nav2 定位与导航栈。
- [src/nav2_minimal_bringup/launch/slam.launch.py](src/nav2_minimal_bringup/launch/slam.launch.py)
  建图入口。
- [src/nav2_minimal_bringup/launch/save_map.launch.py](src/nav2_minimal_bringup/launch/save_map.launch.py)
  保存地图入口。
- [src/mobile_robot_voice_interaction/launch/speech_interaction.launch.py](src/mobile_robot_voice_interaction/launch/speech_interaction.launch.py)
  语音模块交互桥启动入口。
- [src/mobile_robot_cartographer_bringup/launch/cartographer_mapping.launch.py](src/mobile_robot_cartographer_bringup/launch/cartographer_mapping.launch.py)
  Cartographer 实车建图入口。
- [src/mobile_robot_cartographer_bringup/launch/save_map.launch.py](src/mobile_robot_cartographer_bringup/launch/save_map.launch.py)
  保存 Cartographer 地图入口。

## 常见配置入口

- 地图文件：`maps/my_map.yaml`
- Nav2 参数：`src/nav2_minimal_bringup/config/nav2_params.yaml`
- 底盘与 IMU：`src/imu_car_ros2/config/car_params.yaml`
- 雷达参数：`src/lslidar_driver/params/lsx10.yaml`
- 语音规则：`src/mobile_robot_voice_interaction/config/speech_interaction_example.yaml`
- Cartographer 参数：`src/mobile_robot_cartographer_bringup/config/mobile_robot_2d.lua`

## 验证命令

检查包是否被当前工作空间接管：

```bash
source /opt/ros/jazzy/setup.bash
cd Library-robot
source install/setup.bash
ros2 pkg prefix mobile_robot_nav_bringup
ros2 pkg prefix imu_car_ros2
ros2 pkg prefix lslidar_driver
ros2 pkg prefix mobile_robot_voice_interaction
ros2 pkg prefix mobile_robot_cartographer_bringup
```

检查关键话题：

```bash
ros2 topic list | grep -E '/scan|/odom|/map|/cmd_vel|/amcl_pose'
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```
