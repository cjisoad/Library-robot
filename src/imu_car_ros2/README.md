# imu_car_ros2

ROS2 Python package for:

- four-wheel car control from `geometry_msgs/msg/Twist`
- odometry publishing from wheel speed and IMU data

## Nodes

- `car_controller`
  - subscribe: `/cmd_vel`
  - publish: `/wheel_speeds`, `/wheel_joint_states`
  - function: convert `Twist` to chassis serial commands

- `car_odometry`
  - subscribe: `/wheel_speeds`, `/wheel_joint_states`, `/imu/data_raw`
  - publish: `/odom`
  - function: fuse wheel speed with IMU orientation / angular velocity

## Launch

```bash
ros2 launch imu_car_ros2 car_with_imu.launch.py
```

Use a custom parameter file:

```bash
ros2 launch imu_car_ros2 car_with_imu.launch.py \
  params_file:=/absolute/path/to/car_params.yaml
```

## Build

In a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cp -r /Users/baotianyi/Downloads/IMU惯导模块/1.教程资料/2.IMU应用教程/imu_car_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select imu_car_ros2
source install/setup.bash
```

## Run

```bash
ros2 run imu_car_ros2 car_controller
ros2 run imu_car_ros2 car_odometry
```

or:

```bash
ros2 launch imu_car_ros2 car_with_imu.launch.py
```

## Parameters

Default parameters are in:

- `config/car_params.yaml`

You will usually need to adjust:

- `serial_port`
- `track_width`
- `wheel_radius`
- `command_scale`
- `imu_topic`

## Note

If the chassis returns real wheel-speed feedback, update
`imu_car_ros2/car_controller.py` in `parse_feedback_line()` according to the
actual serial protocol so odometry uses measured wheel speed instead of the
commanded speed estimate.
