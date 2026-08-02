#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="/home/boreas/Library_robot"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
OVERLAY_SETUP="$WORKSPACE/install/setup.bash"
IMU_PARAMS_FILE="$WORKSPACE/install/car_ctrl/share/car_ctrl/config/ddsm_hat_diff_drive.yaml"
ROBOT_ENV_FILE="/etc/lr-robot/runtime.env"

# This optional root-managed file only selects the private Gateway parameter
# file. It keeps customer network credentials out of the ROS workspace and Git.
if [ -r "$ROBOT_ENV_FILE" ]; then
  # shellcheck disable=SC1090
  source "$ROBOT_ENV_FILE"
fi
GATEWAY_PARAMS_FILE="${LR_GATEWAY_PARAMS_FILE:-$WORKSPACE/src/robot_decision/config/edge_gateway.yaml}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${LR_ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

if [ ! -f "$ROS_SETUP" ] || [ ! -f "$OVERLAY_SETUP" ]; then
  echo "ROS 运行环境不完整，请先完成机器人工作空间编译。" >&2
  exit 1
fi

if [ ! -r "$GATEWAY_PARAMS_FILE" ]; then
  echo "无法读取 Edge Gateway 参数文件：$GATEWAY_PARAMS_FILE" >&2
  exit 1
fi

# ROS setup scripts intentionally read optional environment variables that may
# be unset in a clean systemd environment, so do not enable bash nounset here.
source "$ROS_SETUP"
source "$OVERLAY_SETUP"

imu_supervisor() {
  local imu_pid exit_code

  stop_imu_supervisor() {
    if [ -n "${imu_pid:-}" ]; then
      kill -TERM "$imu_pid" 2>/dev/null || true
      wait "$imu_pid" 2>/dev/null || true
    fi
    exit 0
  }
  trap stop_imu_supervisor INT TERM

  while true; do
    echo "Starting IMU driver." >&2
    ros2 run car_ctrl imu_driver --ros-args --params-file "$IMU_PARAMS_FILE" &
    imu_pid=$!
    wait "$imu_pid" || exit_code=$?
    imu_pid=""
    echo "IMU driver exited (${exit_code:-0}); restarting in 1 second." >&2
    unset exit_code
    sleep 1
  done
}

# Navigation must own the sensors and AMCL before the gateway reads /amcl_pose.
ros2 launch mobile_robot_nav_bringup full_navigation.launch.py \
  use_nav_rviz:=false \
  use_lidar_rviz:=false \
  start_imu_driver:=false &
NAVIGATION_PID=$!

# The IMU driver owns a USB serial device and must survive transient USB I/O
# failures independently of the navigation launch process.
imu_supervisor &
IMU_SUPERVISOR_PID=$!

for _ in $(seq 1 45); do
  if ros2 node list 2>/dev/null | grep -qx '/amcl'; then
    break
  fi
  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -qx '/amcl'; then
  echo "导航定位服务未在预期时间内启动。" >&2
  exit 1
fi

ros2 launch robot_decision auto_localization.launch.py \
  params_file:="$WORKSPACE/src/robot_decision/config/auto_localization.yaml" &
AUTO_LOCALIZATION_PID=$!

cleanup() {
  local pid
  local live
  local children=(
    "$IMU_SUPERVISOR_PID"
    "$AUTO_LOCALIZATION_PID"
    "$NAVIGATION_PID"
    "$EDGE_GATEWAY_PID"
  )

  # Give ROS launches a chance to propagate a clean shutdown before systemd
  # needs to escalate.  Signalling all parents first keeps the stop bounded.
  for pid in "${children[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  for _ in $(seq 1 50); do
    live=false
    for pid in "${children[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        live=true
        break
      fi
    done
    [ "$live" = false ] && break
    sleep 0.1
  done
  for pid in "${children[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
# Exit non-zero after a service signal so systemd starts a fresh runtime.
trap 'exit 143' INT TERM

ros2 launch robot_decision edge_gateway.launch.py \
  params_file:="$GATEWAY_PARAMS_FILE" &
EDGE_GATEWAY_PID=$!
wait "$EDGE_GATEWAY_PID"
