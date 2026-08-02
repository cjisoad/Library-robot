#!/usr/bin/env bash
set -euo pipefail

# Install one robot's mTLS delivery package without exposing its MQTT password
# in the workspace, shell history, or a manually edited YAML file.
umask 077

WORKSPACE='/home/boreas/Library_robot'
ROBOT_CONFIG_DIR='/etc/lr-robot'
MQTT_DIR="$ROBOT_CONFIG_DIR/mqtt"
GATEWAY_CONFIG="$ROBOT_CONFIG_DIR/edge-gateway.yaml"
RUNTIME_ENV="$ROBOT_CONFIG_DIR/runtime.env"

robot_id=''
station_host=''
delivery_package=''
map_id=''
map_min_x=''
map_max_x=''
map_min_y=''
map_max_y=''
restart_runtime=false

usage() {
  cat <<'EOF'
用法：
  sudo install-control-station-gateway.sh \
    --robot-id LR-01 \
    --station-host 192.168.8.10 \
    --delivery-package /var/lib/lr-platform/pki/exports/robots/LR-01 \
    --map-id MAP-XXXXXXXXXXXX \
    --map-bounds <min-x> <max-x> <min-y> <max-y> [--restart]

该脚本只安装指定机器人的 MQTT 客户端证书和 Gateway 私有配置。
--restart 会在配置校验通过后重启 lr-robot-runtime.service；不发送任何移动命令。
EOF
}

die() {
  echo "错误：$*" >&2
  exit 1
}

require_value() {
  [[ $# -eq 2 && -n "$2" ]] || die "$1 需要一个值"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-id)
      require_value "$1" "${2:-}"
      robot_id="$2"
      shift 2
      ;;
    --station-host)
      require_value "$1" "${2:-}"
      station_host="$2"
      shift 2
      ;;
    --delivery-package)
      require_value "$1" "${2:-}"
      delivery_package="$2"
      shift 2
      ;;
    --map-id)
      require_value "$1" "${2:-}"
      map_id="$2"
      shift 2
      ;;
    --map-bounds)
      [[ $# -ge 5 ]] || die '--map-bounds 需要 4 个数值'
      map_min_x="$2"
      map_max_x="$3"
      map_min_y="$4"
      map_max_y="$5"
      shift 5
      ;;
    --restart)
      restart_runtime=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知参数：$1"
      ;;
  esac
done

[[ "$(id -u)" -eq 0 ]] || die '请使用 sudo 执行。'
[[ "$robot_id" =~ ^LR-[A-Za-z0-9_-]+$ ]] || die '机器人 ID 必须匹配 LR-xx 格式。'
[[ "$map_id" =~ ^MAP-[A-Za-z0-9_-]+$ ]] || die '地图 ID 必须匹配 MAP-xx 格式。'
[[ "$station_host" != *[[:space:]]* && "$station_host" != */* ]] || die '控制站地址不能包含空白或路径。'
[[ -d "$delivery_package" ]] || die "找不到机器人交付包：$delivery_package"

for value in "$map_min_x" "$map_max_x" "$map_min_y" "$map_max_y"; do
  [[ "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || die '地图边界必须是十进制数。'
done
awk -v min_x="$map_min_x" -v max_x="$map_max_x" -v min_y="$map_min_y" -v max_y="$map_max_y" \
  'BEGIN { exit !(min_x < max_x && min_y < max_y) }' || die '地图边界必须满足 min < max。'

ca_file="$delivery_package/mqtt/ca.crt"
cert_file="$delivery_package/mqtt/$robot_id.crt"
key_file="$delivery_package/mqtt/$robot_id.key"
credentials_file="$delivery_package/mqtt-credentials.env"
for file in "$ca_file" "$cert_file" "$key_file" "$credentials_file"; do
  [[ -r "$file" ]] || die "交付包缺少文件：$file"
done

openssl verify -CAfile "$ca_file" "$cert_file" >/dev/null || die '机器人客户端证书不受交付包 CA 信任。'
openssl x509 -in "$cert_file" -checkend 86400 -noout >/dev/null || die '机器人客户端证书将在 24 小时内过期。'
[[ "$(openssl x509 -in "$cert_file" -noout -subject -nameopt RFC2253)" == *"CN=$robot_id"* ]] || \
  die '交付包证书的 CN 与 --robot-id 不一致。'
cert_digest="$(openssl x509 -in "$cert_file" -pubkey -noout | openssl pkey -pubin -outform DER | openssl dgst -sha256)"
key_digest="$(openssl pkey -in "$key_file" -pubout -outform DER | openssl dgst -sha256)"
[[ "$cert_digest" == "$key_digest" ]] || die '机器人客户端证书与私钥不匹配。'

# The delivery file is root-readable. Parse only the two expected assignments;
# never source a credential file as shell code or print the password.
mqtt_username="$(awk -F= '$1 == "mqtt_username" { print $2; found += 1 } END { exit found == 1 ? 0 : 1 }' "$credentials_file")" || \
  die '交付包 MQTT 用户名格式不正确。'
mqtt_password="$(awk -F= '$1 == "mqtt_password" { print $2; found += 1 } END { exit found == 1 ? 0 : 1 }' "$credentials_file")" || \
  die '交付包 MQTT 密码格式不正确。'
[[ "${mqtt_username:-}" == "$robot_id" ]] || die '交付包 MQTT 用户名与机器人 ID 不一致。'
[[ "${mqtt_password:-}" =~ ^[A-Za-z0-9._~!@%+=:-]{16,}$ ]] || die '交付包 MQTT 密码格式不正确。'

# Verify the server certificate before replacing the local runtime files. The
# same station host is later used by Paho for SNI and hostname verification.
verify_option=(-verify_hostname "$station_host")
if [[ "$station_host" =~ ^[0-9]+([.][0-9]+){3}$ ]]; then
  verify_option=(-verify_ip "$station_host")
fi
timeout 8 openssl s_client \
  -connect "$station_host:8883" \
  -servername "$station_host" \
  -CAfile "$ca_file" \
  "${verify_option[@]}" \
  -verify_return_error \
  </dev/null 2>&1 | grep -Fq 'Verify return code: 0 (ok)' || \
  die '无法验证控制站 MQTT TLS 证书；请检查控制站地址、8883 防火墙和站点证书 SAN。'

install -d -o root -g boreas -m 0750 "$ROBOT_CONFIG_DIR" "$MQTT_DIR"
if [[ -f "$GATEWAY_CONFIG" ]]; then
  backup="$GATEWAY_CONFIG.$(date -u +%Y%m%dT%H%M%SZ).bak"
  cp -p "$GATEWAY_CONFIG" "$backup"
  chmod 0600 "$backup"
fi

install -o root -g boreas -m 0644 "$ca_file" "$MQTT_DIR/ca.crt"
install -o root -g boreas -m 0644 "$cert_file" "$MQTT_DIR/$robot_id.crt"
install -o root -g boreas -m 0640 "$key_file" "$MQTT_DIR/$robot_id.key"

temporary_config="$(mktemp "$ROBOT_CONFIG_DIR/.edge-gateway.yaml.XXXXXX")"
trap 'rm -f "$temporary_config"' EXIT
cat >"$temporary_config" <<EOF
edge_gateway_node:
  ros__parameters:
    robot_id: "$robot_id"
    mqtt_host: "$station_host"
    mqtt_port: 8883
    mqtt_protocol_version: "3.1.1"
    mqtt_tls_enabled: true
    mqtt_ca_file: "$MQTT_DIR/ca.crt"
    mqtt_client_cert_file: "$MQTT_DIR/$robot_id.crt"
    mqtt_client_key_file: "$MQTT_DIR/$robot_id.key"
    mqtt_username: "$mqtt_username"
    mqtt_password: "$mqtt_password"
    allowed_map_ids:
      - "$map_id"
    active_map_id: "$map_id"
    map_min_x: $map_min_x
    map_max_x: $map_max_x
    map_min_y: $map_min_y
    map_max_y: $map_max_y
    max_abs_yaw: 6.284
    command_queue_size: 10
    navigate_action_name: "navigate_to_pose"
    manual_cmd_vel_topic: "/cmd_vel_nav"
    manual_command_timeout_seconds: 1.0
    amcl_pose_topic: "/amcl_pose"
    initial_pose_topic: "/initialpose"
    localization_status_topic: "/localization/status"
    localization_scan_mode_topic: "/localization/scan_mode"
    chassis_status_topic: "/wheel_speed_source"
    chassis_status_timeout_seconds: 2.5
    bookarm_api_url: "http://127.0.0.1:18080/api/v1"
    bookarm_timeout_seconds: 3.0
    bookarm_poll_period_seconds: 1.0
    bookarm_rod_device: "/dev/rodmotor"
    bookarm_lift_device: "/dev/lift_port"
    telemetry_period_seconds: 0.5
    pose_stale_timeout_seconds: 5.0
    max_position_variance: 0.5
    max_yaw_variance: 0.35
EOF
install -o root -g boreas -m 0640 "$temporary_config" "$GATEWAY_CONFIG"

temporary_env="$(mktemp "$ROBOT_CONFIG_DIR/.runtime.env.XXXXXX")"
trap 'rm -f "$temporary_config" "$temporary_env"' EXIT
printf '%s\n' \
  'LR_GATEWAY_PARAMS_FILE=/etc/lr-robot/edge-gateway.yaml' \
  'LR_ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >"$temporary_env"
install -o root -g boreas -m 0640 "$temporary_env" "$RUNTIME_ENV"

# Ensure the installed overlay contains the current Gateway implementation
# before systemd restarts it.  The private MQTT configuration remains outside
# the workspace and is not read by this build step.
if [[ "$restart_runtime" == true ]]; then
  [[ -r /opt/ros/jazzy/setup.bash ]] || die '找不到 /opt/ros/jazzy/setup.bash，无法构建车端 Gateway。'
  command -v colcon >/dev/null 2>&1 || die '找不到 colcon，无法构建车端 Gateway。'
  # systemd runs the runtime as boreas.  A root-owned overlay can leave its
  # package metadata unreadable, so repair previous artifacts and build as
  # that same service account.
  for artifact_dir in "$WORKSPACE/build" "$WORKSPACE/install" "$WORKSPACE/log"; do
    [[ -e "$artifact_dir" ]] && chown -R boreas:boreas "$artifact_dir"
  done
  runuser -u boreas -- env HOME=/home/boreas bash -c '
    set -eo pipefail
    cd /home/boreas/Library_robot
    # ROS setup reads optional variables before initializing them.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon build \
      --packages-select mobile_robot_nav_bringup dual_laser_merger robot_decision \
      --symlink-install
  ' || die '车端 Gateway 构建失败；未重启运行时。'
fi

echo "已安装 $robot_id 的 mTLS Gateway 配置。"
echo "配置文件：$GATEWAY_CONFIG（root:boreas 0640）"
echo "证书目录：$MQTT_DIR（私钥 root:boreas 0640）"

if [[ "$restart_runtime" == true ]]; then
  systemctl restart lr-robot-runtime.service || \
    die '无法重启 lr-robot-runtime.service；新配置已写入但尚未启用。'
  systemctl --no-pager --full status lr-robot-runtime.service
else
  echo '尚未重启车载运行时；确认现场安全后执行：sudo systemctl restart lr-robot-runtime.service'
fi
