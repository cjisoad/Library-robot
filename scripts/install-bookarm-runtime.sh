#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BOOKARM_ROOT='/home/boreas/project/lingzu_arm/lingzu-bookarm-py'
SERVICE_FILE='/etc/systemd/system/lr-bookarm-runtime.service'
ENV_DIR='/etc/lr-robot'
ENV_FILE="$ENV_DIR/bookarm.env"
RUNTIME_USER='boreas'

usage() {
  cat <<'EOF'
安装机器人本机的 MotorStudio 机械臂运行服务。

用法：
  sudo ./scripts/install-bookarm-runtime.sh [--python /path/to/python] [--port 18080] [--can can0] [--restart]

服务只监听 127.0.0.1。车端 ROS Edge Gateway 是唯一调用方，网页与控制站不能直接访问机械臂 API。
安装不会连接、使能或移动机械臂。
EOF
}

python_bin='/home/boreas/miniconda3/bin/python3'
port='18080'
can_interface='can0'
restart=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --python) python_bin="${2:?--python 需要路径}"; shift 2 ;;
    --port) port="${2:?--port 需要端口}"; shift 2 ;;
    --can) can_interface="${2:?--can 需要接口名}"; shift 2 ;;
    --restart) restart=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "错误：未知参数：$1" >&2; exit 1 ;;
  esac
done

[[ "$(id -u)" -eq 0 ]] || { echo '错误：请使用 sudo 执行。' >&2; exit 1; }
[[ -x "$python_bin" ]] || { echo "错误：找不到 Python：$python_bin" >&2; exit 1; }
[[ -d "$BOOKARM_ROOT" ]] || { echo "错误：找不到机械臂项目：$BOOKARM_ROOT" >&2; exit 1; }
[[ "$port" =~ ^[0-9]{2,5}$ ]] && ((port >= 1024 && port <= 65535)) || {
  echo '错误：端口必须为 1024-65535。' >&2; exit 1;
}
[[ "$can_interface" =~ ^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$ ]] || {
  echo '错误：CAN 接口名格式不正确。' >&2; exit 1;
}

# The project is intentionally run from its checkout rather than globally
# installed, so validate imports from the same working directory as systemd.
runuser -u "$RUNTIME_USER" -- bash -c '
  cd "$1"
  exec "$2" -c "import PyQt6, fastapi, uvicorn, webui.backend.api"
' bash "$BOOKARM_ROOT" "$python_bin" || {
  echo '错误：机械臂运行环境缺少 PyQt6、FastAPI、Uvicorn 或 WebUI 模块。' >&2
  exit 1
}

install -d -o root -g "$RUNTIME_USER" -m 0750 "$ENV_DIR"
if [[ ! -f "$ENV_FILE" ]]; then
  umask 077
  cat >"$ENV_FILE" <<EOF
LR_BOOKARM_PYTHON=$python_bin
LR_BOOKARM_PORT=$port
LR_BOOKARM_CAN_INTERFACE=$can_interface
EOF
  chown root:"$RUNTIME_USER" "$ENV_FILE"
  chmod 0640 "$ENV_FILE"
fi

install -o root -g root -m 0755 "$ROOT_DIR/scripts/start-bookarm-runtime.sh" /usr/local/libexec/lr-start-bookarm-runtime.sh
cat >"$SERVICE_FILE" <<EOF
[Unit]
Description=LR 机器人本机 MotorStudio 图书机械臂服务
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$RUNTIME_USER
WorkingDirectory=$BOOKARM_ROOT
Environment=HOME=/home/boreas
EnvironmentFile=-$ENV_FILE
ExecStart=/usr/local/libexec/lr-start-bookarm-runtime.sh
Restart=on-failure
RestartSec=5
TimeoutStopSec=30
KillMode=control-group

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable lr-bookarm-runtime.service
if [[ "$restart" == true ]]; then
  systemctl restart lr-bookarm-runtime.service
  sleep 2
  systemctl --no-pager --full status lr-bookarm-runtime.service
else
  echo '已安装 lr-bookarm-runtime.service。确认机械臂周围安全后启动：'
  echo '  sudo systemctl start lr-bookarm-runtime.service'
fi
