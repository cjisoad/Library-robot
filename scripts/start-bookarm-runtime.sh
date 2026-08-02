#!/usr/bin/env bash
set -eo pipefail

# The MotorStudio WebUI remains loopback-only.  ROS calls it locally through
# Edge Gateway; neither the browser nor the MQTT broker can reach CAN/serial.
BOOKARM_ROOT='/home/boreas/project/lingzu_arm/lingzu-bookarm-py'
BOOKARM_ENV='/etc/lr-robot/bookarm.env'

if [[ -r "$BOOKARM_ENV" ]]; then
  # shellcheck disable=SC1090
  source "$BOOKARM_ENV"
fi

PYTHON_BIN="${LR_BOOKARM_PYTHON:-/home/boreas/miniconda3/bin/python3}"
PORT="${LR_BOOKARM_PORT:-18080}"
CAN_INTERFACE="${LR_BOOKARM_CAN_INTERFACE:-can0}"

[[ -d "$BOOKARM_ROOT" ]] || {
  echo "找不到机械臂项目：$BOOKARM_ROOT" >&2
  exit 1
}
[[ -x "$PYTHON_BIN" ]] || {
  echo "找不到机械臂 Python：$PYTHON_BIN" >&2
  exit 1
}
[[ "$PORT" =~ ^[0-9]{2,5}$ ]] && ((PORT >= 1024 && PORT <= 65535)) || {
  echo "LR_BOOKARM_PORT 必须是 1024-65535 之间的端口" >&2
  exit 1
}
[[ "$CAN_INTERFACE" =~ ^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$ ]] || {
  echo "LR_BOOKARM_CAN_INTERFACE 不是有效的 CAN 接口名" >&2
  exit 1
}

cd "$BOOKARM_ROOT"
exec "$PYTHON_BIN" -m webui.backend.api \
  --host 127.0.0.1 \
  --port "$PORT" \
  --can "$CAN_INTERFACE" \
  --backend socketcan \
  --no-frontend \
  --log-level info
