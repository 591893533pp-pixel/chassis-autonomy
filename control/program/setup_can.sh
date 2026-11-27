#!/usr/bin/env bash
# Configure interface only; sends no application CAN frames and enables no motors.
set -euo pipefail
CAN_INTERFACE="${1:-can0}"
if [[ ! "$CAN_INTERFACE" =~ ^can[0-9]+$ ]]; then
  echo 'Usage: ./setup_can.sh [can0]' >&2; exit 2
fi
if [[ "$EUID" -ne 0 ]]; then
  if [[ "$CAN_INTERFACE" == can0 && -x /usr/local/sbin/rk3588-link-can ]]; then
    exec sudo -n /usr/local/sbin/rk3588-link-can can0
  fi
  exec sudo -- bash "$0" "$CAN_INTERFACE"
fi
# Refuse to disrupt an already active bus. Reusing an identical configuration is OK.
CAN_LINK_JSON="$(ip -details -json link show dev "$CAN_INTERFACE")"
CAN_LINK_STATE="$(python3 -c '
import json,sys
x=json.loads(sys.argv[1])[0];d=x.get("linkinfo",{}).get("info_data",{})
if "UP" not in x.get("flags",[]): print("down")
elif x.get("mtu")==16 and d.get("bittiming",{}).get("bitrate")==1000000 and not any(v in str(d.get("ctrlmode",[])).lower() for v in ("listen-only","loopback","fd")): print("ready")
else: print("conflict")
' "$CAN_LINK_JSON")"
if [[ "$CAN_LINK_STATE" == conflict ]]; then
  echo '接口已启动且参数不符。先退出所有CAN控制程序，再手动将接口down后重试。' >&2; exit 1
fi
if [[ "$CAN_LINK_STATE" == down ]]; then
  ip link set dev "$CAN_INTERFACE" type can bitrate 1000000 sample-point 0.75 fd off listen-only off loopback off restart-ms 0
  ip link set dev "$CAN_INTERFACE" txqueuelen 20
  ip link set dev "$CAN_INTERFACE" up
fi
ip -details -statistics link show dev "$CAN_INTERFACE"
echo '接口配置完成：经典CAN 1 Mbps；未发送电机使能或运动命令。'
