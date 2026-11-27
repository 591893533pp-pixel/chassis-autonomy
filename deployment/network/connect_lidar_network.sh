#!/usr/bin/env bash
# Add a temporary secondary address for the discovered MID360S devices.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo '请使用 sudo bash 执行本脚本'; exit 1; }
[[ "$(cat /sys/class/net/enp85s0/carrier)" == 1 ]] || { echo '雷达网口没有物理链路'; exit 1; }
if ! ip -4 -o address show dev enp85s0 | awk '{print $4}' | grep -Fxq '192.168.1.4/24'; then
  echo '检查 192.168.1.4 是否冲突…'
  arping -D -I enp85s0 -c 3 -w 5 192.168.1.4 || { echo '地址冲突或检查失败，未修改网口'; exit 1; }
  ip address add 192.168.1.4/24 dev enp85s0
fi
ip -br address show dev enp85s0
echo '临时地址已就绪；原地址和 Wi-Fi 保留。没有启动运动程序。'
