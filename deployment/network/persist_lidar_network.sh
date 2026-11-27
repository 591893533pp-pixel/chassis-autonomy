#!/usr/bin/env bash
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo '请用 sudo bash 执行'; exit 1; }
PROFILE='lidar-enp85s0-static'
[[ "$(nmcli -g connection.interface-name connection show "$PROFILE")" == enp85s0 ]] || exit 1
[[ "$(nmcli -g ipv4.method connection show "$PROFILE")" == manual ]] || exit 1
if ! nmcli -g ipv4.addresses connection show "$PROFILE" | tr ',' '\n' | grep -Fxq '192.168.1.4/24'; then
  nmcli -g ipv4.addresses connection show "$PROFILE" | tr ',' '\n' | grep -Fxq '192.168.16.4/24' || { echo '原网口配置已变化，停止'; exit 1; }
  nmcli connection modify "$PROFILE" +ipv4.addresses 192.168.1.4/24
fi
priority=$(nmcli -g connection.autoconnect-priority connection show "$PROFILE")
if (( priority < 100 )); then priority=100; fi
nmcli connection modify "$PROFILE" connection.autoconnect yes connection.autoconnect-priority "$priority"
echo '已保存重启后自动恢复的雷达地址；当前链路不重启，Wi-Fi 不变。'
nmcli -f connection.interface-name,connection.autoconnect,ipv4.addresses connection show "$PROFILE"
