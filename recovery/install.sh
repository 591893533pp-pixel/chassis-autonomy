#!/usr/bin/env bash
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
  echo '请使用 sudo bash /home/pc/chassis-autonomy/recovery/install.sh' >&2
  exit 1
fi
root=/home/pc/chassis-autonomy
[[ -f "$root/recovery/lidar_boot_guard.py" ]]
getent passwd pc >/dev/null
# The target unit runs as pc; elevated permission is only needed for installation.
install -m 0644 "$root/recovery/chassis-lidar-boot-guard.service" /etc/systemd/system/chassis-lidar-boot-guard.service
systemctl daemon-reload
systemctl enable --now chassis-lidar-boot-guard.service
systemctl --no-pager --full status chassis-lidar-boot-guard.service
