#!/usr/bin/env bash
# =============================================================================
#  NUC 一键部署脚本  （新机：Ubuntu 24.04 x86_64 + ROS 2 Jazzy，用户 pc）
#  用法：在解压后的包目录里，以 pc 用户执行（脚本内部按需 sudo，会要一次密码）
#      ./deploy.sh                 # 标准部署（不开机自启）
#      ./deploy.sh --autostart     # 同时打开开机自启（定位/Nav2/网关/网页）
#      ./deploy.sh --iface enp2s0  # 新机有线网口名不是 enp85s0 时指定
#  其它选项：--skip-apt --skip-network --skip-ptp --force -y -h
#  说明见同目录 read.md。本脚本不会使能电机，也不会发任何运动命令。
# =============================================================================
set -Eeuo pipefail

PKG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HOME_DIR=/home/pc
APP="$HOME_DIR/rk3588 link"
OLD_IFACE=enp85s0
IFACE=""
LAN_IP=192.168.16.4/24
DO_APT=1; DO_NET=1; DO_PTP=1; AUTOSTART=0; FORCE=0; YES=0

usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    --iface) IFACE="${2:?--iface 需要网口名}"; shift 2 ;;
    --skip-apt) DO_APT=0; shift ;;
    --skip-network) DO_NET=0; shift ;;
    --skip-ptp) DO_PTP=0; shift ;;
    --autostart) AUTOSTART=1; shift ;;
    --force) FORCE=1; shift ;;
    -y|--yes) YES=1; shift ;;
    -h|--help) usage 0 ;;
    *) echo "未知参数：$1"; usage 2 ;;
  esac
done

TS="$(date +%Y%m%d_%H%M%S)"
LOG="$HOME_DIR/nuc_deploy_$TS.log"
exec > >(tee -a "$LOG") 2>&1
WARN=()
step() { echo; echo "==================== [$1] $2"; }
ok()   { echo "  [OK] $*"; }
warn() { echo "  [WARN] $*"; WARN+=("$*"); }
die()  { echo "  [ERROR] $*"; echo "部署中止。日志：$LOG"; exit 1; }
confirm() { [[ $YES == 1 ]] && return 0; local a; read -r -p "  $1 [y/N] " a; [[ $a == y || $a == Y ]]; }
trap 'echo "  [ERROR] 第 $LINENO 行命令失败：$BASH_COMMAND"; echo "部署中止。日志：$LOG"' ERR

# ----------------------------------------------------------------------------- 0
step 0 "环境检查"
# 2026-11-23: 纯底盘版包（无升降立柱、无上半身、无 RK3588）以包根目录的 CHASSIS_ONLY.txt 标识
CHASSIS=0; [[ -f "$PKG/CHASSIS_ONLY.txt" ]] && CHASSIS=1
if [[ $CHASSIS == 1 ]]; then
  echo "  ******************************************************************"
  echo "  *  这是【纯底盘版】部署包：无可升降立柱、无上半身、无 RK3588       *"
  echo "  *  NUC 自己做 PTP 主钟；升降/上半身/RK 相关功能均不可用             *"
  echo "  *  不要部署到带升降立柱、上半身或 RK3588 的完整机器人上             *"
  echo "  ******************************************************************"
  confirm "确认这台 NUC 用于纯底盘机器人？" || exit 1
fi
[[ $EUID -ne 0 ]] || die "不要用 sudo 运行；请以 pc 用户直接执行 ./deploy.sh（内部会调用 sudo）"
[[ "$(id -un)" == pc ]] || die "必须以 pc 用户运行：程序里写死了 /home/pc/rk3588 link"
[[ "$(uname -m)" == x86_64 ]] || die "只支持 x86_64"
. /etc/os-release
if [[ "${VERSION_ID:-}" != 24.04 ]]; then confirm "系统是 $PRETTY_NAME，不是 24.04，仍然继续？" || exit 1; fi
[[ -f /opt/ros/jazzy/setup.bash ]] || die "没找到 /opt/ros/jazzy，请先装好 ROS 2 Jazzy"
ok "$PRETTY_NAME / $(uname -r) / ROS 2 Jazzy"
sudo -v || die "需要 sudo 权限"
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &
echo "  校验包内文件（约 1~3 分钟）..."
python3 "$PKG/verify.py" || die "包文件校验失败，包可能损坏，请重新拷贝"

if [[ -z "$IFACE" ]]; then
  if ip link show "$OLD_IFACE" >/dev/null 2>&1; then IFACE=$OLD_IFACE
  else
    echo "  本机没有 $OLD_IFACE。现有有线网口："; ip -br link | awk '$1 ~ /^(en|eth)/{print "    "$0}'
    die "请用 --iface <连接机器人交换机的网口> 重新运行"
  fi
fi
ip link show "$IFACE" >/dev/null 2>&1 || die "网口 $IFACE 不存在"
ok "机器人有线网口：$IFACE（原机为 $OLD_IFACE）"

if [[ -e "$APP" ]]; then
  if [[ $FORCE == 1 ]]; then
    if pgrep -f "$APP" >/dev/null; then die "$APP 里的程序正在运行，请先 ./nuc.sh stop / ./nuc_web.sh stop"; fi
    mv "$APP" "$APP.before_deploy_$TS"; ok "已把原目录改名为 $APP.before_deploy_$TS"
  else
    die "$APP 已存在。确认要替换时加 --force（原目录会改名保留，不会删除）"
  fi
fi

# ----------------------------------------------------------------------------- 1
step 1 "安装系统依赖（apt）"
if [[ $DO_APT == 1 ]]; then
  sudo apt-get update
  mapfile -t WANT < <(grep -v '^\s*#' "$PKG/apt-packages.txt" | sed '/^\s*$/d')
  HAVE=(); MISS=()
  for p in "${WANT[@]}"; do
    if apt-cache show "$p" >/dev/null 2>&1; then HAVE+=("$p"); else MISS+=("$p"); fi
  done
  echo "  共 ${#WANT[@]} 个包，源里可装 ${#HAVE[@]} 个，找不到 ${#MISS[@]} 个"
  [[ ${#MISS[@]} -eq 0 ]] || warn "apt 源里没有这些包（一般是 ROS 小版本差异，通常不影响）：${MISS[*]}"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${HAVE[@]}"
  DEB="$(ls "$PKG"/packages/linuxptp_*.deb | head -1)"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --allow-downgrades "$DEB"
  sudo apt-mark hold linuxptp >/dev/null
  ok "依赖安装完成；linuxptp 使用包内 3.1.1 版本并锁定（hold）"
else
  echo "  跳过（--skip-apt）"
fi

# ----------------------------------------------------------------------------- 2
step 2 "拷贝程序与数据到 $APP"
cp -a "$PKG/rootfs/home/pc/rk3588 link" "$HOME_DIR/"
mkdir -p "$HOME_DIR/lidar test/logs"                       # 雷达驱动互斥锁所在目录（脚本写死）
[[ -e "$HOME_DIR/.rviz2" ]] || cp -a "$PKG/rootfs/home/pc/.rviz2" "$HOME_DIR/" 2>/dev/null || true
if [[ -d "$PKG/rootfs/home/pc/selffilter_acceptance" && ! -e "$HOME_DIR/selffilter_acceptance" ]]; then
  cp -a "$PKG/rootfs/home/pc/selffilter_acceptance" "$HOME_DIR/"
fi
for f in "$PKG"/rootfs/home/pc/livox-*.json "$PKG/rootfs/home/pc/setup_enp85s0_ip.sh"; do
  [[ -e "$f" && ! -e "$HOME_DIR/$(basename "$f")" ]] && cp -a "$f" "$HOME_DIR/"
done
ok "程序、地图、先验、配置已就位（$(du -sh "$APP" | cut -f1)）"

# 网口改名：只改运行用到的文件（历史备份、文档不改）
IFACE_FILES=(
  "navigation/scripts/ptp_supervisor.py" "navigation/scripts/capture_ptp_runtime.sh"
  "navigation/scripts/raw_time_monitor.py" "navigation/diagnose_imu.sh"
  "navigation/programs/sensors/config/devices.json" "mapping/scripts/ptp_supervisor.py"
  "mapping/programs/sensors/config/devices.json" "xyz_control/config/interface_draft.json"
  "ptp_slave/install.sh" "ptp_slave/health_monitor.py" "ptp_slave/systemd/20-igc-ptm-workaround.conf"
)
STAGE="$(mktemp -d)"; trap 'rm -rf "$STAGE"' EXIT
cp -a "$PKG/rootfs/etc" "$PKG/rootfs/opt" "$PKG/rootfs/usr" "$STAGE/"
if [[ "$IFACE" != "$OLD_IFACE" ]]; then
  for f in "${IFACE_FILES[@]}"; do [[ -f "$APP/$f" ]] && sed -i "s/\b$OLD_IFACE\b/$IFACE/g" "$APP/$f"; done
  { grep -rlI "$OLD_IFACE" "$STAGE" || true; } | while read -r f; do sed -i "s/\b$OLD_IFACE\b/$IFACE/g" "$f"; done
  mv "$STAGE/etc/netplan/01-$OLD_IFACE-static.yaml" "$STAGE/etc/netplan/01-$IFACE-static.yaml" 2>/dev/null || true
  ok "已把 $OLD_IFACE 改成 $IFACE（程序 ${#IFACE_FILES[@]} 个文件 + 系统服务/脚本）"
fi

# ----------------------------------------------------------------------------- 3
step 3 "系统参数：DDS 缓冲 4 MiB、时区"
TZ_SRC="$(sed -n 's/^Timezone=//p' "$PKG/environment/time.txt" | head -1)"
if [[ -n "$TZ_SRC" && "$(timedatectl show -p Timezone --value)" != "$TZ_SRC" ]]; then
  sudo timedatectl set-timezone "$TZ_SRC" && ok "时区设为 $TZ_SRC（与原机一致，日志时间可直接对照）"
fi
sudo install -o root -g root -m 644 "$STAGE/etc/sysctl.d/90-poke-dds-buffers.conf" /etc/sysctl.d/
sudo sysctl --system >/dev/null
ok "$(sysctl -n net.core.rmem_max) / $(sysctl -n net.core.wmem_max)"

# ----------------------------------------------------------------------------- 4
step 4 "有线网络：$IFACE = $LAN_IP（雷达 .84/.85，RK .36）"
if [[ $DO_NET == 1 ]]; then
  if ip -4 -br addr show "$IFACE" | grep -q "${LAN_IP%/*}/"; then
    ok "$IFACE 已经是 $LAN_IP，不改网络"
  else
    sudo ip link set "$IFACE" up 2>/dev/null || true
    DUP=0
    if [[ "$(cat "/sys/class/net/$IFACE/carrier" 2>/dev/null || true)" != 1 ]]; then
      warn "$IFACE 未接线：先安装静态地址配置；接入交换机前确认没有其他 NUC 占用 ${LAN_IP%/*}"
    elif command -v arping >/dev/null; then sudo arping -D -q -c 2 -I "$IFACE" "${LAN_IP%/*}" || DUP=1
    else warn "没有 arping，跳过 IP 冲突检查（请自己确认旧 NUC 已断开）"; fi
    if [[ $DUP == 1 ]]; then
      warn "网段里已经有别的设备在用 ${LAN_IP%/*}（旧 NUC 还连着？）——网络未修改，请拔掉旧 NUC 后用 --skip-apt 重跑或手工设置"
    else
      NP="/etc/netplan/01-$IFACE-static.yaml"
      sudo tee "$NP" >/dev/null <<EOF
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    $IFACE:
      optional: true
      dhcp4: false
      dhcp6: false
      link-local: []
      addresses:
        - $LAN_IP
EOF
      sudo chmod 600 "$NP"
      sudo netplan generate && sudo netplan apply
      sleep 3; ip -br addr show "$IFACE"
      ok "netplan 已写入 $NP（无网关，不影响 WiFi 上网）"
    fi
  fi
else
  echo "  跳过（--skip-network）"
fi

# ----------------------------------------------------------------------------- 5
step 5 "PTP 时间同步（NUC 做从钟跟随 RK）、接收环、功耗监测"
if [[ $DO_PTP == 1 ]]; then
  sudo install -d -o root -g root -m 755 /etc/linuxptp /opt/nuc-ptp-slave /opt/poke-power-monitor /usr/local/sbin /usr/local/linuxptp-3.1.1
  sudo install -o root -g root -m 644 "$STAGE/etc/linuxptp/nuc-slave.conf" /etc/linuxptp/nuc-slave.conf
  sudo install -o root -g root -m 755 "$STAGE/usr/local/sbin/nuc-lidar-rx-ring" /usr/local/sbin/
  sudo install -o root -g root -m 644 "$STAGE/opt/nuc-ptp-slave/health_monitor.py" /opt/nuc-ptp-slave/
  sudo install -o root -g root -m 644 "$STAGE/opt/poke-power-monitor/power_monitor.py" /opt/poke-power-monitor/
  for b in "$STAGE"/usr/local/linuxptp-3.1.1/*; do sudo install -o root -g root -m 755 "$b" /usr/local/linuxptp-3.1.1/; done
  for b in ptp4l phc2sys pmc ts2phc; do sudo ln -sfn "/usr/local/linuxptp-3.1.1/$b" "/usr/local/bin/$b"; done
  if [[ $CHASSIS == 1 ]]; then
    # 纯底盘版：没有 RK 主钟。NUC 用 ptp_supervisor 做软件 PTP 主钟（nuc-ptp-master），双雷达跟随 NUC。
    # 主钟脚本在 /opt/livox-navigation-home-runtime（第 6 步安装），所以这里只装服务文件，第 6 步后再启动。
    for u in nuc-ptp-slave nuc-phc2sys nuc-ptp-health; do
      systemctl list-unit-files "$u.service" >/dev/null 2>&1 && sudo systemctl disable --now "$u" 2>/dev/null || true
    done
    cp "$PKG/ptp_master/nuc-ptp-master.service" "$STAGE/etc/systemd/system/nuc-ptp-master.service"
    UNITS=(nuc-lidar-rx-ring poke-power-monitor nuc-ptp-master)
  else
    UNITS=(nuc-lidar-rx-ring nuc-ptp-slave nuc-phc2sys nuc-ptp-health poke-power-monitor)
  fi
  for u in "${UNITS[@]}"; do
    f="$STAGE/etc/systemd/system/$u.service"
    sed -i "s/sys-subsystem-net-devices-$OLD_IFACE/sys-subsystem-net-devices-$IFACE/g" "$f"
    sudo install -o root -g root -m 644 "$f" /etc/systemd/system/
  done
  # 只能有一个程序调系统时钟：关掉 NTP（完整版由 phc2sys 跟随 RK；纯底盘版主钟取系统时间，避免运行中被 NTP 跳变）
  sudo timedatectl set-ntp false
  for u in chrony ntp ntpsec; do systemctl is-active --quiet "$u" 2>/dev/null && sudo systemctl disable --now "$u"; done
  sudo systemctl daemon-reload
  sudo systemctl enable "${UNITS[@]/%/.service}" >/dev/null
  for u in "${UNITS[@]}"; do
    [[ $u == nuc-ptp-master ]] && continue
    sudo systemctl restart "$u" || warn "$u 启动失败（网口没接 RK 时属正常，接好后 sudo systemctl restart $u）"
  done
  sleep 2
  for u in "${UNITS[@]}"; do printf '    %-22s %s\n' "$u" "$(systemctl is-active "$u" || true)"; done
  ok "PTP / 接收环 / 功耗监测已安装并设为开机启动$([[ $CHASSIS == 1 ]] && echo '（纯底盘版：NUC 主钟第 6 步后启动）')"
else
  echo "  跳过（--skip-ptp）"
fi

# ----------------------------------------------------------------------------- 6
step 6 "root 固定入口与免密规则（CAN can0 配置、PTP/时间监测、导航停止）"
sudo /usr/bin/python3 "$APP/web_console/install_privileges.py"
sudo visudo -c >/dev/null && ok "sudoers 语法检查通过"
for d in livox-navigation-home-runtime rk3588-link-web-runtime; do
  if [[ $CHASSIS == 1 && $d == livox-navigation-home-runtime ]]; then
    ok "/opt/$d 已按纯底盘补丁后的源码重装（ptp_health/timing_policy 与原机快照不同属正常）"; continue
  fi
  if diff -rq -x backups -x logs -x __pycache__ "$STAGE/opt/$d" "/opt/$d" >/dev/null; then ok "/opt/$d 与原机一致"
  else warn "/opt/$d 与原机快照不同：$(diff -rq -x backups -x logs -x __pycache__ "$STAGE/opt/$d" "/opt/$d" | head -3 | tr '\n' ' ')"; fi
done
for f in nuc-lidar-rx-ring livox-navigation-home-runtime rk3588-link-can; do
  cmp -s "$STAGE/usr/local/sbin/$f" "/usr/local/sbin/$f" && ok "/usr/local/sbin/$f 与原机一致" || warn "/usr/local/sbin/$f 与原机不同"
done
# 2026-11-20: 运动服务以 pc 身份读手柄 /dev/input/js0（root:input 0660），pc 必须在 input 组，
# 否则没人登录桌面时读不到（PermissionError）。重启后生效。
if id -nG pc | tr ' ' '\n' | grep -qx input; then ok "pc 已在 input 组（手柄可读）"
else sudo usermod -aG input pc && ok "已把 pc 加入 input 组（手柄读取用，重启后生效）"; fi
if [[ $CHASSIS == 1 && $DO_PTP == 1 ]]; then
  sudo systemctl restart nuc-ptp-master || warn "nuc-ptp-master 启动失败：journalctl -u nuc-ptp-master -n 50"
  sleep 12
  H="$(python3 -c 'import json;d=json.load(open("/run/livox-ptp/health.json"));print(d.get("mode"),d.get("port_state"),d.get("healthy"),d.get("reason"))' 2>/dev/null || true)"
  [[ "$H" == "software MASTER True ok" ]] && ok "NUC PTP 主钟健康（$H）" || warn "NUC PTP 主钟未就绪：${H:-无健康文件}（网口未接时属正常；接好后看 /run/livox-ptp/health.json）"
fi

# ----------------------------------------------------------------------------- 7
step 7 "用户级服务（开机自启入口）、桌面快捷方式、bashrc"
mkdir -p "$HOME_DIR/.config/systemd/user"
for u in poke-nuc-autostart poke-nuc-web-autostart poke-cloud-latency; do
  cp "$PKG/rootfs/home/pc/.config/systemd/user/$u.service" "$HOME_DIR/.config/systemd/user/"
done
systemctl --user daemon-reload 2>/dev/null || warn "当前没有用户 systemd 会话，重新登录后执行 systemctl --user daemon-reload"
if [[ $AUTOSTART == 1 ]]; then
  systemctl --user enable poke-nuc-autostart.service poke-nuc-web-autostart.service poke-cloud-latency.service
  sudo loginctl enable-linger pc
  ok "已开启开机自启（重启后自动启动 定位/Nav2/网关 和 8870 网页；运动服务仍需在 RK 5010 上手动 Start）"
else
  ok "服务文件已放好但未开启开机自启。现场验收通过后执行："
  echo "      systemctl --user enable poke-nuc-autostart poke-nuc-web-autostart poke-cloud-latency && sudo loginctl enable-linger pc"
fi
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME_DIR/桌面")"
if [[ -d "$DESK" ]]; then cp "$APP/机器人控制台.desktop" "$DESK/" && chmod +x "$DESK/机器人控制台.desktop"; fi
grep -q 'source /opt/ros/jazzy/setup.bash' "$HOME_DIR/.bashrc" || printf '\n# ROS 2 Jazzy\nsource /opt/ros/jazzy/setup.bash\n' >> "$HOME_DIR/.bashrc"
ok "完成"

# ----------------------------------------------------------------------------- 8
step 8 "部署后自检"
set +e; trap - ERR
# shellcheck disable=SC1091
set +u  # ROS/ament setup scripts legitimately read optional unset variables.
source /opt/ros/jazzy/setup.bash
set -u
# 工程自带的库目录（colcon install/*/lib、Livox SDK 的 sdk-install/lib 等；启动脚本运行时会自己设置）
LIBS="$(find "$APP" -type f -name '*.so*' -not -path '*/logs/*' -not -path '*/build/*' -printf '%h\n' | sort -u | tr '\n' ':')"
MISSING="$(find "$APP" -type f \( -name '*.so*' -o -perm -u+x \) -not -path '*/logs/*' -not -path '*/build/*' -size +4k -print0 \
  | xargs -0 file -N 2>/dev/null | grep -E 'ELF .*(executable|shared object)' | cut -d: -f1 \
  | while read -r f; do LD_LIBRARY_PATH="$LIBS${LD_LIBRARY_PATH:-}" ldd "$f" 2>/dev/null | grep 'not found' | sed "s|^|$f: |"; done | sort -u)"
if [[ -z "$MISSING" ]]; then ok "所有编译程序的动态库都能找到"
else warn "有动态库缺失（需要补装对应 apt 包或重新编译）："; echo "$MISSING" | head -30; fi
python3 - <<'PY'
import importlib
mods = ['rclpy', 'numpy', 'scipy', 'yaml', 'psutil', 'google.protobuf', 'PIL', 'matplotlib', 'can', 'serial',
        'tf2_ros', 'sensor_msgs_py', 'nav2_msgs', 'rosbag2_py', 'launch_ros', 'lifecycle_msgs']
bad = []
for m in mods:
    try: importlib.import_module(m)
    except Exception as e: bad.append(f'{m} ({e.__class__.__name__}: {e})')
print('  [OK] Python 依赖齐全' if not bad else '  [WARN] Python 模块缺失：' + '; '.join(bad))
PY
ros2 pkg prefix nav2_bringup >/dev/null 2>&1 && ok "Nav2 已安装" || warn "找不到 nav2_bringup"
[[ -x "$APP/RK3588con/program/build/remote_bridge_r2" ]] && ok "底盘桥 remote_bridge_r2 在位" || warn "缺少 RK3588con/program/build/remote_bridge_r2"
ip link show can0 >/dev/null 2>&1 && ok "CAN 口 can0 存在（PEAK USB-CAN 插上即有）" || warn "暂时没有 can0：PEAK XCAN-USB 适配器插上后才会出现"
ip -br addr show "$IFACE"
set -e

echo
echo "==================== 部署结束"
if [[ ${#WARN[@]} -gt 0 ]]; then echo "需要注意的事项（${#WARN[@]} 条）："; for w in "${WARN[@]}"; do echo "  - $w"; done; fi
if [[ $CHASSIS == 1 ]]; then cat <<EOF

【纯底盘版】下一步（详见 read.md 第 5 节）：
  1. NUC 网线接底盘交换机；ping 192.168.16.84 / .85（两颗雷达）。没有 RK，不需要 .36
  2. systemctl status nuc-ptp-master；cat /run/livox-ptp/health.json（mode=software、port_state=MASTER、healthy=true）
  3. cd "$APP" && ./nuc_web.sh start && ./nuc.sh start && ./nuc.sh status
  4. 浏览器打开 NUC 网页 http://<NUC地址>:8870：定位 ready 后「③ 启动运动服务」→「④ 连接协议终端」→ 发目标/速度；
     或「手柄」按钮（独立遥控，纯底盘模式）。有人值守低速验收
日志：$LOG
EOF
else cat <<EOF

下一步（详见 read.md 第 5 节）：
  1. 确认旧 NUC 已断开，新 NUC 网线接机器人交换机；ping 192.168.16.36（RK）、.84/.85（雷达）
  2. cd "$APP" && ./nuc_web.sh start && ./nuc.sh start && ./nuc.sh status
  3. 浏览器打开 RK 网页 http://192.168.16.36:5010，用「Start NUC service」启动运动服务，有人值守低速验收
日志：$LOG
EOF
fi
