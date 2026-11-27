#!/usr/bin/env bash
# Read-only, bounded capture on the chassis Ethernet interface. No service/clock/CAN changes.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo '抓取网卡报文需要 sudo；本脚本不改服务、时间或CAN。'; exit 1; }
DURATION=${1:-90}
DATA_HEADERS=${2:-}
[[ -z "$DATA_HEADERS" || "$DATA_HEADERS" == --with-data-headers ]] || { echo "第二参数仅支持 --with-data-headers"; exit 1; }
[[ "$DURATION" =~ ^[0-9]+$ ]] && (( DURATION >= 30 && DURATION <= 600 )) || { echo '时长需30–600秒'; exit 1; }
OUT=$(mktemp -d /var/tmp/chassis-ptp-trace-XXXXXXXX)
chown pc:pc "$OUT"
chmod 750 "$OUT"
echo "输出目录：$OUT"
echo "开始 $DURATION 秒只读抓包，请保持NUC、交换机和雷达通电，期间不要手动校时。"
START=$(date --iso-8601=seconds)
ip -details -statistics address show enp85s0 > "$OUT/interface_before.txt"
ethtool -T enp85s0 > "$OUT/timestamp_capabilities.txt"
ethtool -S enp85s0 > "$OUT/nic_counters_before.txt" 2>&1 || true
systemctl cat nuc-ptp-master.service > "$OUT/service.txt"
cp /opt/livox-navigation-home-runtime/config/ptp4l.conf "$OUT/ptp4l.conf"
pgrep -a ptp4l > "$OUT/ptp_processes.txt" || true
pgrep -a phc2sys > "$OUT/phc_processes.txt" || true
/usr/sbin/pmc -u -b 0 -s /var/run/ptp4l 'GET DEFAULT_DATA_SET' 'GET CURRENT_DATA_SET' 'GET PARENT_DATA_SET' 'GET TIME_PROPERTIES_DATA_SET' 'GET PORT_DATA_SET' > "$OUT/pmc_before.txt" 2>&1 || true
CAPTURE_PID=''
MONITOR_PID=''
DATA_PID=''
cleanup() {
 if [[ -n "$CAPTURE_PID" ]]; then kill -INT "$CAPTURE_PID" 2>/dev/null || true; fi
 if [[ -n "$DATA_PID" ]]; then kill -INT "$DATA_PID" 2>/dev/null || true; fi
 if [[ -n "$MONITOR_PID" ]]; then kill -TERM "$MONITOR_PID" 2>/dev/null || true; fi
}
trap cleanup INT TERM HUP EXIT
python3 - "$OUT" "$DURATION" <<'PY' &
import pathlib,sys,time,json
out=pathlib.Path(sys.argv[1]);duration=int(sys.argv[2]);start=time.monotonic()
with (out/'host_clock_health.jsonl').open('w') as f:
 while time.monotonic()-start<duration:
  before=time.monotonic_ns();wall=time.time_ns();after=time.monotonic_ns()
  row={'monotonic_ns':(before+after)//2,'realtime_ns':wall,'read_span_ns':after-before}
  try:row['health']=json.loads(pathlib.Path('/run/livox-ptp/health.json').read_text())
  except Exception as e:row['health_error']=str(e)
  f.write(json.dumps(row)+'\n');f.flush();time.sleep(.1)
PY
MONITOR_PID=$!
set +e
if [[ "$DATA_HEADERS" == --with-data-headers ]]; then
 echo '同时保存点云/IMU包头（每包128字节）；不保存完整点云。'
 timeout --signal=INT --kill-after=5s "${DURATION}s" /usr/bin/tcpdump -i enp85s0 -nn -s 128 -B 8192 -U --time-stamp-precision=nano -Z pc -w "$OUT/sensor_headers.pcap" 'udp and (src host 192.168.1.132 or src host 192.168.1.159) and (src port 56300 or src port 56400)' > "$OUT/sensor_tcpdump.log" 2>&1 &
 DATA_PID=$!
fi
timeout --signal=INT --kill-after=5s "${DURATION}s" /usr/bin/tcpdump -i enp85s0 -nn -s 2048 -B 4096 -U --time-stamp-precision=nano -Z pc -w "$OUT/ptp_control.pcap" 'ether proto 0x88f7 or (udp and (port 319 or port 320 or ((host 192.168.1.132 or host 192.168.1.159) and (port 56100 or port 56101 or port 56200 or port 56201))))' > "$OUT/tcpdump.log" 2>&1 &
CAPTURE_PID=$!
wait "$CAPTURE_PID"
CODE=$?
CAPTURE_PID=''
DATA_CODE=0
if [[ -n "$DATA_PID" ]]; then wait "$DATA_PID"; DATA_CODE=$?; DATA_PID=''; fi
wait "$MONITOR_PID"
MONITOR_PID=''
set -e
journalctl -b -u nuc-ptp-master.service --since "$START" --no-pager > "$OUT/ptp_journal.txt"
ip -details -statistics link show enp85s0 > "$OUT/interface_after.txt"
ethtool -S enp85s0 > "$OUT/nic_counters_after.txt" 2>&1 || true
chown -R pc:pc "$OUT"
trap - INT TERM HUP EXIT
if [[ $CODE != 0 && $CODE != 124 ]] || [[ $DATA_CODE != 0 && $DATA_CODE != 124 ]]; then
 echo "抓包失败（控制报文退出码$CODE，数据包头退出码$DATA_CODE），日志：$OUT/tcpdump.log"
 exit 1
fi
echo "抓包完成：$OUT"
echo '请把此目录路径发回；不需要粘贴完整报文。'
