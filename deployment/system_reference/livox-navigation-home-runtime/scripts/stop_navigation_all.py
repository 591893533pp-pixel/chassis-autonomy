#!/usr/bin/env python3
"""Stop identified navigation processes and descendants, preserving data."""
import json, os, signal, sys
from pathlib import Path
import psutil
ROOT = '/home/pc/rk3588 link/navigation'
try:
    chassis_only = json.loads((Path(ROOT).parent/'config/chassis_only.json').read_text()).get('enabled') is True
except (OSError, ValueError, AttributeError):
    chassis_only = False
dry = '--dry-run' in sys.argv
user_only = '--allow-user' in sys.argv
keep_sensors = '--keep-sensors' in sys.argv
if not dry and not user_only and os.geteuid() != 0:
    sys.exit('请使用 stop.sh，通过 sudo 停止 root 所属的 PTP。')
excluded = {os.getpid()} | {p.pid for p in psutil.Process().parents()}
def matches(c):
    if not c or Path(c[0]).name in ('bash','sh','sudo','tee','timeout'): return False
    exe = Path(c[0]).name
    # The chassis-only clock belongs to systemd, not a navigation/mapping session.
    # Preserve its supervisor and ptp4l while still stopping the raw UDP monitor.
    if chassis_only and (
            '/opt/livox-navigation-home-runtime/scripts/ptp_supervisor.py' in c or
            (exe == 'ptp4l' and '/opt/livox-navigation-home-runtime/config/ptp4l.conf' in c)):
        return False
    if keep_sensors and any(x in c for x in (
            ROOT+'/scripts/fusion_timing_trial.py', ROOT+'/scripts/sensors_headless.launch.py',
            ROOT+'/scripts/raw_time_monitor.py', ROOT+'/scripts/ptp_supervisor.py',
            '/opt/livox-navigation-home-runtime/scripts/raw_time_monitor.py',
            '/opt/livox-navigation-home-runtime/scripts/ptp_supervisor.py')): return False
    if keep_sensors and exe == 'ptp4l': return False
    if any('/opt/livox-navigation-home-runtime/scripts/'+s in c for s in ('raw_time_monitor.py','ptp_supervisor.py')): return True
    if exe == 'ptp4l' and '/opt/livox-navigation-home-runtime/config/ptp4l.conf' in c: return True
    if any(ROOT+'/scripts/'+s in c for s in ('cloud_adapter.py','sensor_bridge.py','localization_watch.py','automatic_global_localization.py','cmd_vel_bridge.py','raw_time_monitor.py','fusion_timing_trial.py','rk_body_tf_relay.py','live_map_updater.py')): return True
    if '/home/pc/rk3588 link/navigation/scripts/ptp_supervisor.py' in c: return True
    if exe == 'ptp4l' and any(x in c for x in ('/home/pc/rk3588 link/navigation/programs/sensors/config/ptp4l.conf', ROOT+'/config/ptp4l_delayreq_1s_trial.conf')): return True
    if exe == 'ros2' or '/opt/ros/jazzy/bin/ros2' in c:
        if any(x in c for x in (ROOT+'/scripts/sensors_headless.launch.py',ROOT+'/scripts/localization_guard.launch.py','/home/pc/rk3588 link/navigation/programs/sensors/fusion.launch.py')): return True
        if any(ROOT+'/config/' in x for x in c): return True
    if exe in ('rviz2','fastlio_mapping') and any(ROOT+'/config/' in x for x in c): return True
    if exe == 'rviz2' and '/home/pc/rk3588 link/navigation/programs/sensors/config/fusion.rviz' in c: return True
    if '/home/pc/rk3588 link/navigation/programs/sensors/fusion_node.py' in c: return True
    return False
targets = {}
for p in psutil.process_iter(['cmdline']):
    try:
        if p.pid not in excluded and matches(p.info['cmdline']):
            for q in [p]+p.children(recursive=True):
                if q.pid not in excluded: targets[q.pid] = q
    except (psutil.NoSuchProcess, psutil.AccessDenied): pass
for p in targets.values():
    try: print(p.pid, ' '.join(p.cmdline()), flush=True)
    except psutil.NoSuchProcess: pass
if dry:
    print('仅检查，匹配进程数：',len(targets)); sys.exit(0)
for p in targets.values():
    try: p.send_signal(signal.SIGINT)
    except psutil.NoSuchProcess: pass
_, alive = psutil.wait_procs(list(targets.values()),timeout=12)
for p in alive:
    try: p.terminate()
    except psutil.NoSuchProcess: pass
_, alive = psutil.wait_procs(alive,timeout=5)
if alive:
    print('仍未退出，停止重启流程：',[p.pid for p in alive]); sys.exit(1)
print('已确认本导航链路匹配的进程退出；日志和地图保留。')
