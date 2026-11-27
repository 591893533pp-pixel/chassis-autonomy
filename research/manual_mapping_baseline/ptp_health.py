"""Fail-closed validation of the local PTP supervisor heartbeat."""
import json, math, os, pathlib, stat, time
HEALTH_PATH=pathlib.Path('/run/livox-ptp/health.json')
BOOT_ID=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()

def validate_health(data, now, boot_id=BOOT_ID):
    if not isinstance(data,dict): return False,'invalid_health'
    if data.get('boot_id')!=boot_id: return False,'different_boot'
    updated=data.get('monotonic')
    if not isinstance(updated,(int,float)) or not math.isfinite(updated) or not 0<=now-updated<2.0:
        return False,'stale_heartbeat'
    if not data.get('healthy',False): return False,data.get('reason','master_unhealthy')
    if data.get('mode')=='hardware_slave':
        if data.get('port_state')!='SLAVE': return False,'not_slave'
        if data.get('grandmaster_identity')!='8c3223.fffe.872e6c': return False,'unexpected_grandmaster'
        for prefix in ('', 'system_'):
            if data.get(prefix+'servo_state') not in (2,3): return False,prefix+'servo_unlocked'
            age=data.get(prefix+'servo_age_s')
            if not isinstance(age,(int,float)) or not 0<=age<2.5: return False,prefix+'stale_servo'
            offset=data.get(prefix+'offset_ns')
            if not isinstance(offset,(int,float)) or not math.isfinite(offset) or abs(offset)>100_000: return False,prefix+'offset_out_of_bounds'
        age=data.get('port_age_s')
        if not isinstance(age,(int,float)) or not 0<=age<2: return False,'stale_port_query'
        return True,'ok'
    if data.get('port_state')!='MASTER': return False,'not_master'
    if data.get('mode')=='software':
        if data.get('ptp_timescale')!=0 or data.get('utc_offset_applied_seconds')!=0:
            return False,'unexpected_software_timescale'
        age=data.get('port_age_s')
        if not isinstance(age,(int,float)) or not 0<=age<2:
            return False,'stale_port_query'
        return True,'ok'
    if data.get('servo_state') not in (2,3): return False,'servo_unlocked'
    if not isinstance(data.get('servo_age_s'),(int,float)) or not 0<=data['servo_age_s']<2.5:
        return False,'stale_servo'
    if not isinstance(data.get('offset_ns'),(int,float)) or not math.isfinite(data['offset_ns']) or abs(data['offset_ns'])>100_000:
        return False,'offset_out_of_bounds'
    return True,'ok'

def read_health(path=HEALTH_PATH):
    try:
        with path.open() as f:
            info=os.fstat(f.fileno())
            if info.st_uid!=0 or info.st_mode&(stat.S_IWGRP|stat.S_IWOTH):
                return False,'untrusted_health_file',{}
            data=json.load(f)
        ok,reason=validate_health(data,time.monotonic())
        return ok,reason,data
    except (OSError,ValueError,TypeError): return False,'no_valid_master_heartbeat',{}


# 2026-11-23 chassis-only variant: no RK3588 master clock; the NUC itself is the (software) PTP master.
CHASSIS_ONLY_CONFIG=pathlib.Path('/home/pc/rk3588 link/config/chassis_only.json')

def chassis_only(path=CHASSIS_ONLY_CONFIG):
    try: return json.loads(pathlib.Path(path).read_text(encoding='utf-8')).get('enabled') is True
    except (OSError, ValueError, AttributeError): return False

def expected_mode(path=CHASSIS_ONLY_CONFIG):
    """Clock mode the robot variant runs with: RK hardware slave, or the NUC software master."""
    return 'software' if chassis_only(path) else 'hardware_slave'

def lidar_utc_offset():
    """Livox PTP packet time to ROS CLOCK_REALTIME, from trusted master properties."""
    ok,reason,h=read_health()
    mode=expected_mode()
    if not ok or h.get('mode')!=mode:raise RuntimeError(('NUC主钟' if mode=='software' else '从钟')+'未就绪：'+reason)
    if mode=='software':return 0              # ARB timescale from CLOCK_REALTIME: lidar time = system time
    if h.get('ptp_timescale')==0:return 0
    offset=h.get('current_utc_offset')
    if h.get('ptp_timescale')!=1 or not isinstance(offset,int) or not 0<=offset<=100:raise RuntimeError('PTP UTC offset无效')
    return offset
