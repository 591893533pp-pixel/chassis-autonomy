"""Navigation-only timing trial. No ROS/hardware writes; hard faults latch."""
import math
import time
import json
from pathlib import Path

IPS = ('192.168.16.84', '192.168.16.85')
POLICY = 'raw_udp_bounded_v1'
RAW_PATH = Path('/run/livox-nav-time/status.json')
# Only loss of synchronization updates can recover automatically. Clock steps,
# offsets, packet loss, identities and process restarts are deliberately excluded.
PTP_TRANSIENT = frozenset(('slave_unhealthy:nuc-ptp-slave.service:stale_servo',
    'slave_unhealthy:not_slave', 'slave_warming_up', 'stale_servo', 'not_slave'))
def recoverable_ptp_fault(reason):
    if str(reason).startswith('timing_trial_blocked:'):
        reason=str(reason)[len('timing_trial_blocked:'):]
    for prefix in ('raw_monitor_fault:ptp_clock:', 'ptp_master:', 'PTP未就绪：', 'ptp_clock:'):
        if str(reason).startswith(prefix):
            return str(reason)[len(prefix):] in PTP_TRANSIENT
    return False

OFFSET_RECOVERABLE = frozenset(('self_offset_grace_exhausted',
    'repeated_sync_anomaly_before_recovery'))

def recoverable_control_timing_fault(reason):
    # Deliberately separate from raw/PTP recovery: measured raw offset/clock faults
    # must NEVER become recoverable because device self-report jitter is recoverable.
    prefix = 'timing_trial_blocked:'
    return (recoverable_ptp_fault(reason) or
            (str(reason).startswith(prefix) and str(reason)[len(prefix):] in OFFSET_RECOVERABLE))

SYNC_HISTORY = 67436546  # 0x04050002 only; other HMS codes remain blocking.
# Codes seen with the self-reported +-500ms sequence (500ms data, hms_transient_DQy5mBYl):
# 67239938 ~1s before -500ms, 67502082 during -/+500ms. Raw UDP must stay healthy.
SYNC_EVENT_CODES = frozenset((67239938, 67502082))
# These codes are the Livox MID-360S self-report sequence observed during the
# false 500 ms event.  The host-side raw monitor remains the authority.

def finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)

def validate_raw(d, now, wall, boot):
    if d.get('schema') != POLICY or d.get('boot_id') != boot:
        return 'raw_monitor_identity'
    if not isinstance(d.get('generation'), str) or not d['generation']:
        return 'raw_monitor_generation_missing'
    for key, current in [('monotonic', now), ('utc', wall)]:
        v = d.get(key)
        if not finite(v) or not 0 <= current-v < .15:
            return 'raw_monitor_stale'
    if d.get('fault') or d.get('ready') is not True:
        if not d.get('fault') and d.get('phase') == 'warming':
            return 'raw_monitor_warming_up'
        return 'raw_monitor_fault:' + str(d.get('fault') or 'waiting_four_streams')
    for ip in IPS:
        for port in (56300, 56400):
            v = d.get('streams', {}).get(f'{ip}:{port}', {})
            seen = v.get('seen')
            if not finite(seen) or not 0 <= now-seen < .05:
                return 'raw_stream_stale:' + ip + ':' + str(port)
            age = v.get('age_ns')
            if not finite(age) or abs(age) > 5_000_000:
                return 'raw_clock_age:' + ip
    return None

def read_raw():
    try:
        d = json.loads(RAW_PATH.read_text())
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        return d, validate_raw(d, time.monotonic(), time.time(), boot)
    except (OSError, ValueError, TypeError, KeyError):
        return {}, 'raw_monitor_unreadable'

class TimingPolicy:
    """Require 10s normal startup and fail closed on host-side timing faults.

    Known Livox sync-event HMS codes may remain degraded indefinitely only while
    the raw host receive monitor, PTP master, and packet timestamps are healthy.
    An uncorrelated self-reported offset still gets the bounded <=3s window.
    HMS alone never acquires permission at startup. Hard faults need restart.
    """
    def __init__(self, ptp_only=True):
        self.ptp_only = ptp_only
        self.normal_since = None
        self.armed = False
        self.active = None
        self.last_active = None
        self.events = []
        self.history_allowed = set()
        self.fault = None
        self.raw_generation = None
        self.ptp_generation = None
        self.was_ready = False
        self.ptp_recovery_since = None
        self.ptp_recoveries = []
        self.offset_recoveries = []
        self.offset_recovery_started = None

    def evaluate(self, now, status, seen, raw, raw_error, master_ok, master_generation,
                 master_failure_reason='master_unhealthy'):
        ignored_gps = set()
        def result(state, reason):
            return dict(policy=POLICY, state=state, reason=reason,
                        allow=state in ('normal', 'degraded'), monotonic=now,
                        raw_generation=self.raw_generation,
                        ptp_generation=self.ptp_generation,
                        degraded_seconds=0 if self.active is None else now-self.active,
                        historical_sync_hms_ips=sorted(self.history_allowed),
                        ignored_gps_hms_ips=sorted(ignored_gps) if state in ('normal', 'warming') else [])
        def fail(reason):
            self.normal_since = None
            if self.was_ready:
                self.fault = reason
                if reason in OFFSET_RECOVERABLE and self.offset_recovery_started is None:
                    self.offset_recovery_started = now
            return result('blocked', reason)
        recovering_hms = self.fault in {ip+':other_hms:[67567618]' for ip in IPS}
        recovering_ptp = recoverable_ptp_fault(self.fault)
        recovering_offset = self.fault in OFFSET_RECOVERABLE
        recovering = recovering_offset or recovering_hms or recovering_ptp
        if self.fault and not recovering: return result('blocked', self.fault)
        if recovering_ptp:
            self.ptp_recoveries = [t for t in self.ptp_recoveries if now-t < 600]
            if len(self.ptp_recoveries) >= 2:
                return fail('ptp_recovery_budget_exhausted')
        if recovering_offset:
            self.offset_recoveries = [t for t in self.offset_recoveries if now-t < 600]
            if self.offset_recovery_started is None: self.offset_recovery_started = now
            if len(self.offset_recoveries) >= 2 or now-self.offset_recovery_started > 120:
                return fail('self_offset_recovery_budget_or_timeout')
        if not master_ok: return fail(master_failure_reason)
        if self.ptp_generation not in (None, master_generation): return fail('master_restarted')
        self.ptp_generation = master_generation
        if raw_error: return fail(raw_error)
        if self.raw_generation not in (None, raw.get('generation')): return fail('monitor_restarted')
        self.raw_generation = raw.get('generation')
        abnormal = set()
        event_code_ips = set()
        for ip in IPS:
            d = status.get(ip, {})
            if not 0 <= now-seen.get(ip, -1e30) < 1.5: return fail(ip+':diagnostics_stale')
            if d.get('time_sync_type') != 1: return fail(ip+':not_ptp')
            local, last = d.get('local_time_now'), d.get('last_sync_time')
            if not all(isinstance(x, int) and not isinstance(x, bool) for x in (local, last)) or not 0 <= local-last < 3_000_000_000:
                return fail(ip+':last_sync_stale')
            offset = d.get('time_offset')
            if not isinstance(offset, int) or isinstance(offset, bool): return fail(ip+':offset_missing')
            codes = d.get('hms_code')
            if not isinstance(codes, list) or any(not isinstance(x, int) or isinstance(x, bool) for x in codes): return fail(ip+':hms_missing')
            codes = set(codes)-{0}
            # User explicitly uses PTP only. MID360S official Viewer defines
            # 0x04070002 as PPS/GPS loss, not a PTP clock fault.
            if self.ptp_only and codes == {67567618}:
                ignored_gps.add(ip)
                codes = set()
            if codes-{SYNC_HISTORY}-SYNC_EVENT_CODES: return fail(ip+':other_hms:'+str(sorted(codes)))
            event_known = bool(codes & SYNC_EVENT_CODES)
            if event_known:
                if not self.armed: return fail(ip+':hms_at_startup')
                event_code_ips.add(ip)
                # The event leaves 67436546 latched afterwards; treat it as history.
                self.history_allowed.add(ip)
            elif codes == {SYNC_HISTORY}:
                # 0x04050002 is a latched history indication.  It does not by
                # itself indicate a current clock step; keep strict raw/PTP
                # checks, but do not make startup wait for the radar to erase it.
                self.history_allowed.add(ip)
            bad = abs(offset) > 500_000
            # During this known sequence the device can report +/-500 ms while
            # host receive timestamps, packet timestamps, and raw monitor are
            # healthy.  Keep fusion alive; raw faults still fail above.
            if bad and not event_known: abnormal.add(ip)
            if codes and ip not in self.history_allowed:
                if not self.armed: return fail(ip+':hms_at_startup')
                # A newly seen sync code also consumes the bounded grace window.
                abnormal.add(ip)
        if ignored_gps and (abnormal or event_code_ips):
            return fail('gps_hms_with_ptp_offset_or_sync_event')
        if recovering:
            # Self-offset timeout and cleared 0x04070002 may recover. Host/raw/diagnostic faults
            # above still latch; both devices must be strictly healthy for 10s.
            strict = not event_code_ips and all(
                abs(status[ip]['time_offset']) <= 500_000 for ip in IPS)
            if recovering_hms:
                strict = strict and all(not (set(status[ip]['hms_code'])-{0}) for ip in IPS)
            if not strict:
                self.normal_since = None
                return result('blocked', self.fault)
            if self.normal_since is None:
                self.normal_since = now
            if now-self.normal_since < 10:
                if recovering_ptp: return result('blocked', 'ptp_recovering_10s')
                return result('blocked', 'hms_cleared_recovering_10s' if recovering_hms else 'self_offset_recovering_10s')
            if recovering_ptp: self.ptp_recoveries.append(now)
            if recovering_offset:
                self.offset_recoveries.append(now)
                self.offset_recovery_started = None
            self.fault = None
            self.active = None
            self.last_active = None
            # Keep the rolling anomaly budget; recovery does not erase history.
            return result('normal', 'ok')
        self.events = [(a,b) for a,b in self.events if b > now-60]
        if abnormal:
            if not self.armed: return fail('self_offset_before_stable_startup')
            if self.active is None:
                if self.last_active is not None and (self.normal_since is None or now-self.normal_since < 10):
                    return fail('repeated_sync_anomaly_before_recovery')
                self.active = now
            self.normal_since = None
            used = sum(b-max(a, now-60) for a,b in self.events) + now-self.active
            if now-self.active >= 3 or used >= 6: return fail('self_offset_grace_exhausted')
            self.history_allowed.update(abnormal)
            return result('degraded', 'self_report_only')
        if self.active is not None:
            self.events.append((self.active, now)); self.last_active = now; self.active = None
        if event_code_ips:
            # Code-only phase: not counted in the 3s/6s offset budget, bounded per code.
            return result('degraded', 'sync_event_hms_only')
        if self.normal_since is None: self.normal_since = now
        if not self.armed:
            if now-self.normal_since < 10: return result('warming', 'need_10s_strict_health')
            self.armed = True
        self.was_ready = True
        return result('normal', 'ok')
