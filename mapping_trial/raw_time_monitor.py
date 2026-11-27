#!/usr/bin/env python3
"""Passive AF_PACKET receiver with kernel SO_TIMESTAMPNS; no transmitted packets.

Every point/IMU UDP header is checked, not a sampled ROS callback. Any invalid
clock, packet loss, reordering, stale stream, socket drop or clock step
latches a fault after startup qualification. Before qualification, faults restart
the continuous 10-second warmup; no data is authorized. Snapshots are root-owned.
"""
import json, socket, struct, time, uuid, os, signal, fcntl
from pathlib import Path
from timing_policy import POLICY, IPS, RAW_PATH, recoverable_ptp_fault, PTP_TRANSIENT

MACS = {'192.168.1.132':'0c:9a:e6:62:6b:ef','192.168.1.159':'34:91:f0:e6:fe:29'}

def decode(frame):
    if len(frame) < 42: return None
    off=14; proto=int.from_bytes(frame[12:14],'big')
    if proto in (0x8100,0x88a8):
        if len(frame)<46:return None
        proto=int.from_bytes(frame[16:18],'big');off=18
    if proto != 0x0800 or frame[off]>>4 != 4 or frame[off+9]!=17:return None
    ihl=(frame[off]&15)*4
    if ihl<20 or len(frame)<off+ihl+8:return None
    src=socket.inet_ntoa(frame[off+12:off+16]); dst=socket.inet_ntoa(frame[off+16:off+20])
    sp,dp,ln=struct.unpack_from('!HHH',frame,off+ihl)
    if src not in IPS or sp not in (56300,56400):return None
    if dst!='192.168.1.4' or dp!=sp+1:return None
    if int.from_bytes(frame[off+6:off+8],'big')&0x3fff:raise ValueError('fragmented_sensor_packet')
    if frame[6:12].hex(':')!=MACS[src]:raise ValueError('sensor_MAC_mismatch:'+src)
    b=frame[off+ihl+8:off+ihl+ln]
    if len(b)<36 or len(b)!=ln-8 or int.from_bytes(b[1:3],'little')!=len(b):raise ValueError('invalid_sensor_length')
    if b[0]!=0 or b[10] != (0 if sp==56400 else 1):raise ValueError('unexpected_sensor_format')
    return src,sp,b

class RawTracker:
    def __init__(self, warmup_seconds=10, utc_offset_seconds=0):
        self.utc_offset_ns=int(utc_offset_seconds)*1_000_000_000
        self.streams={};self.fault=None;self.ever_ready=False;self.clock_base=None
        self.warmup_seconds=warmup_seconds;self.good_since=None;self.armed=False
        self.startup_resets=0;self.last_startup_fault=None;self.sequence_gap_evidence=None
        self.last_point_gap=None;self.point_gap_pending={}
        self.recovery_since=None;self.recovery_events=[];self.last_recovery=None;self.secondary_fault=None;self.fault_since=None
        self.tolerated_point_gaps=0;self.last_point_gap_evidence=None
    def fail(self,reason):
        if self.fault is None:self.fault=reason
        # Never hide a second hard failure behind the first PTP failure.
        if self.armed and not recoverable_ptp_fault(reason):self.secondary_fault=reason
        self.recovery_since=None
    def recover_ptp(self,now,healthy,generation_ok):
        if not self.armed or not recoverable_ptp_fault(self.fault):return False
        if self.fault_since is None:self.fault_since=now
        self.recovery_events=[t for t in self.recovery_events if now-t<600]
        if self.secondary_fault or not generation_ok or len(self.recovery_events)>=2 or now-self.fault_since>120:
            reason=self.secondary_fault or ('ptp_clock_generation_changed' if not generation_ok else 'ptp_recovery_budget_or_timeout')
            self.fault=reason;self.secondary_fault=reason;self.recovery_since=None
            return False
        complete=len(self.streams)==4 and all(0<=now-v['seen']<.05 and abs(v['age_ns'])<=5_000_000 for v in self.streams.values())
        if not healthy or not complete or self.point_gap_pending:
            self.recovery_since=None;return False
        if self.recovery_since is None:self.recovery_since=now
        if now-self.recovery_since<10:return False
        self.last_recovery=dict(reason=self.fault,monotonic=now,healthy_seconds=now-self.recovery_since)
        self.recovery_events.append(now);self.fault=None;self.fault_since=None;self.recovery_since=None
        # Keep packet histories: no skipped sequence or clock discontinuity is forgiven.
        return True

    def observe(self,ip,port,b,cap_ns,now,wall_ns):
        key=f'{ip}:{port}';stamp=int.from_bytes(b[28:36],'little')-self.utc_offset_ns;seq=int.from_bytes(b[7:9],'little');age=cap_ns-stamp
        if b[11]!=1:self.fail(key+':not_PTP')
        if abs(age)>5_000_000:self.fail(key+':clock_age_ns='+str(age))
        if not 0<=wall_ns-cap_ns<=50_000_000:self.fail(key+':capture_backlog_or_clock_step')
        old=self.streams.get(key)
        if old:
            dt=stamp-old['stamp_ns']
            if dt<=0 or dt>(15_000_000 if port==56400 else 5_000_000):self.fail(key+':stamp_interval_ns='+str(dt))
            delta=(seq-old['sequence'])%65536
            if delta!=1:
                evidence=dict(stream=key,previous_sequence=old['sequence'],
                    sequence=seq,sequence_delta=delta,previous_stamp_ns=old['stamp_ns'],
                    stamp_ns=stamp,interval_ns=dt,capture_ns=cap_ns,
                    receive_wall_ns=wall_ns,monotonic=now)
                # Commissioning: every sequence gap is blocking; no loss grace.
                if self.sequence_gap_evidence is None:self.sequence_gap_evidence=evidence
                self.fail(key+':sequence_gap')
            elif key in self.point_gap_pending:
                if not 0<=now-self.point_gap_pending[key]<=.005:
                    self.fail(key+':point_gap_recovery_timeout')
                elif self.fault is None:
                    del self.point_gap_pending[key]
        self.streams[key]=dict(seen=now,stamp_ns=stamp,sequence=seq,age_ns=age,count=1+(old['count'] if old else 0),
                               max_abs_age_ns=max(abs(age),old['max_abs_age_ns'] if old else 0))
    def ready(self,now,wall):
        for key,started in self.point_gap_pending.items():
            if not 0<=now-started<=.005:self.fail(key+':point_gap_recovery_timeout')
        delta=wall-now
        if self.clock_base is not None and abs(delta-self.clock_base)>.002:self.fail('host_clock_step')
        self.clock_base=delta
        complete=len(self.streams)==4
        if complete:self.ever_ready=True
        if self.ever_ready and any(not 0<=now-v['seen']<.05 for v in self.streams.values()):self.fail('raw_stream_stale')
        if self.fault is not None:
            if not self.armed:
                # Preserve evidence but discard pre-qualification packet history.
                # A fresh full set and full healthy interval are required again.
                self.last_startup_fault=dict(reason=self.fault,monotonic=now,utc=wall,
                                             streams=self.streams,sequence_gap_evidence=self.sequence_gap_evidence)
                self.sequence_gap_evidence=None
                self.startup_resets+=1
                self.streams={};self.fault=None;self.ever_ready=False
                self.clock_base=None;self.good_since=None
            return False
        if not complete:
            self.good_since=None
            return False
        if self.good_since is None:self.good_since=now
        if now-self.good_since>=self.warmup_seconds:self.armed=True
        return self.armed

def main():
    import argparse
    parser=argparse.ArgumentParser(description='Passive timing monitor; no CAN access or transmitted packets')
    parser.add_argument('--seconds',type=int,default=1800)
    args=parser.parse_args()
    if not 30<=args.seconds<=3600:parser.error('seconds must be 30..3600')
    deadline=time.monotonic()+args.seconds
    if os.geteuid()!=0:raise SystemExit('通过 sudo 启动只读原始时间监测器；不打开CAN。')
    RAW_PATH.parent.mkdir(mode=0o755,exist_ok=True)
    owner=(RAW_PATH.parent/'owner.lock').open('a');fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();generation=str(uuid.uuid4())
    # Linux x86_64 SO_TIMESTAMPNS=35, SO_RXQ_OVFL=40. No NIC clock substitution.
    s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(3));s.bind(('enp85s0',0))
    s.setsockopt(socket.SOL_SOCKET,35,1);s.setsockopt(socket.SOL_SOCKET,40,1)
    s.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024);s.settimeout(.01)
    from ptp_health import lidar_utc_offset,read_health
    utc_offset=lidar_utc_offset()
    _,_,initial_health=read_health()
    tracker=RawTracker(utc_offset_seconds=utc_offset);running=True;last=0;reported=None
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    capture_stats=dict(packets=0,drops=0)
    def publish():
        nonlocal reported
        # AF_PACKET exposes reliable per-socket counters through PACKET_STATISTICS.
        packets,drops=struct.unpack('II',s.getsockopt(263,6,8))
        capture_stats['packets']+=packets;capture_stats['drops']+=drops
        if drops:tracker.fail('kernel_capture_drops')
        now=time.monotonic();wall=time.time()
        ok,reason,h=read_health()
        if not ok:tracker.fail('ptp_clock:'+reason)
        generation_known=bool(h.get('generation'))
        generation_ok=generation_known and h.get('generation')==initial_health.get('generation')
        if generation_known and not generation_ok:tracker.fail('ptp_clock_generation_changed')
        if not generation_known and reason not in PTP_TRANSIENT:tracker.fail('ptp_clock_generation_missing')
        ready=tracker.ready(now,wall)
        # This trial policy forbids automatic recovery after qualification.
        if tracker.recover_ptp(now,ok and generation_ok,generation_ok or (not generation_known and reason in PTP_TRANSIENT)):
            ready=tracker.ready(now,wall)
        d=dict(schema=POLICY,boot_id=boot,generation=generation,monotonic=now,utc=wall,ready=ready,
               fault=tracker.fault,streams=tracker.streams,
               ptp_recovery=dict(pending=bool(recoverable_ptp_fault(tracker.fault) and not tracker.secondary_fault),
                   healthy_seconds=0 if tracker.recovery_since is None else now-tracker.recovery_since,
                   last=tracker.last_recovery,blocked_by=tracker.secondary_fault,count_10min=len(tracker.recovery_events)),
               sequence_gap_evidence=tracker.sequence_gap_evidence,capture_stats=capture_stats,
               point_gap_policy='no_packet_loss_grace',
               tolerated_point_gaps=tracker.tolerated_point_gaps,
               last_point_gap_evidence=tracker.last_point_gap_evidence,
               point_gap_pending=tracker.point_gap_pending,
               phase='armed' if tracker.armed else 'warming',
               utc_offset_seconds=utc_offset,warmup_seconds=tracker.warmup_seconds,
               healthy_seconds=0 if tracker.good_since is None else now-tracker.good_since,
               startup_resets=tracker.startup_resets,last_startup_fault=tracker.last_startup_fault)
        tmp=RAW_PATH.with_suffix('.tmp');tmp.write_text(json.dumps(d));tmp.chmod(0o644);tmp.replace(RAW_PATH)
        state=(ready,tracker.fault,tracker.startup_resets,tracker.tolerated_point_gaps)
        if state!=reported:print(json.dumps(d),flush=True);reported=state
    try:
        publish()
        while running and time.monotonic()<deadline:
            try:
                frame,anc,flags,address=s.recvmsg(65535,256)
                now=time.monotonic();wall_ns=time.time_ns();cap=None
                if flags&(socket.MSG_TRUNC|socket.MSG_CTRUNC):tracker.fail('truncated_capture')
                for level,typ,data in anc:
                    if level==socket.SOL_SOCKET and typ==35:
                        sec,ns=struct.unpack('@ll',data[:16]);cap=sec*10**9+ns
                    if level==socket.SOL_SOCKET and typ==40 and struct.unpack('I',data[:4])[0]:tracker.fail('kernel_capture_drops')
                packet=decode(frame)
                if packet:
                    if cap is None:tracker.fail('kernel_timestamp_missing')
                    else:tracker.observe(*packet,cap,now,wall_ns)
            except socket.timeout:pass
            except (ValueError,struct.error) as e:tracker.fail(str(e))
            if time.monotonic()-last>=.02:publish();last=time.monotonic()
    finally:
        tracker.armed=True  # Shutdown must always publish a terminal fault.
        tracker.fail('monitor_stopped');publish();s.close();owner.close()
if __name__=='__main__':main()
