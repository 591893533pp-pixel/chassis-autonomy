#!/usr/bin/env python3
"""Integrated controller. One joystick reader; disjoint wheel/PV CAN owners."""
import argparse,datetime,fcntl,json,math,os,signal,subprocess,sys,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'program/remote'))
sys.path.insert(0,str(ROOT/'lift_reference'))
from run_remote import manual_config
from vendor.navslam_core import TeleopCore
from lift_pv_remote import Joystick,Bus,enable_pv,operation_enabled,record_and_validate
from lift_policy import LiftPolicy,COUNTS_MM
from stationary_gate import StationaryGate
from lift_limits import LIMITS,MAX_SPEED_MM_S,MAX_POSITION_MM,within_position_fault_range,POSITION_FAULT_MARGIN_MM
from lift_profile import VelocityProfile
from control_timing import CONTROL_HZ, Periodic

class LiftWorker:
    def __init__(self,folder,speed,allow_simultaneous=False):
        self.folder=folder;self.policy=LiftPolicy(speed);self.done=threading.Event();self.lock=threading.Lock()
        self.allow_simultaneous=allow_simultaneous;self.armed=False
        self.command=(time.monotonic(),False,0,False);self.error=None;self.ready=False;self.stopped=True
        self.profile=VelocityProfile()
        self.zero_gate=StationaryGate(10);self.stop_reason='not_enabled'
        self.thread=threading.Thread(target=self.run,name='lift-pv',daemon=False);self.thread.start()
    def update(self,enabled,direction,wheels_stopped):
        with self.lock:self.command=(time.monotonic(),enabled,direction,wheels_stopped)
    def read_command(self):
        with self.lock:c=self.command
        if time.monotonic()-c[0]>.3:raise RuntimeError('主遥控心跳超过300ms')
        if self.done.is_set():return (c[0],False,0,False)
        return c
    def run(self):
        bus=None;touched=False;last=0;deadline=None;reference=0
        log=(self.folder/'lift_events.jsonl').open('w',buffering=1)
        feedback=(self.folder/'lift_feedback.jsonl').open('w',buffering=65536)
        timing=(self.folder/'lift_timing.jsonl').open('w',buffering=65536)
        def event(name,**kw):log.write(json.dumps(dict(t=time.monotonic(),event=name,**kw),ensure_ascii=False)+'\n')
        try:
            bus=Bus(self.folder);before=bus.snapshot()
            (self.folder/'lift_before.json').write_text(json.dumps(before,indent=2))
            if operation_enabled(before['statusword']) or abs(before['velocity_counts_s'])>10:raise RuntimeError('升降已使能或运动，拒绝接管')
            if not within_position_fault_range(before['position_mm'],MAX_POSITION_MM):raise RuntimeError(f'升降反馈超出-1–{MAX_POSITION_MM+1:g}mm，拒绝自动归零')
            self.ready=True;enabled=False;last_tick=time.monotonic();schedule=Periodic(late_tolerance=.002);last_flush=last_tick
            while not self.done.is_set():
                now=time.monotonic()
                if enabled and now-last_tick>.3:raise RuntimeError('升降反馈循环超过300ms')
                profile_dt=now-last_tick
                last_tick=now
                _,request,direction,wheels_stopped=self.read_command()
                if request and not enabled:
                    if direction or not wheels_stopped:schedule.wait();continue
                    def neutral():
                        _,e,d,ws=self.read_command()
                        if not e or d or not ws:raise RuntimeError('升降使能时输入非中立')
                    touched=True;enable_pv(bus,neutral);enabled=True;self.armed=True;last_tick=time.monotonic();schedule=Periodic(late_tolerance=.002);schedule.wait();event('enabled_pv');continue
                if not enabled:schedule.wait();continue
                state=bus.snapshot()
                record_and_validate(feedback,event,state,last,before['position_mm'],reference if deadline else None,position_max_mm=MAX_POSITION_MM,position_fault_margin_mm=POSITION_FAULT_MARGIN_MM)
                if abs(state['velocity_counts_s'])/COUNTS_MM>MAX_SPEED_MM_S:raise RuntimeError(f'升降反馈速度超过{MAX_SPEED_MM_S:g}mm/s')
                _,request,direction,wheels_stopped=self.read_command()
                target=self.policy.target(direction if request and (self.allow_simultaneous or wheels_stopped) else 0,state['position_mm'])
                zero_verified=self.zero_gate.observe(time.monotonic(),
                    [state['velocity_counts_s']], time.monotonic(),zero_command=(last==0))
                self.stop_reason=self.zero_gate.reason
                if deadline:
                    if zero_verified:deadline=None;event('zero_verified',**state)
                    elif time.monotonic()>deadline:raise RuntimeError('升降写零后0.5s内未确认停车')
                    else:target=0
                target=math.trunc(self.profile.step(target/COUNTS_MM,profile_dt)*COUNTS_MM)
                # Refresh PV target every cycle, including unchanged/zero targets.
                bus.sdo(0x60ff,target)
                if target!=last:
                    self.stopped=False
                    self.zero_gate.reset();self.stop_reason='target_changed'
                    event('velocity_target',counts_s=target,mm_s=target/COUNTS_MM,position_mm=state['position_mm'])
                    if last and not target:reference=last;deadline=time.monotonic()+.5
                    last=target
                self.stopped=not target and deadline is None and zero_verified
                timing.write(json.dumps(dict(t=now,cycle_dt_s=profile_dt,work_s=time.monotonic()-now,missed_slots=schedule.missed,late_cycles=schedule.late_cycles,target_counts_s=target))+'\n')
                if time.monotonic()-last_flush>=.1:
                    feedback.flush();timing.flush();last_flush=time.monotonic()
                schedule.wait()
        except Exception as exc:self.error=str(exc);self.stopped=False;event('fault',message=str(exc))
        finally:
            self.armed=False
            if bus and touched:
                for name,action in [('zero',bus.zero),('disable',lambda:bus.sdo(0x6040,0,2)),('preop',lambda:bus.send(0,bytes([128,1])))]:
                    try:
                        action()
                        if name=='zero':time.sleep(.15)
                        event('cleanup_'+name)
                    except Exception as exc:self.error='升降停止未确认：'+str(exc);event('cleanup_failed',message=str(exc))
                try:
                    s=bus.snapshot();event('final_readback',**s)
                    if operation_enabled(s['statusword']) or abs(s['velocity_counts_s'])>10:self.error='升降末次停车/失能反馈未确认'
                except Exception as exc:self.error=str(exc)
            if bus:bus.sock.close();bus.log.close()
            timing.close();feedback.close();log.close()
    def stop(self):
        self.done.set();self.thread.join(timeout=6)
        if self.thread.is_alive():raise RuntimeError('升降线程尚未退出，请使用物理急停')
        if self.error:raise RuntimeError(self.error)

class WheelFeedback:
    def __init__(self,path):
        self.path=path;self.file=None;self.columns=None;self.line=None;self.partial=''
        self.zero_gate=StationaryGate(.1,dimensions=4,release_limit=.15)
    def stationary(self):
        if self.file is None:
            if not self.path.exists():return False
            self.file=self.path.open()
        data=self.partial+self.file.read();parts=data.split('\n');self.partial=parts.pop()
        for line in parts:
            values=line.split(',')
            if self.columns is None:self.columns=values
            elif len(values)==len(self.columns):
                self.line=dict(zip(self.columns,values))
                self._observe()
        if not self.line:return False
        return self._observe()
    def _observe(self):
        try:
            return self.zero_gate.observe(float(self.line['monotonic_s']),
                [float(self.line[f'wheel{i}_feedback_rad_s']) for i in range(1,5)],time.monotonic(),
                valid=(self.line['feedback_valid']=='1' and 0<=float(self.line['feedback_age_ms'])<200
                       and all(float(self.line[f'wheel{i}_fault'])==0 for i in range(1,5))),
                zero_command=all(float(self.line[f'wheel{i}_cmd_rad_s'])==0 for i in range(1,5)))
        except (ValueError,KeyError):return self.zero_gate.reset('invalid_feedback')

def cleanup_report(*args,**kwargs):
    try:print(*args,**kwargs)
    except Exception:pass

def route_motion(command,direction,lift_present,lift_armed,lift_stopped,simultaneous):
    chassis_request=abs(command.x)+abs(command.y)+abs(command.yaw)>1e-8
    lift_direction=direction if simultaneous or not chassis_request else 0
    block=lift_present and (not lift_armed or (not simultaneous and (direction!=0 or not lift_stopped)))
    xyz=(0.,0.,0.) if block else (command.x,command.y,command.yaw)
    return xyz,lift_direction,block

def main():
    defaults=json.loads((ROOT/'config/defaults.json').read_text())
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dry-run',action='store_true');p.add_argument('--chassis-only',action='store_true')
    p.add_argument('--exclusive-motion',action='store_true',help='恢复底盘/升降互锁；默认独立遥控允许同时操作')
    p.add_argument('--lift-speed-mm-s',type=float,default=defaults['lift_speed_mm_s'])
    p.add_argument('--max-linear-speed',type=float,default=defaults['max_linear_speed']);p.add_argument('--initial-gear',type=int,default=defaults['initial_gear'])
    p.add_argument('--log-dir',type=Path);p.add_argument('--motion-enable-file',type=Path)
    a=p.parse_args();LiftPolicy(a.lift_speed_mm_s);cfg=manual_config(a.max_linear_speed,a.initial_gear)
    simultaneous=not a.chassis_only and not a.exclusive_motion
    if a.dry_run:
        print(json.dumps(dict(can='can0, classic 1Mbps',control_target_hz=CONTROL_HZ,lift_feedback_target_hz=CONTROL_HZ,lift=not a.chassis_only,lift_mode='PV(3)',lift_limits=LIMITS,lift_range_mm=[0,MAX_POSITION_MM],
            lift_request_mm_s=a.lift_speed_mm_s,lift_max_mm_s=MAX_SPEED_MM_S,quantized_counts_s=math.floor(a.lift_speed_mm_s*COUNTS_MM),
            chassis_max_m_s=a.max_linear_speed,initial_gear=a.initial_gear,simultaneous_motion=simultaneous,logs=str(a.log_dir or ROOT/'logs'),hardware_started=False),ensure_ascii=False,indent=2));return 0
    lock=open(f'/tmp/poke-remote-{os.getuid()}.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if not Path('/dev/input/js0').exists():raise RuntimeError('未找到DINPUT手柄 /dev/input/js0')
    bridge=ROOT/'program/build/remote_bridge'
    if not bridge.exists():raise RuntimeError('先执行 ./build.sh 编译本目录遥控程序')
    folder=a.log_dir or ROOT/'logs'/('remote_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    (ROOT/'LAST_LOG.txt').write_text(str(folder)+'\n')
    (ROOT/'STATE.json').write_text(json.dumps(dict(pid=os.getpid(),start_ticks=Path('/proc/self/stat').read_text().split()[21],log=str(folder),running=True)))
    (folder/'session.json').write_text(json.dumps(dict(control_target_hz=CONTROL_HZ,lift_feedback_target_hz=CONTROL_HZ,lift=not a.chassis_only,lift_mode='PV',lift_limits=LIMITS,lift_request_mm_s=a.lift_speed_mm_s,
        lift_max_mm_s=MAX_SPEED_MM_S,lift_range_mm=[0,MAX_POSITION_MM],simultaneous_motion=simultaneous,chassis_config=vars(cfg),started=datetime.datetime.now().astimezone().isoformat()),ensure_ascii=False,indent=2))
    print('日志：'+str(folder),flush=True)
    print('控制目标100Hz；升降SDO反馈目标100Hz，实际频率请核验日志。',flush=True)
    print('松开摇杆/按键后按START使能；左摇杆/方向键平移，右摇杆旋转，LB/RB换档；X升/Y降，松开停止；双扳机退出。',flush=True)
    print(f'升降PV：0–{MAX_POSITION_MM:g}mm，请求速度'+str(a.lift_speed_mm_s)+f'mm/s，命令上限{MAX_SPEED_MM_S:g}mm/s。' if not a.chassis_only else '建图/底盘模式：升降锁定。',flush=True)
    if not a.chassis_only:print('运动模式：底盘与升降可同时操作；首次使能仍需保持中立。' if simultaneous else '运动模式：底盘与升降互锁。',flush=True)
    proc=joy=lift=None;exitcode=0
    bridge_log=(folder/'bridge.log').open('w');commands=(folder/'commands.jsonl').open('w',buffering=1)
    stopping=False
    def stop(*_):
        # Once shutdown starts, another terminal signal must not interrupt cleanup.
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            signal.signal(sig,signal.SIG_IGN)
        raise KeyboardInterrupt
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
    try:
        joy=Joystick(folder)
        # This C++ bridge owns only wheel frames. It never enables/commands the lift.
        proc=subprocess.Popen([str(bridge),'--hardware','socketcan:can0','--log',str(folder/'feedback.csv')],stdin=subprocess.PIPE,stdout=bridge_log,stderr=bridge_log,text=True,start_new_session=True)
        if not a.chassis_only:lift=LiftWorker(folder,a.lift_speed_mm_s,allow_simultaneous=simultaneous)
        core=TeleopCore(cfg);wheel=WheelFeedback(folder/'feedback.csv');last_state=None;schedule=Periodic()
        while True:
            tick=time.monotonic()
            if proc.poll() is not None:raise RuntimeError('底盘桥接退出，请看bridge.log')
            if lift and lift.error:raise RuntimeError('升降异常：'+lift.error)
            sample=joy.get();core.output(tick)
            if sample and sample[3]:core.receive_joy(sample[1],sample[2],tick)
            permitted=a.motion_enable_file is None or a.motion_enable_file.exists()
            if not permitted:core._disarm('mapping_static_or_not_ready')
            command=core.output(tick)
            direction=0 if a.chassis_only else (1 if command.lift>0 else -1 if command.lift<0 else 0)
            if not a.chassis_only and sample and sample[3] and sample[2][3] and sample[2][4]:direction=2
            # First arming is stationary/neutral in both modes. After arming,
            # independent CAN owners may run both axes in simultaneous mode.
            chassis_request=abs(command.x)+abs(command.y)+abs(command.yaw)>1e-8
            wheels_stopped=wheel.stationary() if lift else None
            xyz,lift_direction,block=route_motion(command,direction,lift is not None,
                lift.armed if lift else False,lift.stopped if lift else True,simultaneous)
            if lift:lift.update(core.enabled,lift_direction,wheels_stopped and not chassis_request)
            enabled=core.enabled and (lift is None or lift.ready)
            proc.stdin.write(f'{tick:.9f} {int(enabled)} {xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f} 0\n');proc.stdin.flush()
            status=core.status(tick)
            state=(enabled,status['reason'],status['speed_level'])
            if state!=last_state:print('遥控状态：'+json.dumps(status,ensure_ascii=False),flush=True);last_state=state
            commands.write(json.dumps(dict(t=tick,enabled=enabled,vx=xyz[0],vy=xyz[1],yaw=xyz[2],lift_direction=direction,permitted=permitted,
                missed_slots=schedule.missed,requested_vx=command.x,requested_vy=command.y,requested_yaw=command.yaw,
                interlock_blocked=block,simultaneous_motion=simultaneous,lift_command_direction=lift_direction,
                lift_armed=lift.armed if lift else None,wheels_stopped=wheels_stopped,wheel_stop_reason=wheel.zero_gate.reason if lift else None,
                lift_stopped=lift.stopped if lift else None,lift_stop_reason=lift.stop_reason if lift else None))+'\n')
            schedule.wait()
    except KeyboardInterrupt:print('正在停止底盘与升降…',flush=True)
    except Exception as exc:print('遥控故障：'+str(exc),flush=True);exitcode=1
    finally:
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,signal.SIG_IGN)
        # Repeated stop requests must not interrupt zero/disable/readback.
        stopping=True
        # Stop both axes promptly; no SIGKILL that would bypass wheel cleanup.
        if proc and proc.poll() is None:
            try:proc.stdin.write(f'{time.monotonic():.9f} 0 0 0 0 0\n');proc.stdin.flush()
            except (BrokenPipeError,OSError):pass
            proc.terminate()
        if lift:
            try:lift.stop()
            except Exception as exc:cleanup_report(str(exc),flush=True);exitcode=1
        if proc:
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:cleanup_report('底盘清理超时，停车未确认，请用物理急停',flush=True);exitcode=1
            if proc.returncode not in (None,0):exitcode=1
        if joy:joy.done.set();joy.thread.join(timeout=1)
        commands.close();bridge_log.close()
        (folder/'exit.json').write_text(json.dumps(dict(exit_code=exitcode,ended=datetime.datetime.now().astimezone().isoformat()),indent=2))
    (ROOT/'STATE.json').write_text(json.dumps(dict(pid=os.getpid(),log=str(folder),running=False,exit_code=exitcode)))
    return exitcode
if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,RuntimeError,BlockingIOError) as e:raise SystemExit(str(e))
