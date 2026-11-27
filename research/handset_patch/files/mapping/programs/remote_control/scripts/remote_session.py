#!/usr/bin/env python3
"""Integrated controller. One joystick reader; disjoint wheel/PV CAN owners."""
import argparse,datetime,fcntl,json,math,os,signal,subprocess,sys,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'program/remote'))
sys.path.insert(0,str(ROOT/'lift_reference'))
from run_remote import manual_config
from joystick_layout import detect_device, wait_initialized
from control_timing import CONTROL_HZ, Periodic
from vendor.navslam_core import TeleopCore
from lift_pv_remote import Joystick,Bus,enable_pv,operation_enabled,record_and_validate
from lift_policy import LiftPolicy,COUNTS_MM

class LiftWorker:
    def __init__(self,folder,speed):
        self.folder=folder;self.policy=LiftPolicy(speed);self.done=threading.Event();self.lock=threading.Lock()
        self.command=(time.monotonic(),False,0,False);self.error=None;self.ready=False;self.stopped=True
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
        feedback=(self.folder/'lift_feedback.jsonl').open('w',buffering=1)
        def event(name,**kw):log.write(json.dumps(dict(t=time.monotonic(),event=name,**kw),ensure_ascii=False)+'\n')
        try:
            bus=Bus(self.folder);before=bus.snapshot()
            (self.folder/'lift_before.json').write_text(json.dumps(before,indent=2))
            if operation_enabled(before['statusword']) or abs(before['velocity_counts_s'])>10:raise RuntimeError('升降已使能或运动，拒绝接管')
            if not 0<=before['position_mm']<=250:raise RuntimeError('升降位置超出0–250mm，拒绝自动归零')
            self.ready=True;enabled=False;last_tick=time.monotonic()
            while not self.done.is_set():
                now=time.monotonic()
                if enabled and now-last_tick>.3:raise RuntimeError('升降反馈循环超过300ms')
                last_tick=now
                _,request,direction,wheels_stopped=self.read_command()
                if request and not enabled:
                    if direction or not wheels_stopped:time.sleep(.02);continue
                    def neutral():
                        _,e,d,ws=self.read_command()
                        if not e or d or not ws:raise RuntimeError('升降使能时输入非中立')
                    touched=True;enable_pv(bus,neutral);enabled=True;last_tick=time.monotonic();event('enabled_pv');continue
                if not enabled:time.sleep(.02);continue
                state=bus.snapshot()
                record_and_validate(feedback,event,state,last,before['position_mm'],reference if deadline else None)
                if abs(state['velocity_counts_s'])/COUNTS_MM>50:raise RuntimeError('升降反馈速度超过50mm/s')
                _,request,direction,wheels_stopped=self.read_command()
                target=self.policy.target(direction if request and wheels_stopped else 0,state['position_mm'])
                if deadline:
                    if abs(state['velocity_counts_s'])<=10:deadline=None;event('zero_verified',**state)
                    elif time.monotonic()>deadline:raise RuntimeError('升降写零后0.5s内未确认停车')
                    else:target=0
                if target!=last:
                    self.stopped=False
                    bus.sdo(0x60ff,target);event('velocity_target',counts_s=target,mm_s=target/COUNTS_MM,position_mm=state['position_mm'])
                    if last and not target:reference=last;deadline=time.monotonic()+.5
                    last=target
                self.stopped=not target and deadline is None and abs(state['velocity_counts_s'])<=10
                time.sleep(.02)
        except Exception as exc:self.error=str(exc);self.stopped=False;event('fault',message=str(exc))
        finally:
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
            feedback.close();log.close()
    def stop(self):
        self.done.set();self.thread.join(timeout=6)
        if self.thread.is_alive():raise RuntimeError('升降线程尚未退出，请使用物理急停')
        if self.error:raise RuntimeError(self.error)

class WheelFeedback:
    def __init__(self,path):self.path=path;self.file=None;self.columns=None;self.line=None;self.partial=''
    def stationary(self):
        if self.file is None:
            if not self.path.exists():return False
            self.file=self.path.open()
        data=self.partial+self.file.read();parts=data.split('\n');self.partial=parts.pop()
        for line in parts:
            values=line.split(',')
            if self.columns is None:self.columns=values
            elif len(values)==len(self.columns):self.line=dict(zip(self.columns,values))
        if not self.line:return False
        try:
            return (0<=time.monotonic()-float(self.line['monotonic_s'])<.2 and self.line['feedback_valid']=='1'
                    and max(abs(float(self.line[f'wheel{i}_feedback_rad_s'])) for i in range(1,5))<.1)
        except (ValueError,KeyError):return False

def main():
    defaults=json.loads((ROOT/'config/defaults.json').read_text())
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dry-run',action='store_true');p.add_argument('--chassis-only',action='store_true')
    p.add_argument('--lift-speed-mm-s',type=float,default=defaults['lift_speed_mm_s'])
    p.add_argument('--max-linear-speed',type=float,default=defaults['max_linear_speed']);p.add_argument('--initial-gear',type=int,default=defaults['initial_gear'])
    p.add_argument('--log-dir',type=Path);p.add_argument('--motion-enable-file',type=Path)
    a=p.parse_args();LiftPolicy(a.lift_speed_mm_s);cfg=manual_config(a.max_linear_speed,a.initial_gear)
    if a.dry_run:
        print(json.dumps(dict(can='can0, classic 1Mbps',lift=not a.chassis_only,lift_mode='PV(3)',lift_range_mm=[0,250],
            lift_request_mm_s=a.lift_speed_mm_s,lift_max_mm_s=50,quantized_counts_s=math.floor(a.lift_speed_mm_s*COUNTS_MM),
            chassis_max_m_s=a.max_linear_speed,initial_gear=a.initial_gear,logs=str(a.log_dir or ROOT/'logs'),hardware_started=False),ensure_ascii=False,indent=2));return 0
    lock=open(f'/tmp/poke-remote-{os.getuid()}.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if not Path('/dev/input/js0').exists():raise RuntimeError('未找到手柄 /dev/input/js0')
    layout=detect_device() if a.chassis_only else None
    if layout is not None:
        cfg=layout.chassis_config(cfg)
    bridge=ROOT/'program/build/remote_bridge'
    if not bridge.exists():raise RuntimeError('先执行 ./build.sh 编译本目录遥控程序')
    folder=a.log_dir or ROOT/'logs'/('remote_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    if layout is not None:
        (folder/'joystick_layout.json').write_text(json.dumps(vars(layout),ensure_ascii=False,indent=2))
        print('手柄布局：'+layout.profile+'；START使能/失能，LB降档/RB升档，双扳机退出',flush=True)
    (ROOT/'LAST_LOG.txt').write_text(str(folder)+'\n')
    (ROOT/'STATE.json').write_text(json.dumps(dict(pid=os.getpid(),start_ticks=Path('/proc/self/stat').read_text().split()[21],log=str(folder),running=True)))
    (folder/'session.json').write_text(json.dumps(dict(lift=not a.chassis_only,lift_mode='PV',lift_request_mm_s=a.lift_speed_mm_s,
        lift_max_mm_s=50,lift_range_mm=[0,250],chassis_config=vars(cfg),started=datetime.datetime.now().astimezone().isoformat()),ensure_ascii=False,indent=2))
    print('日志：'+str(folder),flush=True)
    print('松开摇杆/按键后按START使能；左摇杆/方向键平移，右摇杆旋转，LB/RB换档；X升/Y降，松开停止；双扳机退出。',flush=True)
    print('升降PV：0–250mm，请求速度'+str(a.lift_speed_mm_s)+'mm/s，命令上限50mm/s。' if not a.chassis_only else '建图/底盘模式：升降锁定。',flush=True)
    proc=joy=lift=None;exitcode=0
    bridge_log=(folder/'bridge.log').open('w');commands=(folder/'commands.jsonl').open('w',buffering=1)
    def stop(*_):
        # Once shutdown starts, another terminal signal must not interrupt cleanup.
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            signal.signal(sig,signal.SIG_IGN)
        raise KeyboardInterrupt
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
    try:
        joy=Joystick(folder,layout=layout)
        wait_initialized(joy)
        # This C++ bridge owns only wheel frames. It never enables/commands the lift.
        proc=subprocess.Popen([str(bridge),'--hardware','socketcan:can0','--log',str(folder/'feedback.csv')],stdin=subprocess.PIPE,stdout=bridge_log,stderr=bridge_log,text=True,start_new_session=True)
        if not a.chassis_only:lift=LiftWorker(folder,a.lift_speed_mm_s)
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
            # Lift only starts after measured wheel zero. Wheels wait for confirmed lift zero.
            chassis_request=abs(command.x)+abs(command.y)+abs(command.yaw)>1e-8
            if lift:lift.update(core.enabled,0 if chassis_request else direction,wheel.stationary())
            block=direction!=0 or (lift is not None and not lift.stopped)
            xyz=(0.,0.,0.) if block else (command.x,command.y,command.yaw)
            enabled=core.enabled and (lift is None or lift.ready)
            proc.stdin.write(f'{tick:.9f} {int(enabled)} {xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f} 0\n');proc.stdin.flush()
            status=core.status(tick)
            state=(enabled,status['reason'],status['speed_level'])
            if state!=last_state:print('遥控状态：'+json.dumps(status,ensure_ascii=False),flush=True);last_state=state
            commands.write(json.dumps(dict(t=tick,enabled=enabled,vx=xyz[0],vy=xyz[1],yaw=xyz[2],lift_direction=direction,permitted=permitted))+'\n')
            schedule.wait()
    except KeyboardInterrupt:print('正在停止底盘与升降…',flush=True)
    except Exception as exc:print('遥控故障：'+str(exc),flush=True);exitcode=1
    finally:
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,signal.SIG_IGN)
        # Stop both axes promptly; no SIGKILL that would bypass wheel cleanup.
        if proc and proc.poll() is None:
            try:proc.stdin.write(f'{time.monotonic():.9f} 0 0 0 0 0\n');proc.stdin.flush()
            except (BrokenPipeError,OSError):pass
            proc.terminate()
        if lift:
            try:lift.stop()
            except Exception as exc:print(str(exc),flush=True);exitcode=1
        if proc:
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:print('底盘清理超时，停车未确认，请用物理急停',flush=True);exitcode=1
            if proc.returncode not in (None,0):exitcode=1
        if joy:joy.done.set();joy.thread.join(timeout=1)
        commands.close();bridge_log.close()
        (folder/'exit.json').write_text(json.dumps(dict(exit_code=exitcode,ended=datetime.datetime.now().astimezone().isoformat()),indent=2))
    (ROOT/'STATE.json').write_text(json.dumps(dict(pid=os.getpid(),log=str(folder),running=False,exit_code=exitcode)))
    return exitcode
if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,RuntimeError,BlockingIOError) as e:raise SystemExit(str(e))
