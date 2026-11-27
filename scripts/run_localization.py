#!/usr/bin/env python3
"""Dual LiDAR + FAST-LIO local odometry only. AMCL owns map->trial_odom.
No CAN driver, no joystick process, no motor enable command.
"""
import argparse,datetime,fcntl,json,os,signal,subprocess,sys,time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parents[1];TRIAL=ROOT/'mapping_trial';sys.path.insert(0,str(TRIAL))
from live_gate import read as raw_health

def main():
 p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=1800);a=p.parse_args()
 if not 60<=a.seconds<=3600:raise ValueError('seconds must be 60..3600')
 sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'calibration'));from capture import preflight
 preflight(json.loads((Path(__file__).resolve().parents[1]/'calibration/config/settings.json').read_text()))
 for d in Path('/proc').iterdir():
  if not d.name.isdigit() or int(d.name)==os.getpid():continue
  try:names={Path(x.decode(errors='replace')).name for x in (d/'cmdline').read_bytes().split(b'\0') if x}
  except OSError:continue
  if names & {'fastlio_mapping','run_mapping.py','run_localization.py','remote_bridge','guarded_remote.py'}:raise RuntimeError('Existing mapping/control process: '+d.name)
 lock=Path('/home/pc/lidar test/logs/driver.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 run=ROOT/'localization_runs'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');run.mkdir(parents=True);children=[];logs=[];failure=None
 def launch(name,cmd):
  f=(run/(name+'.log')).open('w');logs.append(f);proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,cwd=run);children.append((name,proc));return proc
 def stop(*_):raise KeyboardInterrupt()
 for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,stop)
 rclpy.init();node=rclpy.create_node('chassis_localization_supervisor');odom=[0.,0.]
 def receive(m):odom[:]=[time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9]
 sub=node.create_subscription(Odometry,'/Odometry',receive,100)
 started=time.monotonic();stable=None;lio_at=None;generation=None
 try:
  dev=json.loads((TRIAL/'config/devices.json').read_text());cmd=[str(Path(__file__).resolve().parents[1]/'sensors/driver_workspace/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'),'--ros-args','-r','__node:=dual_lidar_navigation']
  for k,v in {'user_config_path':str(TRIAL/'config/dual.json'),'xfer_format':0,'multi_topic':1,'data_src':0,'publish_freq':10.0,'output_data_type':0,'frame_id':'livox_frame','ptp_utc_offset_seconds':0}.items():cmd+=['-p',f'{k}:={v}']
  for d in dev['devices']:
   suffix=d['ip'].replace('.','_');cmd+=['-r',f'/livox/lidar_{suffix}:=/{d["name"]}/points','-r',f'/livox/imu_{suffix}:=/{d["name"]}/imu']
  launch('fusion',['python3',str(TRIAL/'fusion_node.py')]);launch('adapter',['python3',str(TRIAL/'cloud_adapter.py')]);launch('driver',cmd)
  print('等待原始时间监测和双雷达稳定；初始化时保持静止。本入口不启动电机。',flush=True)
  while time.monotonic()-started<a.seconds:
   rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic();reason=None
   for name,proc in children:
    if proc.poll() is not None:raise RuntimeError(name+' exited: '+str(proc.returncode))
   try:
    raw=raw_health(generation);generation=raw['generation'];f=json.loads((TRIAL/'logs/fusion_status.json').read_text())
    if not (0<=now-f.get('monotonic',0)<1. and f.get('ptp_master_healthy') and f.get('device_ptp_reason')=='ok' and f.get('last_output_age_s') is not None and f['last_output_age_s']<.3):raise RuntimeError('fusion not healthy')
   except Exception as exc:reason=str(exc)
   if lio_at is not None:
    if reason:raise RuntimeError(reason)
    if now-lio_at>20 and (now-odom[0]>.3 or not -.05<=time.time()-odom[1]<.3):raise RuntimeError('LIO odometry stale')
   elif reason:
    stable=None
    if now-started>60:raise RuntimeError(reason)
   else:
    stable=stable or now
    if now-stable>=10:
     launch('fastlio',[str(ROOT/'mapping_ws/install/lib/fast_lio_sam/fastlio_mapping'),'--ros-args','--params-file',str(ROOT/'config/localization_lio.yaml'),'-p','result_save/output_dir:='+str(run/'lio_output')]);lio_at=now
     print('连续里程计已启动；map→trial_odom 由下一步 AMCL 提供。',flush=True)
 except (Exception,KeyboardInterrupt) as exc:failure=repr(exc)
 finally:
  cleanup=[]
  # Stop sensors first so FAST-LIO can drain inputs and finish naturally.
  for name,proc in children:
   if name=='fastlio':continue
   if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
  for name,proc in reversed(children):
   try:proc.wait(timeout=60 if name=='fastlio' else 10)
   except subprocess.TimeoutExpired:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait();failure='Forced cleanup: '+name
   cleanup.append({'name':name,'returncode':proc.returncode})
  for f in logs:f.close()
  (run/'report.json').write_text(json.dumps({'failure':failure,'hardware_control_started':False,'cleanup':cleanup},indent=2));node.destroy_node();rclpy.try_shutdown();lock.close();print(run)
 return 0 if failure in (None,'KeyboardInterrupt()') else 1
if __name__=='__main__':raise SystemExit(main())
