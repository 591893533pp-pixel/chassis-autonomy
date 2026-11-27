#!/usr/bin/env python3
"""Explicit navigation trial. Default check-only. Motor startup requires acceptance,
--enable-motion, stable live inputs, then a physical START press on the gamepad.
"""
import argparse,datetime,json,math,os,signal,subprocess,sys,time,uuid
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'mapping_trial'))
from live_gate import read as raw_health
from motion_lease import publish
from navigation_gate import require_acceptance,velocity_command

def main():
 p=argparse.ArgumentParser();p.add_argument('--enable-motion',action='store_true');p.add_argument('--seconds',type=int,default=900);a=p.parse_args()
 if not 30<=a.seconds<=900:raise ValueError('seconds must be 30..900')
 if a.enable_motion:require_acceptance()
 run=ROOT/'navigation_runs'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');run.mkdir(parents=True)
 rclpy.init();n=rclpy.create_node('chassis_navigation_guard');tf=Buffer();listener=TransformListener(tf,n)
 last={};pose=None;previous=None;velocity=(0.,0.,0.);fault=[];perception={};localization=None;raw_generation=None;proc=None;map_transform=None;last_stamp={}
 boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();session=str(uuid.uuid4());lease=run/'lease.json';command=run/'command.json';started=time.monotonic();stable=None;active=None
 def receive(kind,m):
  nonlocal pose,previous,velocity,localization,perception
  now=time.monotonic();last[kind]=now
  if hasattr(m,'header'):
   stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
   if kind in ('odom','costmap') and not -.05<=time.time()-stamp<.5:fault.append(kind+'_stamp_stale')
   if kind=='odom' and stamp<=last_stamp.get(kind,0):fault.append('odom_time_nonmonotonic')
   last_stamp[kind]=stamp
  if kind=='command':
   velocity=(m.linear.x,m.linear.y,m.angular.z)
   if not np.isfinite(velocity).all() or math.hypot(*velocity[:2])>.050001 or abs(velocity[2])>.040001:fault.append('command_limit')
  elif kind=='odom':
   q=m.pose.pose.orientation;v=m.pose.pose.position;pose=(np.array([v.x,v.y,v.z]),Rotation.from_quat([q.x,q.y,q.z,q.w]),stamp)
   if previous:
    dt=stamp-previous[2]
    if np.linalg.norm(pose[0]-previous[0])>.05+.3*max(0,dt) or (pose[1]*previous[1].inv()).magnitude()>.1+.4*max(0,dt):fault.append('local_odometry_jump')
   previous=pose
  elif kind=='amcl':localization=m
  elif kind=='perception':perception=json.loads(m.data)
 subscriptions=[n.create_subscription(t,topic,lambda m,k=k:receive(k,m),100) for k,t,topic in [('command',Twist,'/chassis_nav/cmd_vel_safe'),('odom',Odometry,'/chassis_nav/odom'),('costmap',OccupancyGrid,'/local_costmap/costmap'),('amcl',PoseWithCovarianceStamped,'/amcl_pose'),('perception',String,'/chassis_nav/perception_status')]]
 log=(run/'guard.jsonl').open('w',buffering=1);childlog=(run/'control.log').open('w');error=None
 try:
  while time.monotonic()-started<a.seconds:
   rclpy.spin_once(n,timeout_sec=.01);now=time.monotonic();reason=None
   try:
    if fault:raise RuntimeError(fault[0])
    raw=raw_health(raw_generation);raw_generation=raw['generation']
    for k,limit in [('odom',.3),('costmap',.5),('perception',.25)]:
     if now-last.get(k,0)>limit:raise RuntimeError(k+'_not_fresh')
    for name in ['front_left','rear_right']:
     if perception.get('sources',{}).get(name,{}).get('age',99)>.2:raise RuntimeError(name+'_not_fresh')
    if localization is None:raise RuntimeError('Set initial pose and verify AMCL first')
    covariance=np.array(localization.pose.covariance).reshape(6,6)
    if not np.isfinite(covariance).all() or min(covariance[0,0],covariance[1,1],covariance[5,5])<0 or max(covariance[0,0],covariance[1,1])>.0025 or covariance[5,5]>.0076:raise RuntimeError('localization_uncertain')
    tr=tf.lookup_transform('map','trial_odom',rclpy.time.Time());q=tr.transform.rotation;v=tr.transform.translation;mt=(np.array([v.x,v.y,v.z]),Rotation.from_quat([q.x,q.y,q.z,q.w]))
    stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
    if not -.3<=time.time()-stamp<.5:raise RuntimeError('localization_tf_stale')
    if map_transform and active and (np.linalg.norm(mt[0]-map_transform[0])>.1 or (mt[1]*map_transform[1].inv()).magnitude()>.1):raise RuntimeError('map_localization_jump')
    map_transform=mt
   except Exception as exc:reason=str(exc)
   if reason:
    stable=None;lease.unlink(missing_ok=True)
    if active is not None:raise RuntimeError(reason)
    if now-started>60:raise RuntimeError('Readiness timeout: '+reason)
   else:
    stable=stable or now
    # Stale Nav2 commands explicitly become zero; never replay the last nonzero command.
    target=velocity if now-last.get('command',0)<.2 and active and now-active>1. else (0.,0.,0.)
    data={'session':session,'boot_id':boot,'monotonic':now,'velocity':list(target)};velocity_command(data,now,session,boot)
    tmp=command.with_suffix('.tmp');tmp.write_text(json.dumps(data));tmp.replace(command);publish(lease,session,boot)
    if active is None and now-stable>=3:
     active=now
     if a.enable_motion:
      proc=subprocess.Popen(['python3',str(ROOT/'mapping_trial/guarded_remote.py'),'--seconds',str(a.seconds),'--session-id',session,'--motion-enable-file',str(lease),'--nav-command-file',str(command),'--log-dir',str(run/'control')],stdout=childlog,stderr=subprocess.STDOUT,start_new_session=True)
      print('导航门禁已就绪；摇杆回中，按START允许运动。双扳机退出，拨杆退出。',flush=True)
     else:print('只读门禁已就绪；不打开CAN，不使能电机。',flush=True)
   log.write(json.dumps({'monotonic':now,'ready':reason is None,'reason':reason,'active':active is not None})+'\n')
   if proc and proc.poll() is not None:break
 except (Exception,KeyboardInterrupt) as exc:error=repr(exc)
 finally:
  lease.unlink(missing_ok=True);command.unlink(missing_ok=True)
  if proc and proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=20)
   except subprocess.TimeoutExpired:error='Motor cleanup unconfirmed; use physical emergency stop'
  report={'error':error,'hardware_requested':a.enable_motion,'control_returncode':proc.returncode if proc else None,'run':str(run)};(run/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,ensure_ascii=False));log.close();childlog.close();n.destroy_node();rclpy.try_shutdown()
 return 1 if error else 0
if __name__=='__main__':raise SystemExit(main())
