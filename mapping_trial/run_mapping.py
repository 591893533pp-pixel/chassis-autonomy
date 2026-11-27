#!/usr/bin/env python3
"""Supervised manual mapping trial. Default control locked; --motion needs operator START."""
import argparse,collections,datetime,fcntl,hashlib,json,os,pathlib,shutil,signal,subprocess,sys,time
import uuid
from live_gate import read as read_raw
from motion_lease import publish
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
ROOT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'calibration'))
from capture import preflight

def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=int,default=30);ap.add_argument('--motion',action='store_true');a=ap.parse_args()
 if not 30<=a.seconds<=900:raise ValueError('seconds must be 30..900')
 preflight(json.loads((pathlib.Path(__file__).resolve().parents[1]/'calibration/config/settings.json').read_text()))
 conflicts=[]
 for p in pathlib.Path('/proc').iterdir():
  if not p.name.isdigit() or int(p.name)==os.getpid():continue
  try:tokens=(p/'cmdline').read_bytes().split(b'\0');names={pathlib.Path(x.decode(errors='replace')).name for x in tokens if x}
  except OSError:continue
  if names&{'fastlio_mapping','cloud_adapter.py','run_static.py','run_mapping.py','guarded_remote.py','remote_bridge'}:conflicts.append(p.name)
 if conflicts:raise RuntimeError('Conflicting mapping trial: '+str(conflicts))
 dev=json.loads((ROOT/'config/devices.json').read_text())
 neighbors=json.loads(subprocess.check_output(['ip','-j','neigh','show','dev',dev['interface']],text=True))
 for d in dev['devices']:
  if not any(v.get('dst')==d['ip'] and v.get('lladdr','').lower()==d['mac'] for v in neighbors):raise RuntimeError('IP/MAC mismatch or absent: '+d['ip'])
 lock=pathlib.Path('/home/pc/lidar test/logs/driver.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 run=ROOT/'runs'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S');run.mkdir(parents=True,exist_ok=False)
 shutil.copytree(ROOT/'config',run/'config');shutil.copy2(__file__,run/'run_mapping.py');(ROOT/'LAST_RUN.txt').write_text(str(run)+'\n')
 statuspath=ROOT/'logs/fusion_status.json'
 if statuspath.exists():statuspath.unlink()
 (ROOT/'STOP').unlink(missing_ok=True)
 save(ROOT/'STATE.json',{'running':True,'pid':os.getpid(),'run':str(run),'scope':'MANUAL_MAPPING_TRIAL' if a.motion else 'MAPPING_LOCKED_CONTROL_CHECK'})
 session=str(uuid.uuid4());boot=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip();permit=run/'motion_lease.json';control=None;control_started=None;raw_generation=None;raw_samples=[];map_samples=[];world_count=0;stop_reason=None
 children=[];logs=[];cleanup=[];failure=None;poses=[];clouds=[];fusion=[];imu=collections.defaultdict(list);last={};world=None;sample=None;faults=[];started=time.monotonic();phase='startup';previous_stamp={}
 def launch(name,cmd):
  f=(run/(name+'.log')).open('w');logs.append(f)
  p=subprocess.Popen(['timeout','--signal=INT','--kill-after=10s',str(a.seconds+120)+'s',*cmd],stdout=f,stderr=subprocess.STDOUT,start_new_session=True,cwd=run)
  children.append((name,p));return p
 def interrupt(*_):raise KeyboardInterrupt()
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
 rclpy.init();node=rclpy.create_node('stationary_mapping_integration_audit');tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,node);tf_checks=0
 def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
 def receive_pose(m):
  last['odom']=time.monotonic();v=m.pose.pose.position;q=m.pose.pose.orientation
  row=[stamp(m)*1e-9,v.x,v.y,v.z,q.x,q.y,q.z,q.w]
  if not np.isfinite(row).all() or not .95<=np.linalg.norm(row[4:])<=1.05:faults.append('odom_invalid')
  if poses and row[0]<=poses[-1][0]:faults.append('odom_time_nonmonotonic')
  if control is not None and poses:
   dt=row[0]-poses[-1][0]
   if np.linalg.norm(np.array(row[1:4])-poses[-1][1:4])>.05+.3*max(0,dt):faults.append('odom_position_jump')
   if np.isfinite(row).all() and np.linalg.norm(row[4:])>.1:
    angle=(Rotation.from_quat(row[4:])*Rotation.from_quat(poses[-1][4:]).inv()).magnitude()
    if angle>.1+.4*max(0,dt):faults.append('odom_rotation_jump')
  poses.append(row)
 def receive_cloud(m):
  nonlocal sample
  arr=point_cloud2.read_points(m).reshape(-1);ts=arr['timestamp'];ns=stamp(m);last['cloud']=time.monotonic()
  row={'stamp_ns':ns,'count':len(arr),'sensors':np.unique(arr['sensor_id']).tolist(),'span_ms':float(np.ptp(ts))*1e-6,'sorted':bool(np.all(np.diff(ts)>=0)),'first_delta_ns':float(ts[0]-ns),'frame':m.header.frame_id}
  clouds.append(row)
  if row['sensors']!=[0,1] or not row['sorted'] or row['span_ms']>150 or abs(row['first_delta_ns'])>512 or row['frame']!='livox_192_168_1_159':faults.append('adapted_cloud_invalid')
  if ns<=previous_stamp.get('cloud',0):faults.append('cloud_time_nonmonotonic')
  previous_stamp['cloud']=ns
  if sample is None:sample=arr.copy()
 def receive_imu(m,role):
  last[role]=time.monotonic();acc=m.linear_acceleration;g=m.angular_velocity;ns=stamp(m)
  imu[role].append([ns*1e-9,g.x,g.y,g.z,acc.x,acc.y,acc.z])
  if ns<=previous_stamp.get(role,0):faults.append(role+':imu_time_nonmonotonic')
  if control is not None and ns-previous_stamp.get(role,ns)>15_000_000:faults.append(role+':ROS_imu_gap')
  if control is not None and not -5_000_000<=time.time_ns()-ns<=200_000_000:faults.append(role+':ROS_imu_stamp_stale')
  previous_stamp[role]=ns
 def receive_world(m):
  nonlocal world,world_count
  p=point_cloud2.read_points(m,field_names=('x','y','z')).reshape(-1);world=np.column_stack([p[k] for k in ('x','y','z')]);last['world']=time.monotonic();world_count+=1
  if world_count%10==0:map_samples.append(world[::max(1,len(world)//5000)].astype(np.float32).copy())
 subs=[node.create_subscription(Odometry,'/Odometry',receive_pose,100),node.create_subscription(PointCloud2,'/slam/points',receive_cloud,qos_profile_sensor_data),node.create_subscription(PointCloud2,'/cloud_registered',receive_world,qos_profile_sensor_data)]
 # Preserve 200 Hz IMU samples across brief point-cloud callback stalls.
 # Header-age and raw-stream gates still reject stale data and actual gaps.
 imu_qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
 for role in ('rear_right','front_left'):subs.append(node.create_subscription(Imu,'/'+role+'/imu',lambda m,r=role:receive_imu(m,r),imu_qos))
 stable=None;map_started=None;next_check=0;ptp_generation=None
 try:
  launch('fusion',['python3',str(ROOT/'fusion_node.py')]);launch('adapter',['python3',str(ROOT/'cloud_adapter.py')])
  cmd=[str(pathlib.Path(__file__).resolve().parents[1]/'sensors/driver_workspace/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'),'--ros-args','-r','__node:=dual_lidar_static_trial']
  for k,v in {'user_config_path':str(ROOT/'config/dual.json'),'xfer_format':0,'multi_topic':1,'data_src':0,'publish_freq':10.0,'output_data_type':0,'frame_id':'livox_frame','ptp_utc_offset_seconds':0}.items():cmd+=['-p',f'{k}:={v}']
  for d in dev['devices']:
   suffix=d['ip'].replace('.','_')
   cmd+=['-r',f'/livox/lidar_{suffix}:=/{d["name"]}/points','-r',f'/livox/imu_{suffix}:=/{d["name"]}/imu']
  launch('driver',cmd)
  print('融合和原始时间预检；初始化时保持静止，控制器默认锁定，只有--motion才允许手动START。',flush=True)
  while True:
   rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
   if (ROOT/'STOP').exists():raise RuntimeError('Operator requested stop')
   if faults:raise RuntimeError(faults[0])
   if control is not None and control.poll() is not None:
    text=(run/'control.log').read_text();stop_reason='operator_dual_trigger' if 'dual trigger stop' in text else 'control_exited'
    if control.returncode!=0 and stop_reason!='operator_dual_trigger':raise RuntimeError('Control exited: '+text[-1000:])
    break
   for name,p in children:
    if p.poll() is not None:raise RuntimeError(name+' exited early: '+str(p.returncode))
   if now<next_check:continue
   next_check=now+.05
   try:raw=read_raw(raw_generation);raw_ok=True
   except RuntimeError as e:raw={'error':str(e)};raw_ok=False
   if len(raw_samples)==0 or now-raw_samples[-1]['monotonic']>=1:raw_samples.append({'monotonic':now,'status':raw})
   if raw_ok and raw_generation is None:raw_generation=raw['generation']
   try:f=json.loads(statuspath.read_text())
   except (OSError,ValueError):f={}
   ok=(raw_ok and 0<=now-f.get('monotonic',0)<1.5 and f.get('ptp_master_healthy') and f.get('device_ptp_reason')=='ok' and f.get('last_output_age_s') is not None and f['last_output_age_s']<.7 and now-last.get('cloud',0)<.7)
   fusion.append({'monotonic':now,'phase':phase,'ok':bool(ok),'fusion':f})
   if map_started is None:
    stable=stable if ok and stable else (now if ok else None)
    if now-started>60:raise RuntimeError('Startup timeout: '+str(f)+' raw='+str(raw))
    if stable and now-stable>=10:
     ptp_generation=f.get('ptp_master_generation')
     launch('bag',['ros2','bag','record','-o',str(run/'bag'),'/front_left/points','/rear_right/points','/livox/device_diagnostics','/slam/points','/rear_right/imu','/front_left/imu','/Odometry','/cloud_registered','/livox/fusion_status','/tf','/tf_static','/lio_sam/mapping/cloud_global','/lio_sam/mapping/path','/lio_sam/loop_closure_constraints'])
     launch('fastlio',[str(ROOT.parent/'mapping_ws/install/lib/fast_lio_sam/fastlio_mapping'),'--ros-args','--params-file',str(ROOT/'config/lio.yaml'),'-p','result_save/output_dir:='+str(run/'optimized')])
     map_started=now;phase='static_lio';print('FAST-LIO已启动，保持静止约25秒，等待遥控入口就绪。',flush=True)
   else:
    if not ok:raise RuntimeError('Health became invalid: '+str(f)+' raw='+str(raw))
    if shutil.disk_usage(run).free<5*1024**3:raise RuntimeError('Disk free below 5GiB')
    if f.get('ptp_master_generation')!=ptp_generation:raise RuntimeError('PTP master restarted')
    for role in ('rear_right','front_left'):
     if now-last.get(role,0)>.2:raise RuntimeError(role+' IMU stale')
     rows=np.array(imu[role][-200:])
     if (control is None or not a.motion) and len(rows)==200 and (np.percentile(np.linalg.norm(rows[:,1:4],axis=1),95)>.04 or np.linalg.norm(np.std(rows[:,4:],axis=0))>.025):raise RuntimeError(role+' not static or IMU unstable')
    if now-map_started>15 and (now-last.get('odom',0)>1 or now-last.get('world',0)>1):raise RuntimeError('LIO odometry/cloud missing or stale')
    if now-map_started>15:
     for frame in ['base_link_trial','livox_192_168_1_159','livox_192_168_1_132']:
      if not tf_buffer.can_transform('trial_map',frame,rclpy.time.Time()):raise RuntimeError('TF chain unavailable: '+frame)
     tf_checks+=1
    if control is None and now-map_started>=25 and len(poses)>=100:
     args=['python3',str(ROOT/'guarded_remote.py'),'--seconds',str(a.seconds),'--session-id',session,'--motion-enable-file',str(permit),'--log-dir',str(run/'control')]
     if not a.motion:args.append('--lock-motion')
     publish(permit,session,boot);control=launch('control',args);control_started=now
     print(('运动窗口已开放：摇杆回中，按START后才可低速遥控；双扳机退出。' if a.motion else '控制器已启动但运动锁定，按START也不使能。')+' 窗口'+str(a.seconds)+'秒。',flush=True)
    if control is not None:publish(permit,session,boot)
    if control_started and now-control_started>=a.seconds-1:stop_reason='time_limit';break
    if control is None and now-map_started>40:raise RuntimeError('Control readiness timed out')
 except BaseException as exc:failure=type(exc).__name__+': '+str(exc)
 finally:
  permit.unlink(missing_ok=True)
  for name,p in reversed(children):
   forced=False
   if p.poll() is None:
    try:os.killpg(p.pid,signal.SIGINT)
    except ProcessLookupError:pass
    try:p.wait(timeout=60 if name=='fastlio' else 12)
    except subprocess.TimeoutExpired:
     forced=True;os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
   cleanup.append({'name':name,'returncode':p.returncode,'forced':forced})
  for f in logs:f.close()
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
  lock.close()
 report={'scope':'MANUAL_MAPPING_TRIAL' if a.motion else 'MAPPING_LOCKED_CONTROL_CHECK','failure':failure,'requested_lio_seconds':a.seconds,'elapsed_s':time.monotonic()-started,'odom_messages':len(poses),'adapted_clouds':len(clouds),'cleanup':cleanup,'body_anchor_verified':False,'ground_issue_resolved':False,'controller_started':control is not None,'motion_allowed_by_option':a.motion,'operator_automatic_enable':False,'stop_reason':stop_reason,'nearfield_filter':'1m_per_sensor_excluded_mapping_experiment_only_not_obstacle_map','configuration_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (run/'config').iterdir()}}
 if poses and not a.motion:
  p=np.array(poses);steady=p[p[:,0]>=p[0,0]+20];report['odom_span_s']=float(p[-1,0]-p[0,0])
  if len(steady):
   report['after_20s_max_position_change_m']=float(np.linalg.norm(steady[:,1:4]-steady[0,1:4],axis=1).max());rot=Rotation.from_quat(steady[:,4:]);report['after_20s_max_angle_change_deg']=float(np.degrees((rot*rot[0].inv()).magnitude()).max())
 if clouds:report['cloud_summary']={'max_scan_span_ms':max(c['span_ms'] for c in clouds),'both_sensors_all':all(c['sensors']==[0,1] for c in clouds),'all_sorted':all(c['sorted'] for c in clouds),'min_points':min(c['count'] for c in clouds)}
 report['backend']='FAST-LIO-SAM; exports in optimized/KEY_FRAMES; accumulated_map.pcd is a diagnostic uncorrected overlay'
 report['tf_chain_checks']=tf_checks
 report['cleanup_expected']=all(c['returncode'] in ((0,2) if c['name']=='bag' else ((0,1) if c['name']=='control' and stop_reason=='operator_dual_trigger' else (0,))) and not c['forced'] for c in cleanup)
 report['pipeline_observed']=report['cleanup_expected'] and tf_checks>0 and not failure and bool(poses) and bool(clouds) and world is not None and not any(c['forced'] for c in cleanup)
 if control is not None:
  bridge=(run/'control/bridge.log').read_text() if (run/'control/bridge.log').exists() else ''
  report['control_cleanup_confirmed']='Bridge exited' in bridge and ' FAILED' not in bridge
  report['pipeline_observed']=report['pipeline_observed'] and report['control_cleanup_confirmed']
 sys.path.insert(0,str(ROOT.parent/'scripts'))
 from verify_map_export import verify
 try:report['optimized_export']=verify(run/'optimized')
 except Exception as exc:
  report['optimized_export']={'export_valid':False,'error':str(exc)}
  report['pipeline_observed']=False
 for name,value in [('raw_health',raw_samples),('report',report),('poses',poses),('clouds',clouds),('fusion_health',fusion)]:save(run/(name+'.json'),value)
 for role,rows in imu.items():np.save(run/(role+'_imu.npy'),np.array(rows))
 if sample is not None:np.savez_compressed(run/'adapted_sample.npz',points=sample)
 if map_samples:
  combined=np.concatenate(map_samples);combined=combined[np.isfinite(combined).all(1)]
  _,indices=np.unique(np.floor(combined/.1).astype(np.int64),axis=0,return_index=True);combined=combined[indices]
  np.savez_compressed(run/'accumulated_map.npz',xyz=combined)
  with (run/'accumulated_map.pcd').open('w') as f:
   f.write('# Sampled accumulated registered scans; no loop closure; not a navigation map\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH '+str(len(combined))+'\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS '+str(len(combined))+'\nDATA ascii\n');np.savetxt(f,combined,fmt='%.5f')
 if world is not None:
  world=world[np.isfinite(world).all(1)];np.savez_compressed(run/'registered_cloud.npz',xyz=world)
  with (run/'last_scan_preview.pcd').open('w') as f:
   f.write('# Last registered scan only; not navigation map\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH '+str(len(world))+'\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS '+str(len(world))+'\nDATA ascii\n');np.savetxt(f,world,fmt='%.5f')
 save(ROOT/'STATE.json',{'running':False,'run':str(run),'failure':failure,'pipeline_observed':report['pipeline_observed']})
 print(json.dumps(report,ensure_ascii=False,indent=2));print('输出目录：'+str(run),flush=True)
 return 0 if report['pipeline_observed'] else 2
if __name__=='__main__':raise SystemExit(main())
