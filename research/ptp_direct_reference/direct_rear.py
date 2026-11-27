"""Rear LiDAR direct-cable diagnosis, bounded, no CAN or motion control."""
import argparse,collections,datetime,fcntl,json,pathlib,signal,subprocess,sys,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Imu
from std_msgs.msg import String
ROOT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,'/home/pc/dual_lidar_calibration_20261125')
from capture import preflight
ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=int,default=600);a=ap.parse_args()
if not 60<=a.seconds<=600:ap.error('seconds must be 60..600')
settings=json.loads(pathlib.Path('/home/pc/dual_lidar_calibration_20261125/config/settings.json').read_text());settings['sensors']={'rear_right':settings['sensors']['rear_right']}
preflight(settings)
if subprocess.run(['ping','-I','enp85s0','-c','2','-W','1','192.168.1.132'],stdout=subprocess.DEVNULL).returncode==0:
 raise SystemExit('前左雷达仍可通过同一网口访问，拒绝把当前拓扑标记为后右单台直连。')
neighbors=json.loads(subprocess.check_output(['ip','-j','neigh','show','dev','enp85s0'],text=True))
if not any(x.get('dst')=='192.168.1.159' and x.get('lladdr')=='34:91:f0:e6:fe:29' for x in neighbors):raise SystemExit('后右IP/MAC核对失败')
lock=pathlib.Path('/home/pc/lidar test/logs/driver.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
run=ROOT/'runs'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S');run.mkdir(parents=True)
config=json.loads(pathlib.Path('/home/pc/fusion_mapping_trial_20261125/config/dual.json').read_text());config['lidar_configs']=[d for d in config['lidar_configs'] if d['ip']=='192.168.1.159']
(run/'rear_only.json').write_text(json.dumps(config,indent=2));(ROOT/'LAST_RUN.txt').write_text(str(run)+'\n')
cmd=['/home/pc/rk3588 link/navigation/driver_workspace/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node','--ros-args','-r','__node:=direct_rear_diagnostic']
for k,v in {'user_config_path':str(run/'rear_only.json'),'xfer_format':0,'multi_topic':1,'data_src':0,'publish_freq':10.0,'output_data_type':0,'frame_id':'livox_frame','ptp_utc_offset_seconds':0}.items():cmd+=['-p',f'{k}:={v}']
rows=[];states=collections.Counter();stamps=collections.defaultdict(list);running=True;p=None;node=None;cleanup={}
def stop(*_):
 global running
 running=False
for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,stop)
def diag(m):
 try:d=json.loads(m.data)
 except ValueError:return
 if d.get('ip')!='192.168.1.159':return
 rows.append({'monotonic':time.monotonic(),'data':d});info=d.get('info',{})
 if 'cur_work_state' in info:states[str(info['cur_work_state'])]+=1
 if 'time_offset' in info:
  brief={'monotonic':time.monotonic(),'offset_ns':info['time_offset'],'hms':[f'0x{x:08X}' for x in info.get('hms_code',[]) if x],'work_state':info.get('cur_work_state'),'ptp_type':info.get('time_sync_type')}
  (run/'latest.json').write_text(json.dumps(brief,indent=2));journal.write(json.dumps(brief)+'\n');journal.flush()
try:
 rclpy.init();node=rclpy.create_node('direct_rear_observer');subs=[node.create_subscription(String,'/livox/device_diagnostics',diag,100)]
 for kind,typ in [('lidar',PointCloud2),('imu',Imu)]:
  subs.append(node.create_subscription(typ,'/livox/'+kind+'_192_168_1_159',lambda m,k=kind:stamps[k].append(m.header.stamp.sec*10**9+m.header.stamp.nanosec),qos_profile_sensor_data))
 with (run/'driver.log').open('w') as log,(run/'timing.jsonl').open('w') as journal:
  p=subprocess.Popen(['timeout','--signal=INT','--kill-after=10s',str(a.seconds+20)+'s',*cmd],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  print('后右单雷达只读对照开始：'+str(run),flush=True);end=time.monotonic()+a.seconds
  while running and time.monotonic()<end:
   if p.poll() is not None:raise RuntimeError('driver exited early')
   rclpy.spin_once(node,timeout_sec=.05)
finally:
 if p and p.poll() is None:
  for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,signal.SIG_IGN)
  signal_sent=False
  try:signal_sent=True;__import__('os').killpg(p.pid,signal.SIGINT);p.wait(timeout=12)
  except subprocess.TimeoutExpired:__import__('os').killpg(p.pid,signal.SIGKILL);p.wait();cleanup['forced']=True
  cleanup['returncode']=p.returncode
 if node:node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 lock.close()
 (run/'diagnostics.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
 (run/'stamps_ns.json').write_text(json.dumps(stamps))
 report={'topology':'operator_confirmed_rear_direct_front_unreachable','work_states':dict(states),'topics':{k:{'count':len(v),'nonmonotonic':sum(y<=x for x,y in zip(v,v[1:])),'max_gap_ns':max([y-x for x,y in zip(v,v[1:])],default=None)} for k,v in stamps.items()},'diagnostics':len(rows),'cleanup':cleanup}
 (run/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
