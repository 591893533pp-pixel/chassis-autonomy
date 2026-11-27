"""Stationary capture with bounded driver lifetime; no CAN or chassis operations."""
import collections, datetime, fcntl, importlib, json, os, pathlib, signal, socket, subprocess, sys, time, uuid
import numpy as np

def device_status_issues(role,info,expected_serial):
 issues=[]
 if info.get('sn')!=expected_serial:issues.append(role+':序列号不匹配')
 if info.get('cur_work_state')!=1:issues.append(role+':设备工作状态异常 '+str(info.get('cur_work_state')))
 if info.get('lidar_diag_status')!=0:issues.append(role+':设备诊断状态异常 '+str(info.get('lidar_diag_status')))
 codes=[c for c in info.get('hms_code',[1]) if c]
 if codes:issues.append(role+':HMS告警 '+','.join(f'0x{c:08X}' for c in codes))
 return issues

def ptp_health():
 p=pathlib.Path('/run/livox-ptp/health.json');d=json.loads(p.read_text())
 boot=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
 if d.get('boot_id')!=boot or not d.get('healthy') or d.get('port_state')!='MASTER' or not 0<=time.monotonic()-d.get('monotonic',0)<3 or d.get('utc_offset_applied_seconds')!=0:
  raise RuntimeError('NUC PTP 不健康、过期或UTC偏移非0；请先恢复时间同步')
 return d

def preflight(s):
 nav_path=pathlib.Path(s['navigation_root']).expanduser()
 if not nav_path.is_absolute():nav_path=pathlib.Path(__file__).resolve().parent/nav_path
 s['navigation_root']=str(nav_path.resolve())
 for name in ['rclpy','sensor_msgs_py.point_cloud2','numpy','scipy','matplotlib']:importlib.import_module(name)
 addresses=json.loads(subprocess.check_output(['ip','-j','-4','addr','show','dev',s['interface']],text=True))
 if not any(a.get('local')==s['host_ip'] for d in addresses for a in d.get('addr_info',[])):raise RuntimeError('雷达网口缺少 '+s['host_ip'])
 if pathlib.Path('/sys/class/net',s['interface'],'carrier').read_text().strip()!='1':raise RuntimeError('雷达网口无物理链路')
 ptp_health()
 conflicts=[]
 for proc in pathlib.Path('/proc').iterdir():
  if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
  try:args=(proc/'cmdline').read_bytes().split(b'\0');tokens=[a.decode(errors='replace') for a in args]
  except (OSError,PermissionError):continue
  # Match executable/path tokens, not ssh/shell command strings containing their names.
  names={pathlib.Path(x).name for x in tokens if x}
  if names&{'livox_ros_driver2_node','run_ground_check.py','run_reposition.py','remote_session.py','fusion_node.py','run_remote.py','can_bridge','robstride_can_bridge'}:conflicts.append((proc.name,tokens[:3]))
 if conflicts:raise RuntimeError('请先正常退出现有采集/遥控/融合进程：'+str(conflicts))
 nav=pathlib.Path(s['navigation_root'])
 if not (nav/'driver_workspace/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node').is_file():raise RuntimeError('雷达驱动不存在')
 for d in s['sensors'].values():
  if subprocess.run(['ping','-c','1','-W','1',d['ip']],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:raise RuntimeError('雷达不可达：'+d['ip'])
 # These sockets are used only as a preflight conflict check and immediately closed.
 for port in (56101,56201,56301,56401):
  with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
   try:sock.bind((s['host_ip'],port))
   except OSError as e:raise RuntimeError(f'UDP {port} 已占用：{e}')
 return True

def run(args,session,root):
 import rclpy
 from rclpy.qos import qos_profile_sensor_data
 from sensor_msgs.msg import PointCloud2,Imu
 from sensor_msgs_py import point_cloud2
 from std_msgs.msg import String
 from calibrate import read,write,signature
 s=read(root/'config/settings.json');preflight(s)
 driver=read(root/'config/driver.json')
 if {v['ip'] for v in driver['lidar_configs']}!={v['ip'] for v in s['sensors'].values()}:raise ValueError('driver.json与settings.json的雷达IP不一致')
 for device in driver['lidar_configs']:
  if any(device['extrinsic_parameter'].values()):raise ValueError('驱动内部外参必须为零，避免变换两次')
 for typ in ('Mid360s','MID360'):
  if any(v['host_ip']!=s['host_ip'] for v in driver[typ]['host_net_info']):raise ValueError('驱动host_ip与settings.json不一致')
 duration=float(s['capture_seconds']);warmup=float(s['warmup_seconds'])
 if not 20<=duration<=60 or not 3<=warmup<=30:raise ValueError('采集时长需20–60秒，预热3–30秒')
 lockpath=pathlib.Path('/home/pc/lidar test/logs/driver.lock');lockpath.parent.mkdir(parents=True,exist_ok=True)
 lock=lockpath.open('w')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:lock.close();raise RuntimeError('雷达驱动锁已占用')
 folder=session/args.label
 if folder.exists():lock.close();raise ValueError('采集目录已存在；换一个标签，避免覆盖数据：'+str(folder))
 folder.mkdir(parents=True)
 meta={'capture_id':str(uuid.uuid4()),'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
       'pose_note':args.pose_note,'sensors':s['sensors'],'settings_sha256':signature(root/'config/settings.json'),
       'health_ok':False,'capture_seconds':duration,'warmup_seconds':warmup,'errors':[],
       'stationarity_note':'User must keep chassis still; IMU and later point-cloud checks are supplementary.'}
 write(folder/'metadata.json',meta)
 errors=set();proc=None;node=None;initialized=False;log=None;completed=False;diag=[];sync_rows=[];clouds={};stats={};imus={};raw_counts=collections.Counter();last_diag={};last_sync={}
 for role in s['sensors']:
  clouds[role]=[];imus[role]=[]
  for kind in ['cloud','imu']:stats[(role,kind)]={'count':0,'backward':0,'max_gap_s':0.,'last_stamp':None,'min_age_s':1e10,'max_age_s':-1e10}
 def interrupted(*_):raise KeyboardInterrupt()
 oldterm=signal.signal(signal.SIGTERM,interrupted)
 oldhup=signal.signal(signal.SIGHUP,interrupted)
 try:
  log=(folder/'driver.log').open('w');nav=pathlib.Path(s['navigation_root'])
  proc=subprocess.Popen(['timeout','--signal=INT','--kill-after=10s',str(int(warmup+duration+20))+'s',str(nav/'driver_workspace/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'),'--ros-args','-r','__node:=dual_lidar_calibration','-p','user_config_path:='+str(root/'config/driver.json'),'-p','xfer_format:=0','-p','multi_topic:=1','-p','data_src:=0','-p','publish_freq:=10.0','-p','output_data_type:=0','-p','frame_id:=livox_frame','-p','ptp_utc_offset_seconds:=0'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  rclpy.init();initialized=True;node=rclpy.create_node('dual_lidar_calibration_capture')
  begin=time.monotonic()+warmup;end=begin+duration;subs=[]
  def active():return begin<=time.monotonic()<end
  def update(role,kind,msg):
   st=stats[(role,kind)];stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;age=time.time()-stamp
   st['count']+=1
   if st['last_stamp'] is not None:
    gap=stamp-st['last_stamp'];st['backward']+=int(gap<=0);st['max_gap_s']=max(st['max_gap_s'],gap)
   st['last_stamp']=stamp;st['min_age_s']=min(st['min_age_s'],age);st['max_age_s']=max(st['max_age_s'],age)
  def cloud_cb(role):
   def cb(m):
    if not active():return
    update(role,'cloud',m);raw_counts[role]+=1
    if raw_counts[role]%5:return
    d=point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True).reshape(-1)
    xyz=np.column_stack([d[k] for k in ('x','y','z')]).astype(np.float32);r=np.linalg.norm(xyz,axis=1)
    xyz=xyz[np.isfinite(xyz).all(axis=1)&(r>.3)&(r<15)]
    if len(xyz)<500:errors.add(role+':单帧有效点过少')
    clouds[role].append((time.monotonic()-begin,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,xyz))
   return cb
  def imu_cb(role):
   def cb(m):
    if not active():return
    update(role,'imu',m);imus[role].append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
   return cb
  def on_diag(m):
   try:
    d=json.loads(m.data);ip=d['ip'];info=d['info'];last_diag[ip]=(time.monotonic(),info)
    if active():diag.append(d)
   except (ValueError,KeyError,TypeError):errors.add('诊断格式错误')
  def on_sync(m):
   nonlocal last_sync
   try:
    last_sync=(time.monotonic(),json.loads(m.data))
    if active():sync_rows.append(last_sync[1])
   except ValueError:errors.add('同步数据格式错误')
  for role,d in s['sensors'].items():
   suffix=d['ip'].replace('.','_')
   subs.extend([node.create_subscription(PointCloud2,'/livox/lidar_'+suffix,cloud_cb(role),qos_profile_sensor_data),node.create_subscription(Imu,'/livox/imu_'+suffix,imu_cb(role),qos_profile_sensor_data)])
  subs.extend([node.create_subscription(String,'/livox/device_diagnostics',on_diag,10),node.create_subscription(String,'/livox/time_sync',on_sync,10)])
  print(f'预热 {warmup:g} 秒，随后静止采集 {duration:g} 秒。请勿移动底盘或遮挡雷达。',flush=True)
  next_check=begin;generation=ptp_health()['generation']
  while time.monotonic()<end:
   rclpy.spin_once(node,timeout_sec=.05)
   if proc.poll() is not None:raise RuntimeError('雷达驱动提前退出，请查看driver.log')
   if time.monotonic()>=next_check:
    now=time.monotonic();next_check=now+.5
    try:
     if ptp_health()['generation']!=generation:errors.add('采集中PTP主钟重启')
    except Exception as e:errors.add(str(e))
    for role,d in s['sensors'].items():
     stamp,info=last_diag.get(d['ip'],(0,{}))
     if now-stamp>=3:errors.add(role+':诊断缺失/过期');continue
     errors.update(device_status_issues(role,info,d['serial']))
     offset=info.get('time_offset');local=info.get('local_time_now',0);last=info.get('last_sync_time',0)
     if info.get('time_sync_type')!=1 or not isinstance(offset,int) or abs(offset)>s['max_device_offset_ns'] or not 0<=local-last<3_000_000_000:errors.add(role+':设备PTP不合格')
     if not last_sync or now-last_sync[0]>1:errors.add(role+':包同步信息过期')
     else:
      v=last_sync[1].get(d['ip'],{})
      if v.get('time_type')!=1 or not 0<=v.get('packet_age_ms',1e9)<500:errors.add(role+':数据包PTP缺失/过期')
  completed=True
 except BaseException as exc:
  errors.add(type(exc).__name__+': '+str(exc))
 finally:
  if proc is not None and proc.poll() is None:
   try:os.killpg(proc.pid,signal.SIGINT)
   except ProcessLookupError:pass
   try:proc.wait(timeout=10)
   except subprocess.TimeoutExpired:
    errors.add('驱动未正常退出，已强制结束本次进程组');os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
  if node is not None:node.destroy_node()
  if initialized and rclpy.ok():rclpy.shutdown()
  if log is not None:log.close()
  signal.signal(signal.SIGTERM,oldterm);signal.signal(signal.SIGHUP,oldhup);lock.close()
 for role in s['sensors']:
  rows=[d['info'] for d in diag if d.get('ip')==s['sensors'][role]['ip']]
  if len(rows)<duration*.6:errors.add(role+':设备诊断样本不足')
  for info in rows:
   off=info.get('time_offset')
   errors.update(device_status_issues(role,info,s['sensors'][role]['serial']))
   if info.get('time_sync_type')!=1 or not isinstance(off,int) or abs(off)>s['max_device_offset_ns']:errors.add(role+':采集中曾出现PTP异常')
  samples=clouds[role];first=[v[2] for v in samples if v[0]<duration/2];second=[v[2] for v in samples if v[0]>=duration/2]
  if first and second:np.savez_compressed(folder/(role+'.npz'),first=np.concatenate(first),second=np.concatenate(second),stamps=np.array([v[1] for v in samples]))
  else:errors.add(role+':前半段/后半段点云缺失')
  a=np.asarray(imus[role]);np.save(folder/(role+'_imu.npy'),a)
  if a.size:
   gyro95=float(np.percentile(np.linalg.norm(a[:,:3],axis=1),95));acc_std=float(np.linalg.norm(np.std(a[:,3:],axis=0)))
   meta.setdefault('stationarity',{})[role]={'gyro_norm_p95':gyro95,'acceleration_std_norm':acc_std}
   # Driver reports gyro rad/s and acceleration g. Constant translation still needs visual/scene checks.
   if not np.isfinite(a).all() or gyro95>.04 or acc_std>.025:errors.add(role+':IMU静止检查不通过')
  for kind,min_hz,max_gap in [('cloud',8,.3),('imu',150,.1)]:
   st=stats[(role,kind)];st['hz_over_window']=st['count']/duration
   if st['count']==0:st.update(min_age_s=None,max_age_s=None)
   if st['hz_over_window']<min_hz or st['backward'] or st['max_gap_s']>max_gap:errors.add(role+':'+kind+'频率/时间戳不合格')
   if st['count'] and (st['min_age_s']<-.05 or st['max_age_s']>.5):errors.add(role+':'+kind+'时间戳不在当前时钟范围')
 if not completed:errors.add('采集未完成')
 if proc is not None and proc.returncode not in (0,-signal.SIGINT,130):errors.add('驱动退出码异常:'+str(proc.returncode))
 if not (folder/'driver.log').exists() or 'Deinit completely' not in (folder/'driver.log').read_text(errors='replace'):errors.add('未确认SDK正常清理')
 meta.update(health_ok=not errors,errors=sorted(errors),stats={role:{kind:stats[(role,kind)] for kind in ('cloud','imu')} for role in s['sensors']})
 diagnostics={'diagnostics':diag,'sync':sync_rows}
 if not meta['health_ok']:
  from static_policy import qualify_static
  meta['static_review']=qualify_static(meta,diagnostics,s)
 write(folder/'diagnostics.json',diagnostics);write(folder/'metadata.json',meta)
 eligible=meta['health_ok'] or meta.get('static_review',{}).get('eligible',False)
 title='采集通过：' if meta['health_ok'] else ('静止标定采集通过（保留历史同步告警）：' if eligible else '采集未通过：')
 print(title,folder,flush=True)
 for e in sorted(errors):print('  -',e)
 return 0 if eligible else 2
