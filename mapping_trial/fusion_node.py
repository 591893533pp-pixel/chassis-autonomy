#!/usr/bin/env python3
import collections, copy, json, pathlib, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2
from fusion_core import transform_cloud,stamp_ns,take_pair,ptp_ready,device_ptp_ready
from ptp_health import read_health
from self_filter import RobotSelfFilter
ROOT=pathlib.Path(__file__).resolve().parent
cloud_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)

class Fusion(Node):
    def __init__(self):
        super().__init__('dual_lidar_fusion')
        self.declare_parameter('extrinsics',str(ROOT/'config/extrinsics_active.json'))
        self.cfg=json.loads(pathlib.Path(self.get_parameter('extrinsics').value).read_text())
        filter_cfg=json.loads((ROOT/'config/self_filter.json').read_text())
        if filter_cfg.get('coordinate_revision') != self.cfg.get('coordinate_revision'):
            raise ValueError('Extrinsics/self-filter coordinate revision mismatch')
        self.self_filter=RobotSelfFilter(filter_cfg,self.cfg['frame'])
        self.self_removed=[0,0];self.self_removed_total=[0,0]
        self.devices=json.loads((ROOT/'config/devices.json').read_text())['devices']
        if json.loads((ROOT/'config/devices.json').read_text()).get('coordinate_revision') != self.cfg.get('coordinate_revision'):
            raise ValueError('Devices/extrinsics coordinate revision mismatch')
        self.names=[d['name'] for d in self.devices]
        self.ips=[d['ip'] for d in self.devices]
        for name in self.names:
            e=self.cfg['sensors'][name]
            if len(e['xyz_m'])!=3 or len(e['rpy_deg'])!=3 or not np.isfinite(e['xyz_m']+e['rpy_deg']).all():
                raise ValueError('Invalid extrinsics: '+name)
        self.queues=[collections.deque(maxlen=5),collections.deque(maxlen=5)]
        self.last=[0,0];self.sync={};self.sync_seen=0.;self.ready=False
        self.device_status={};self.device_seen={};self.device_reason='not_checked'
        self.count=0;self.dropped=0;self.pair_dt=None;self.last_pub=0.
        self.master_ok=False;self.master_reason="not_checked";self.master_health={};self.master_generation=None
        self.preview=not self.cfg.get('calibrated',False)
        topic='/livox/fused_preview' if self.preview else '/livox/fused_points'
        self.pub=self.create_publisher(PointCloud2,topic,cloud_qos)
        self.diag=self.create_publisher(String,'/livox/fusion_status',10)
        self.subs=[self.create_subscription(PointCloud2,f'/{n}/points',lambda m,i=i:self.receive(m,i),cloud_qos) for i,n in enumerate(self.names)]
        self.sub_sync=self.create_subscription(String,'/livox/time_sync',self.on_sync,10)
        self.sub_device=self.create_subscription(String,'/livox/device_diagnostics',self.on_device,10)
        self.tf=StaticTransformBroadcaster(self)
        transforms=[]
        for d in self.devices:
            e=self.cfg['sensors'][d['name']]
            t=TransformStamped();t.header.frame_id=self.cfg['frame'];t.header.stamp=self.get_clock().now().to_msg()
            t.child_frame_id='livox_'+d['ip'].replace('.','_')
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,e['xyz_m'])
            q=Rotation.from_euler('xyz',e['rpy_deg'],degrees=True).as_quat()
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
            transforms.append(t)
        # LIO estimates the primary IMU pose. Connect its frame to the provisional body.
        import yaml
        params=yaml.safe_load((ROOT/'config/lio.yaml').read_text())['/**']['ros__parameters']
        rear=self.cfg['sensors']['rear_right']
        body_lidar=np.eye(4);body_lidar[:3,:3]=Rotation.from_euler('xyz',rear['rpy_deg'],degrees=True).as_matrix();body_lidar[:3,3]=rear['xyz_m']
        imu_lidar=np.eye(4);imu_lidar[:3,:3]=np.array(params['mapping/extrinsic_R']).reshape(3,3);imu_lidar[:3,3]=params['mapping/extrinsic_T']
        imu_body=imu_lidar@np.linalg.inv(body_lidar)
        t=TransformStamped();t.header.frame_id=params['common/base_frame'];t.header.stamp=self.get_clock().now().to_msg();t.child_frame_id=self.cfg['frame']
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,imu_body[:3,3])
        q=Rotation.from_matrix(imu_body[:3,:3]).as_quat();t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
        transforms.append(t)
        self.tf.sendTransform(transforms)
        self.timer=self.create_timer(0.5,self.report)
        self.get_logger().info(f'Output {topic}; PTP required; CAD calibrated={not self.preview}; no motion deskew')
    def on_sync(self,msg):
        try:self.sync=json.loads(msg.data);self.sync_seen=time.monotonic()
        except (ValueError,TypeError):self.sync={}
        self.check_sync()
    def on_device(self,msg):
        try:
            d=json.loads(msg.data);ip=d['ip'];info=d['info']
            if ip not in self.ips or not isinstance(info,dict):return
            # Only complete timing diagnostics can refresh the precision watchdog.
            if all(k in info for k in ('time_offset','time_sync_type','local_time_now','last_sync_time')):
                self.device_status[ip]=info;self.device_seen[ip]=time.monotonic()
        except (ValueError,KeyError,TypeError):return
        self.check_sync()
    def check_sync(self):
        self.master_ok,self.master_reason,self.master_health=read_health()
        generation=self.master_health.get("generation")
        if generation != self.master_generation:
            for q in self.queues:q.clear()
            self.last=[0,0];self.master_generation=generation
        ready=self.master_ok and time.monotonic()-self.sync_seen<1.0 and ptp_ready(self.sync,self.ips)
        device_ok,self.device_reason=device_ptp_ready(self.device_status,self.device_seen,self.ips,
            time.monotonic(),self.cfg.get('max_device_offset_ns',500_000))
        # Static-only trial: no self-report anomaly grace and no unknown HMS pass-through.
        for device in self.devices:
            d=self.device_status.get(device['ip'],{})
            codes=d.get('hms_code')
            if (d.get('sn')!=device['serial'] or d.get('cur_work_state')!=1
                or d.get('lidar_diag_status')!=0 or not isinstance(codes,list)
                or any(c not in (0,0x04050002) for c in codes)):
                device_ok=False;self.device_reason=device['ip']+':identity_workstate_or_hms'
        if self.cfg.get('require_device_diagnostics',False):ready=ready and device_ok
        if ready!=self.ready:
            for q in self.queues:q.clear()
            self.last=[0,0]
            self.get_logger().info('PTP checks passed' if ready else 'PTP unavailable: fusion paused ('+self.master_reason+'; '+self.device_reason+')')
        self.ready=ready
        return ready
    def receive(self,msg,i):
        if not self.check_sync():return
        stamp=stamp_ns(msg);now=self.get_clock().now().nanoseconds
        # Epoch mismatch (e.g. uncorrected TAI/UTC) must not silently enter fusion.
        age=now-stamp
        if age < -50_000_000 or age>int(self.cfg['max_age_ms']*1e6):
            self.dropped+=1;return
        if msg.header.frame_id!='livox_'+self.ips[i].replace('.','_'):
            self.dropped+=1;return
        if stamp<=self.last[i]:
            for q in self.queues:q.clear()
            self.last=[0,0];self.dropped+=1;return
        self.last[i]=stamp;self.queues[i].append(msg)
        pair=take_pair(self.queues,int(self.cfg['max_pair_dt_ms']*1e6))
        if pair is None:return
        if any(now-stamp_ns(m)>int(self.cfg['max_age_ms']*1e6) for m in pair):
            self.dropped+=1;return
        arrays=[];fields=None
        for j,m in enumerate(pair):
            arr,f=transform_cloud(m,self.cfg['sensors'][self.names[j]],j)
            arr,removed=self.self_filter.filter(arr)
            self.self_removed[j]=removed;self.self_removed_total[j]+=removed
            arrays.append(arr);fields=f
        if arrays[0].dtype!=arrays[1].dtype:
            self.dropped+=1;return
        header=copy.deepcopy(pair[0].header);header.frame_id=self.cfg['frame']
        # Header uses first scan start; each point retains its absolute timestamp in ns.
        merged=point_cloud2.create_cloud(header,fields,np.concatenate(arrays))
        self.pub.publish(merged);self.count+=1;self.last_pub=time.monotonic()
        self.pair_dt=abs(stamp_ns(pair[0])-stamp_ns(pair[1]))/1e6
    def report(self):
        self.check_sync()
        age=time.monotonic()-self.last_pub if self.last_pub else None
        state='WAIT_PTP' if not self.ready else (('RELATIVE_CALIBRATED_PREVIEW' if self.cfg.get('relative_calibrated') else 'CAD_PREVIEW') if self.preview else 'FUSED')
        if self.ready and (age is None or age>1):state='WAIT_VALID_PAIR'
        d=dict(state=state,ptp_packets_verified=time.monotonic()-self.sync_seen<1.0 and ptp_ready(self.sync,self.ips),
               ptp_master_healthy=self.master_ok,ptp_master_reason=self.master_reason,
               ptp_master_generation=self.master_generation,calibrated=not self.preview,relative_calibrated=self.cfg.get("relative_calibrated",False),
               pairs=self.count,last_pair_dt_ms=self.pair_dt,last_output_age_s=age,rejected=self.dropped,
               motion_deskew=False)
        d.update(self_filter_enabled=self.self_filter.enabled,
            self_filter_removed_last=dict(zip(self.names,self.self_removed)),
            self_filter_removed_total=dict(zip(self.names,self.self_removed_total)))
        d.update(scope='STATIONARY_TRIAL_NOT_NAVIGATION',monotonic=time.monotonic(),device_ptp_reason=self.device_reason,
            device_offset_limit_ns=self.cfg.get('max_device_offset_ns',500_000),
            device_offset_ns={ip:v.get('time_offset') for ip,v in self.device_status.items()},
            device_hms_warnings={ip:[c for c in v.get('hms_code',[]) if c] for ip,v in self.device_status.items()})
        msg=String();msg.data=json.dumps(d);self.diag.publish(msg)
        target=ROOT/'logs/fusion_status.json';tmp=target.with_suffix('.tmp');tmp.write_text(json.dumps(d,indent=2));tmp.replace(target)

def main():
    rclpy.init();node=Fusion()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
