"""Point transforms preserve intensity, tags, line and absolute per-point timestamps."""
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField

def transform_cloud(msg, extrinsic, sensor_id):
    data=point_cloud2.read_points(msg).reshape(-1)
    xyz=np.column_stack([data[n] for n in ('x','y','z')])
    valid=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>1.0)
    data=data[valid]
    rot=Rotation.from_euler('xyz',extrinsic['rpy_deg'],degrees=True).as_matrix()
    xyz=xyz[valid]@rot.T+np.asarray(extrinsic['xyz_m'])
    dtype=np.dtype([(n,data.dtype.fields[n][0]) for n in data.dtype.names]+[('sensor_id','u1')])
    out=np.empty(len(data),dtype=dtype)
    for n in data.dtype.names: out[n]=data[n]
    for i,n in enumerate(('x','y','z')): out[n]=xyz[:,i]
    out['sensor_id']=sensor_id
    fields=[PointField(name=f.name,offset=out.dtype.fields[f.name][1],datatype=f.datatype,count=f.count) for f in msg.fields]
    fields.append(PointField(name='sensor_id',offset=out.dtype.fields['sensor_id'][1],datatype=PointField.UINT8,count=1))
    return out,fields

def stamp_ns(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

def take_pair(queues,tolerance_ns):
    """Consume each monotonic frame once; discard frames too old for the other stream."""
    while queues[0] and queues[1]:
        dt=stamp_ns(queues[0][0])-stamp_ns(queues[1][0])
        if abs(dt)<=tolerance_ns: return queues[0].popleft(),queues[1].popleft()
        queues[0 if dt<0 else 1].popleft()
    return None

def ptp_ready(status,ips):
    return all(ip in status and status[ip].get('time_type')==1 and
               0<=status[ip].get('packet_age_ms',float('inf'))<500 for ip in ips)

def device_ptp_ready(status, seen, ips, now, max_offset_ns=500_000):
    """Bound sensor-reported PTP error; missing/stale data must fail closed."""
    for ip in ips:
        d=status.get(ip,{})
        if not 0<=now-seen.get(ip,float('-inf'))<3:
            return False,ip+':diagnostics_stale'
        if d.get('time_sync_type')!=1:
            return False,ip+':not_ptp'
        offset=d.get('time_offset')
        if not isinstance(offset,int) or isinstance(offset,bool) or abs(offset)>max_offset_ns:
            return False,ip+':offset_out_of_bounds'
        local,last=d.get('local_time_now'),d.get('last_sync_time')
        if not isinstance(local,int) or not isinstance(last,int) or not 0<=local-last<3_000_000_000:
            return False,ip+':last_sync_stale'
    return True,'ok'
