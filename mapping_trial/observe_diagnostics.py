"""Bounded ROS subscriber only; saves full device states, no CAN or SDK commands."""
import collections,datetime,json,pathlib,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2,Imu
rclpy.init();node=rclpy.create_node('reboot_lidar_diagnostic_observer')
rows=[];counts=collections.Counter();subs=[];stamps=collections.defaultdict(list)
def record(m):
    try:rows.append({'monotonic':time.monotonic(),'data':json.loads(m.data)})
    except ValueError:pass
subs.append(node.create_subscription(String,'/livox/device_diagnostics',record,100))
def count(m,t):
    counts.update([t]);stamps[t].append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
for role in ['front_left','rear_right']:
    for suffix,typ in [('points',PointCloud2),('imu',Imu)]:
        topic='/'+role+'/'+suffix
        subs.append(node.create_subscription(typ,topic,lambda m,t=topic:count(m,t),qos_profile_sensor_data))
end=time.monotonic()+60
try:
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
folder=pathlib.Path(__file__).resolve().parent/'observations';folder.mkdir(exist_ok=True)
output=folder/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
output.write_text(json.dumps({'counts':counts,'stamps_ns':stamps,'diagnostics':rows},ensure_ascii=False,indent=2))
last={r['data']['ip']:r['data'] for r in rows if 'ip' in r['data']}
print(json.dumps({'file':str(output),'counts':counts,'last':last,'nonmonotonic':{k:sum(b<=a for a,b in zip(v,v[1:])) for k,v in stamps.items()}},ensure_ascii=False,indent=2))
