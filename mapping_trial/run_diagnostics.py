"""Read-only bounded diagnostic collection with exclusive driver ownership."""
import fcntl,json,os,pathlib,signal,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'calibration'))
from capture import preflight
preflight(json.loads((pathlib.Path(__file__).resolve().parents[1]/'calibration/config/settings.json').read_text()))
lock=pathlib.Path('/home/pc/lidar test/logs/driver.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
config=pathlib.Path(__file__).resolve().parent/'config'
cmd=[str(pathlib.Path(__file__).resolve().parents[1]/'sensors/driver_workspace/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'),'--ros-args','-r','__node:=reboot_diagnostic_only']
for k,v in {'user_config_path':str(config/'dual.json'),'xfer_format':0,'multi_topic':1,'data_src':0,'publish_freq':10.0,'output_data_type':0,'frame_id':'livox_frame','ptp_utc_offset_seconds':0}.items():cmd+=['-p',f'{k}:={v}']
for d in json.loads((config/'devices.json').read_text())['devices']:
    suffix=d['ip'].replace('.','_')
    cmd+=['-r',f'/livox/lidar_{suffix}:=/{d["name"]}/points','-r',f'/livox/imu_{suffix}:=/{d["name"]}/imu']
p=None
try:
    with (ROOT/'diagnostic_driver.log').open('a') as log:
        p=subprocess.Popen(['timeout','--signal=INT','--kill-after=10s','80s',*cmd],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        code=subprocess.call([sys.executable,str(ROOT/'observe_diagnostics.py')])
finally:
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL);p.wait();print('Driver forced exit',flush=True)
    print('driver_returncode',None if p is None else p.returncode,flush=True)
    lock.close()
raise SystemExit(code)
