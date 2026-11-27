"""Passive standard-CAN capture around the existing manual lift remote; no CAN writes here."""
from pathlib import Path
import datetime,json,socket,struct,subprocess,threading,time,signal,hashlib
ROOT=Path(__file__).resolve().parents[1]
folder=ROOT/'logs'/('lift-halt-capture-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
folder.mkdir(parents=True);(folder/'capture_used.py').write_bytes(Path(__file__).read_bytes())
stop=threading.Event();errors=[];proc=None
sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
# Only standard data frames; include SDO, NMT, heartbeat and EMCY. Passive receive only.
sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,struct.pack('=II',0,0xC0000000))
sock.bind(('can0',));sock.settimeout(.1)
def capture():
 try:
  with (folder/'standard_can.jsonl').open('w',buffering=1) as f:
   while not stop.is_set():
    try:packet=sock.recv(16)
    except socket.timeout:continue
    c,l,b=struct.unpack('=IB3x8s',packet)
    f.write(json.dumps({'monotonic_s':time.monotonic(),'can_id':hex(c),'dlc':l,'data':b[:l].hex()})+'\n')
 except Exception as e:errors.append(repr(e))
def interrupted(*args):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupted)
thread=threading.Thread(target=capture);thread.start()
cmd=['python3',str(ROOT/'program/remote/run_remote.py'),'--device','/dev/input/js0','--hardware','--can-interface','can0','--lift','--lift-zero-verified','--max-linear-speed','0.05']
meta={'started':datetime.datetime.now().astimezone().isoformat(),'command':cmd,'capture':'standard CAN passive receive; timestamps are user-space monotonic','timeout_s':300,'reason':'manual Y press/release HALT diagnosis'}
print('CAPTURE',folder,flush=True)
try:
 if errors:raise RuntimeError(str(errors))
 proc=subprocess.Popen(cmd,start_new_session=True)
 deadline=time.monotonic()+300
 while proc.poll() is None:
  if errors:raise RuntimeError(str(errors))
  if time.monotonic()>deadline:raise TimeoutError('manual session five-minute limit')
  time.sleep(.1)
 meta['remote_exit_code']=proc.returncode
except (KeyboardInterrupt,Exception) as e:meta['interruption']=repr(e)
finally:
 if proc is not None and proc.poll() is None:
  proc.send_signal(signal.SIGTERM)
  try:proc.wait(timeout=10)
  except subprocess.TimeoutExpired:meta['cleanup_error']='remote has not exited after SIGTERM; inspect process; no SIGKILL sent'
 time.sleep(.3);stop.set();thread.join(timeout=2);sock.close()
 meta['capture_errors']=errors;meta['ended']=datetime.datetime.now().astimezone().isoformat()
 latest=ROOT/'logs/remote-latest'
 if latest.exists():meta['remote_log_directory']=str(latest.resolve())
 (folder/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2))
 (folder/'sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()},indent=2))
 print('CAPTURE_SAVED',folder,flush=True)
