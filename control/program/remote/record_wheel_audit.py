"""Record finite 10 s suspended-wheel tests and passive CAN TX/RX evidence."""
import argparse,csv,datetime,json,socket,struct,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def run(folder,signed,hardware):
 folder.mkdir();sock=None;child=None;code=None
 meta={'signed_id':signed,'selected_id':abs(signed),'started':datetime.datetime.now().astimezone().isoformat(),'duration_s':10,'ramp_s':.5,'zero_tail_s':.5,'speed_rad_s':0 if signed==0 else (.2 if signed>0 else -.2),'hardware':hardware,'clock':'CLOCK_MONOTONIC, userspace receive timestamp','passive_capture':'all classic CAN frames observed on independent socket; local TX observation is not motor acknowledgement'}
 (folder/'metadata.json').write_text(json.dumps(meta,indent=2))
 try:
  if hardware:
   sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);sock.bind(('can0',));sock.settimeout(.05)
  with (folder/'bridge.log').open('w') as log,(folder/'can_frames.csv').open('w') as raw:
   w=csv.writer(raw);w.writerow(['monotonic_s','can_id_hex','dlc','data_hex'])
   args=[str(ROOT/'build/single_wheel'),str(signed),str(folder/'feedback.csv')]
   if hardware:args+=['--hardware','can0']
   child=subprocess.Popen(args,stdout=log,stderr=log)
   deadline=time.monotonic()+25
   while child.poll() is None:
    if time.monotonic()>deadline:raise RuntimeError('test exceeded 25 s wall deadline')
    if sock:
     try:data=sock.recv(16)
     except socket.timeout:continue
     cid,dlc,payload=struct.unpack('=IB3x8s',data);w.writerow([f'{time.monotonic():.9f}',hex(cid),dlc,payload.hex()])
    else:time.sleep(.02)
   code=child.returncode
 except BaseException as e:
  meta['exception']=repr(e);raise
 finally:
  if child is not None and child.poll() is None:
   child.terminate()
   try:child.wait(timeout=15)
   except subprocess.TimeoutExpired:meta['cleanup_unconfirmed']=True
  if sock:sock.close()
  meta['exit_code']=None if child is None else child.returncode;meta['ended']=datetime.datetime.now().astimezone().isoformat()
  (folder/'metadata.json').write_text(json.dumps(meta,indent=2))
 if code:raise RuntimeError(f'Test failed; no subsequent tests: {folder}')
def main():
 p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--hardware',action='store_true');p.add_argument('--ids',type=int,nargs='+',default=[0,1,-1,2,-2,3,-3,4,-4]);a=p.parse_args()
 if any(abs(i)>4 for i in a.ids):p.error('IDs must be -4..4')
 for i in a.ids:
  label='zero' if i==0 else f'id{abs(i)}_'+('positive' if i>0 else 'negative')
  print(f'START {label}: 10 seconds',flush=True);run(a.output/label,i,a.hardware);print(f'STOPPED {label}',flush=True);time.sleep(.5)
if __name__=='__main__':main()
