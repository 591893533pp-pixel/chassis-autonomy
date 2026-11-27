import socket,struct,select,time,json,fcntl,os,datetime
from pathlib import Path
root=Path(__file__).resolve().parents[1]
lock=open(f'/tmp/poke-remote-{os.getuid()}.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
p=root/'logs'/('lift-status-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));p.mkdir(parents=True)
s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);s.bind(('can0',));s.setblocking(False)
raw=[];results={}
try:
 for idx in [0x6064,0x6041,0x603f,0x6040,0x6061,0x606c,0x607a,0x60fd,0x605a,0x6083,0x6084,0x6085]:
  d=bytes([0x40,idx&255,idx>>8,0,0,0,0,0]);s.send(struct.pack('=IB3x8s',0x601,8,d));raw.append({'t':time.monotonic(),'direction':'tx','id':'0x601','data':d.hex()});end=time.monotonic()+1
  rec={'error':'timeout'}
  while time.monotonic()<end:
   if not select.select([s],[],[],max(0,end-time.monotonic()))[0]:break
   c,l,b=struct.unpack('=IB3x8s',s.recv(16));raw.append({'t':time.monotonic(),'direction':'rx','id':hex(c),'data':b.hex()})
   if c!=0x581 or l!=8 or b[1:4]!=d[1:4]:continue
   if b[0]==0x80:rec={'abort':hex(int.from_bytes(b[4:],'little'))};break
   if b[0] not in [0x43,0x47,0x4b,0x4f]:rec={'error':'unsupported response','raw':b.hex()};break
   n=4-((b[0]>>2)&3);rec={'value':int.from_bytes(b[4:4+n],'little',signed=idx in [0x6064,0x606c,0x607a,0x6061]),'raw':b.hex()};break
  results[hex(idx)]=rec
finally:
 s.close();(p/'raw_can.json').write_text(json.dumps(raw,indent=2));(p/'readings.json').write_text(json.dumps(results,indent=2));(p/'read_status_used.py').write_bytes(Path(__file__).read_bytes());print(p);print(json.dumps(results,indent=2))
