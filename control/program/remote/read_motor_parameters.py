#!/usr/bin/env python3
"""Read-only Robstride type-17 queries. Never enables, writes or saves parameters."""
import socket,struct,time,json,datetime,fcntl,os,math
from pathlib import Path
lock=open(f'/tmp/poke-remote-{os.getuid()}.lock','w')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
params=[(0x7005,'run_mode','u8','SDK'),(0x700a,'speed_reference','float','SDK'),
(0x7017,'limit_speed','float','SDK'),(0x7018,'limit_current','float','SDK'),
(0x7024,'vel_max','float','SDK'),(0x7025,'acc_set','float','SDK'),
(0x701f,'speed_kp_candidate','float','common Robstride table; firmware mapping unverified'),
(0x7020,'speed_ki_candidate','float','common Robstride table; firmware mapping unverified'),
(0x701b,'measured_speed_candidate','float','common Robstride table; firmware mapping unverified')]
s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);s.bind(('can0',));s.settimeout(.1)
report={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'interface':'can0','operation':'type17_read_only','motors':{}}
for motor in range(1,5):
 result={}
 for index,name,kind,evidence in params:
  found=None
  for attempt in range(2):
   payload=struct.pack('<H',index)+bytes(6)
   s.send(struct.pack('=IB3x8s',0x80000000|(17<<24)|(0x1f<<8)|motor,8,payload))
   deadline=time.monotonic()+.4
   while time.monotonic()<deadline:
    try:frame=s.recv(16)
    except socket.timeout:continue
    cid,dlc,data=struct.unpack('=IB3x8s',frame)
    if cid&0x80000000 and not cid&0x60000000 and (cid>>24)&0x1f==17 and (cid>>8)&0xff==motor and cid&0xff==0x1f and dlc==8 and struct.unpack('<H',data[:2])[0]==index:
     raw=data[4:8];value=raw[0] if kind=='u8' else struct.unpack('<f',raw)[0]
     found={'value':value if math.isfinite(value) else None,'raw':data.hex(),'response_id':hex(cid),'reserved_bytes':data[2:4].hex()};break
   if found is not None:break
  result[name]={'index':hex(index),'evidence':evidence,'response':found}
 report['motors'][str(motor)]=result
 print('ID',motor,json.dumps(result),flush=True)
s.close()
folder=Path(__file__).resolve().parents[1]/'logs/parameter_reads';folder.mkdir(exist_ok=True)
p=folder/(datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json');p.write_text(json.dumps(report,indent=2)+'\n');print('Saved',p)
