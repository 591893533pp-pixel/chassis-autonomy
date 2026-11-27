"""30-second enabled-zero test with independent passive CAN capture. No nonzero target."""
import csv,datetime,json,socket,struct,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parents[1]
folder=root/'logs'/'single_wheel'/('zero-feedback-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));folder.mkdir()
sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW);sock.bind(('can0',));sock.settimeout(.1)
packets={i:[] for i in range(1,5)};process=None
try:
 with (folder/'bridge.log').open('w') as log,(folder/'raw.csv').open('w') as raw:
  writer=csv.writer(raw);writer.writerow(['monotonic_s','motor','type','position_raw','velocity_raw','fault'])
  process=subprocess.Popen([str(root/'build/single_wheel'),'0',str(folder/'feedback.csv'),'--hardware','can0'],stdout=log,stderr=log)
  while process.poll() is None:
   try:frame=sock.recv(16)
   except socket.timeout:continue
   stamp=time.monotonic();cid,dlc,data=struct.unpack('=IB3x8s',frame);typ=(cid>>24)&31;i=(cid>>8)&255
   if cid&0x80000000 and not cid&0x60000000 and typ in (2,24) and i in packets and dlc==8:
    packets[i].append(stamp);writer.writerow([stamp,i,typ,int.from_bytes(data[:2],'big'),int.from_bytes(data[2:4],'big'),(cid>>16)&63])
finally:
 if process is not None and process.poll() is None:
  process.terminate();process.wait(timeout=15)
 sock.close()
summary={'exit_code':process.returncode,'motors':{}}
for i,times in packets.items():
 gaps=[b-a for a,b in zip(times,times[1:])]
 summary['motors'][i]={'frames':len(times),'max_rx_gap_ms':max(gaps)*1000 if gaps else None,'gaps_over_100ms':sum(x>.1 for x in gaps)}
(folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(folder);print(json.dumps(summary,indent=2));print((folder/'bridge.log').read_text())
raise SystemExit(process.returncode)
