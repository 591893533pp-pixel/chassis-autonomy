#!/usr/bin/env python3
"""Read classic tcpdump pcap; summarize PTP sources, gaps, version and paired timestamps.
No transmitted packets or clock changes. A host capture is not proof of sensor-side delivery.
"""
import argparse,collections,json,math,pathlib,socket,statistics,struct
TYPES={0:'Sync',1:'Delay_Req',2:'Pdelay_Req',3:'Pdelay_Resp',8:'Follow_Up',9:'Delay_Resp',10:'Pdelay_Resp_Follow_Up',11:'Announce',12:'Signaling',13:'Management'}
def packets(path):
 with path.open('rb') as f:
  magic=f.read(4);formats={b'\xd4\xc3\xb2\xa1':('<',1000),b'\xa1\xb2\xc3\xd4':('>',1000),b'\x4d\x3c\xb2\xa1':('<',1),b'\xa1\xb2\x3c\x4d':('>',1)}
  if magic not in formats:raise ValueError('Expected classic micro/nanosecond pcap')
  endian,scale=formats[magic];header=f.read(20)
  if len(header)!=20 or struct.unpack(endian+'HHIIII',header)[-1]!=1:raise ValueError('Expected Ethernet PCAP')
  while True:
   h=f.read(16)
   if not h:break
   if len(h)!=16:raise ValueError('truncated pcap record')
   sec,fraction,caplen,wirelen=struct.unpack(endian+'IIII',h);data=f.read(caplen)
   if len(data)!=caplen:raise ValueError('truncated frame')
   yield sec*10**9+fraction*scale,data,wirelen

def decode(ts,data):
 if len(data)<14:return None
 proto=int.from_bytes(data[12:14],'big');off=14
 while proto in (0x8100,0x88a8):
  if len(data)<off+4:return None
  proto=int.from_bytes(data[off+2:off+4],'big');off+=4
 src=data[6:12].hex(':');dst=data[:6].hex(':');sp=dp=None
 if proto==0x0800:
  if len(data)<off+20 or data[off+9]!=17:return None
  ihl=(data[off]&15)*4
  if ihl<20 or len(data)<off+ihl+8 or int.from_bytes(data[off+6:off+8],'big')&0x3fff:return None
  src=socket.inet_ntoa(data[off+12:off+16]);dst=socket.inet_ntoa(data[off+16:off+20]);off+=ihl
  sp,dp,length=struct.unpack_from('!HHH',data,off);off+=8
  if sp not in (319,320) and dp not in (319,320):return None
  payload=data[off:off+length-8];transport='UDPv4'
 elif proto==0x88f7:payload=data[off:];transport='Ethernet'
 else:return None
 if len(payload)<34:return None
 length=int.from_bytes(payload[2:4],'big')
 if length<34 or len(payload)<length:return None
 typ=payload[0]&15
 d={'capture_ns':ts,'type':TYPES.get(typ,str(typ)),'transport':transport,'src':src,'dst':dst,'source_port_identity':payload[20:30].hex(),
    'version_major':payload[1]&15,'version_minor':payload[1]>>4,'domain':payload[4],'transport_specific':payload[0]>>4,
    'sequence':int.from_bytes(payload[30:32],'big'),'flags':int.from_bytes(payload[6:8],'big'),
    'correction_ns':int.from_bytes(payload[8:16],'big',signed=True)/65536,'log_interval':int.from_bytes(payload[33:34],'big',signed=True)}
 if typ in (0,1,2,3,8,9,10,11) and len(payload)>=44:
  d['origin_ns']=int.from_bytes(payload[34:40],'big')*10**9+int.from_bytes(payload[40:44],'big')
 if typ in (3,9,10) and len(payload)>=54:d['requester']=payload[44:54].hex()
 if typ==11 and len(payload)>=64:d.update(grandmaster=payload[53:61].hex(),priority1=payload[47],clock_class=payload[48],priority2=payload[52],steps_removed=int.from_bytes(payload[61:63],'big'),utc_offset=int.from_bytes(payload[44:46],'big',signed=True))
 return d

def summarize(folder):
 rows=[];total=0;truncated=0
 for ts,p,wirelen in packets(folder/'ptp_control.pcap'):
  total+=1;truncated+=int(len(p)<wirelen);d=decode(ts,p)
  if d:rows.append(d)
 groups=collections.defaultdict(list)
 for d in rows:groups[(d['type'],d['source_port_identity'],d['domain'],d.get('requester',''))].append(d)
 streams=[]
 for (typ,identity,domain,requester),r in sorted(groups.items()):
  gaps=[(b['capture_ns']-a['capture_ns'])/1e9 for a,b in zip(r,r[1:])];deltas=[(b['sequence']-a['sequence'])%65536 for a,b in zip(r,r[1:])]
  streams.append({'type':typ,'identity':identity,'domain':domain,'requester':requester,'sources':sorted({x['src'] for x in r}),'count':len(r),
    'interval_median_s':statistics.median(gaps) if gaps else None,'interval_max_s':max(gaps) if gaps else None,
    'nonconsecutive_sequences':sum(x!=1 for x in deltas),'sequence_delta_examples':[x for x in deltas if x!=1][:12],
    'log_intervals':sorted({x['log_interval'] for x in r})})
 syncs={(r['source_port_identity'],r['domain'],r['sequence']):r for r in rows if r['type']=='Sync'}
 followups={(r['source_port_identity'],r['domain'],r['sequence']):r for r in rows if r['type']=='Follow_Up'}
 pairs=[]
 for key,s in syncs.items():
  f=followups.get(key)
  if f and f.get('origin_ns'):
   pairs.append({'identity':key[0],'sequence':key[2],'followup_delay_ms':(f['capture_ns']-s['capture_ns'])/1e6,
    'origin_minus_sync_capture_us':(f['origin_ns']-s['capture_ns'])/1000})
 clock=[];health=[]
 hp=folder/'host_clock_health.jsonl'
 if hp.exists():
  for line in hp.read_text().splitlines():
   x=json.loads(line);clock.append(x);h=x.get('health',{});health.append(h.get('healthy',False))
 steps=[((b['realtime_ns']-a['realtime_ns'])-(b['monotonic_ns']-a['monotonic_ns']))/1e6 for a,b in zip(clock,clock[1:])]
 anomalies=[]
 for stream in streams:
  if stream['type'] in ('Sync','Follow_Up') and stream['interval_max_s'] is not None:
   expected=[2.0**v for v in stream['log_intervals'] if -10<=v<=10]
   if expected and stream['interval_max_s']>max(expected)*1.5:anomalies.append('Captured '+stream['type']+' gap exceeds 1.5x advertised interval: '+stream['identity'])
 versions=sorted({(r['version_major'],r['version_minor']) for r in rows})
 if any(v!=(2,0) for v in versions):anomalies.append('PTP version other than 2.0 observed; inspect domain/source')
 masters=sorted({(r['source_port_identity'],r['domain']) for r in rows if r['type']=='Sync'})
 for domain in {d for _,d in masters}:
  if sum(d==domain for _,d in masters)>1:anomalies.append(f'Multiple Sync sources observed in domain {domain}; distinguish transition from competition')
 if steps and max(map(abs,steps))>2:anomalies.append('Host realtime-vs-monotonic step over 2ms observed')
 result={'packet_count':total,'ptp_count':len(rows),'truncated_frames':truncated,'versions':versions,'sync_sources':masters,'streams':streams,
 'announces':[dict(source=r['source_port_identity'],grandmaster=r.get('grandmaster'),domain=r['domain'],clock_class=r.get('clock_class'),utc_offset=r.get('utc_offset')) for r in rows if r['type']=='Announce'][:10],
 'matched_sync_followups':len(pairs),'two_step_sync_without_captured_followup':sum(bool(r['flags']&0x0200) and k not in followups for k,r in syncs.items()),
 'followup_delay_max_ms':max([x['followup_delay_ms'] for x in pairs],default=None),'origin_vs_capture_max_abs_us':max([abs(x['origin_minus_sync_capture_us']) for x in pairs],default=None),
 'host_step_max_abs_ms':max(map(abs,steps),default=None),'host_unhealthy_samples':sum(not v for v in health),'observed_anomalies':anomalies,
 'limitations':'Capture boundary may lose pair members. Sequence jumps can reflect capture loss: inspect tcpdump.log. Source count alone does not prove competing masters. NIC capture does not prove sensor-side receipt or identify a past HMS onset.'}
 (folder/'ptp_analysis.json').write_text(json.dumps(result,indent=2)+'\n');(folder/'ptp_packets.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
 print(json.dumps(result,indent=2))
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('directory',type=pathlib.Path);a=parser.parse_args();summarize(a.directory)
