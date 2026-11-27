#!/usr/bin/env python3
"""Offline MID360 UDP header timing audit; accepts intentional snaplen truncation.
Arrival minus packet timestamp includes transport latency; it is not clock offset.
"""
import argparse,collections,json,pathlib,socket,struct
from analyze_ptp_trace import packets
EXPECTED={'192.168.1.132':'0c:9a:e6:62:6b:ef','192.168.1.159':'34:91:f0:e6:fe:29'}
def decode(ts,p):
 if len(p)<14:return None
 proto=int.from_bytes(p[12:14],'big');off=14
 while proto in (0x8100,0x88a8):
  if len(p)<off+4:return None
  proto=int.from_bytes(p[off+2:off+4],'big');off+=4
 if proto!=0x0800 or len(p)<off+20 or p[off+9]!=17:return None
 ip=socket.inet_ntoa(p[off+12:off+16]);dst=socket.inet_ntoa(p[off+16:off+20]);ihl=(p[off]&15)*4
 if ip not in EXPECTED:return None
 if ihl<20 or len(p)<off+ihl+8:raise ValueError('bad IP header')
 if int.from_bytes(p[off+6:off+8],'big')&0x3fff:raise ValueError('fragmented sensor packet')
 off+=ihl;sp,dp,n=struct.unpack_from('!HHH',p,off);off+=8
 if sp not in (56300,56400):return None
 if p[6:12].hex(':')!=EXPECTED[ip] or dst!='192.168.1.4' or dp!=sp+1:raise ValueError('identity/destination mismatch')
 b=p[off:off+n-8]
 if len(b)<36 or b[0]!=0 or int.from_bytes(b[1:3],'little')!=n-8:raise ValueError('invalid Livox header')
 if b[10]!=(0 if sp==56400 else 1):raise ValueError('unexpected point/IMU encoding')
 return {'capture_ns':ts,'stream':f'{ip}:{sp}','stamp_ns':int.from_bytes(b[28:36],'little'),
  'sequence':int.from_bytes(b[7:9],'little'),'time_type':b[11]}
def new_stats():
 return dict(count=0,age_min_ns=None,age_max_ns=None,stamp_gap_max_ns=0,capture_gap_max_ns=0,
             backward_or_duplicate_stamps=0,sequence_breaks=0,non_ptp_packets=0,age_over_5ms=0)
def update(s,r,old):
 age=r['capture_ns']-r['stamp_ns'];s['count']+=1
 s['age_min_ns']=age if s['age_min_ns'] is None else min(age,s['age_min_ns'])
 s['age_max_ns']=age if s['age_max_ns'] is None else max(age,s['age_max_ns'])
 s['non_ptp_packets']+=int(r['time_type']!=1);s['age_over_5ms']+=int(abs(age)>5000000)
 if old:
  dt=r['stamp_ns']-old['stamp_ns'];s['stamp_gap_max_ns']=max(s['stamp_gap_max_ns'],dt)
  s['capture_gap_max_ns']=max(s['capture_gap_max_ns'],r['capture_ns']-old['capture_ns'])
  s['backward_or_duplicate_stamps']+=int(dt<=0)
  s['sequence_breaks']+=int((r['sequence']-old['sequence'])%65536!=1)
def summarize(folder):
 allstats=collections.defaultdict(new_stats);bins=collections.defaultdict(new_stats);last={};first={};errors=[];invalid=0
 for ts,p,_ in packets(folder/'sensor_headers.pcap'):
  try:r=decode(ts,p)
  except ValueError as e:
   invalid+=1
   if len(errors)<20:errors.append({'capture_ns':ts,'error':str(e)})
   continue
  if r is None:continue
  key=r['stream'];old=last.get(key);first.setdefault(key,ts)
  update(allstats[key],r,old);update(bins[(ts//10**9,key)],r,old);last[key]=r
 for key,s in allstats.items():s.update(first_capture_ns=first[key],last_capture_ns=last[key]['capture_ns'],duration_s=(last[key]['capture_ns']-first[key])/1e9)
 result={'streams':dict(allstats),'invalid_count':invalid,'invalid_examples':errors,
 'limitations':'Kernel receive time minus packet timestamp includes latency; this is a continuity/large-jump check, not a precision clock calibration. Inspect sensor_tcpdump.log for capture loss. Snaplen truncation is intentional.'}
 (folder/'sensor_timing_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
 (folder/'sensor_timing_seconds.jsonl').write_text(''.join(json.dumps(dict(second=sec,stream=key,**s))+'\n' for (sec,key),s in sorted(bins.items())))
 print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('directory',type=pathlib.Path);summarize(p.parse_args().directory)
