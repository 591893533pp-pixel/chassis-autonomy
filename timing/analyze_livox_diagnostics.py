#!/usr/bin/env python3
"""Offline decode MID360 SDK2 0x0102 state packets; no device access.
Layout and key IDs follow Livox-SDK2 sdk_protocol.h and livox_lidar_def.h.
Checks packet lengths, CCITT header CRC, CRC32 data and TLV lengths.
"""
import argparse,binascii,collections,json,pathlib,socket,struct,zlib
from analyze_ptp_trace import packets
FIELDS={0x8006:('work_state',1,False),0x8008:('powerup_count',4,False),
 0x8009:('local_time_ns',8,False),0x800a:('last_sync_ns',8,False),
 0x800b:('offset_ns',8,True),0x800c:('sync_type',1,False),0x800e:('diag_status',2,False)}
def decode(ts,frame):
 if len(frame)<14:return None
 proto=int.from_bytes(frame[12:14],'big');off=14
 while proto in (0x8100,0x88a8):
  if len(frame)<off+4:return None
  proto=int.from_bytes(frame[off+2:off+4],'big');off+=4
 if proto!=0x0800 or len(frame)<off+20 or frame[off+9]!=17:return None
 ihl=(frame[off]&15)*4
 if ihl<20 or len(frame)<off+ihl+8 or int.from_bytes(frame[off+6:off+8],'big')&0x3fff:return None
 ip=socket.inet_ntoa(frame[off+12:off+16]);off+=ihl
 sp,dp,n=struct.unpack_from('!HHH',frame,off);off+=8
 if (sp,dp) not in ((56100,56101),(56200,56201)):return None
 p=frame[off:off+n-8]
 if len(p)<28 or p[:2]!=b'\xaa\x00' or int.from_bytes(p[8:10],'little')!=0x0102:return None
 if int.from_bytes(p[2:4],'little')!=len(p):raise ValueError('SDK length mismatch')
 if binascii.crc_hqx(p[:18],0xffff)!=int.from_bytes(p[18:20],'little'):raise ValueError('SDK header CRC mismatch')
 if zlib.crc32(p[24:])!=int.from_bytes(p[20:24],'little'):raise ValueError('SDK data CRC mismatch')
 count=int.from_bytes(p[24:26],'little');off=28
 d={'capture_ns':ts,'ip':ip,'sequence':int.from_bytes(p[4:8],'little')}
 seen=set()
 for _ in range(count):
  if off+4>len(p):raise ValueError('truncated TLV header')
  key,n=struct.unpack_from('<HH',p,off);off+=4
  if off+n>len(p) or key in seen:raise ValueError('invalid TLV bounds/duplicate')
  seen.add(key);v=p[off:off+n];off+=n
  if key in FIELDS:
   name,size,signed=FIELDS[key]
   if n!=size:raise ValueError('unexpected numeric field length')
   d[name]=int.from_bytes(v,'little',signed=signed)
  elif key==0x8011:
   if n!=32:raise ValueError('HMS must contain 8 codes')
   d['hms']=['0x%08x'%x for x in struct.unpack('<8I',v) if x]
  elif key in (0x8000,0x8001):d['sn' if key==0x8000 else 'product_info']=v.rstrip(b'\0').decode('ascii')
 if off!=len(p):raise ValueError('trailing TLV data')
 if 'local_time_ns' in d and 'last_sync_ns' in d:d['sync_age_ms']=(d['local_time_ns']-d['last_sync_ns'])/1e6
 return d

def summarize(folder):
 rows=[];bad=[]
 for ts,p,_ in packets(folder/'ptp_control.pcap'):
  try:r=decode(ts,p)
  except ValueError as e:bad.append({'capture_ns':ts,'error':str(e)});continue
  if r:rows.append(r)
 groups=collections.defaultdict(list)
 for r in rows:groups[r['ip']].append(r)
 result={'invalid_packets':bad,'sensors':{}}
 for ip,rr in sorted(groups.items()):
  transitions=[{'capture_ns':b['capture_ns'],'before':a.get('hms'),'after':b.get('hms')} for a,b in zip(rr,rr[1:]) if a.get('hms')!=b.get('hms')]
  offsets=[r['offset_ns'] for r in rr if 'offset_ns' in r];ages=[r['sync_age_ms'] for r in rr if 'sync_age_ms' in r]
  result['sensors'][ip]={'count':len(rr),'duration_s':(rr[-1]['capture_ns']-rr[0]['capture_ns'])/1e9,
   'sn':sorted({r.get('sn') for r in rr}),'product_info':sorted({r.get('product_info') for r in rr}),
   'powerup_counts':sorted({r.get('powerup_count') for r in rr}),'work_states':sorted({r.get('work_state') for r in rr}),
   'sync_types':sorted({r.get('sync_type') for r in rr}),'diag_statuses':sorted({r.get('diag_status') for r in rr}),
   'offset_abs_max_ns':max(map(abs,offsets),default=None),'sync_age_min_ms':min(ages,default=None),'sync_age_max_ms':max(ages,default=None),
   'hms_values':sorted({code for r in rr for code in r.get('hms',[])}),'hms_transitions':transitions,
   'first_sample_hms':rr[0].get('hms'),'diag_gap_max_s':max([(b['capture_ns']-a['capture_ns'])/1e9 for a,b in zip(rr,rr[1:])],default=None),
   'local_time_backward_count':sum(b['local_time_ns']<=a['local_time_ns'] for a,b in zip(rr,rr[1:]) if 'local_time_ns' in a and 'local_time_ns' in b)}
 (folder/'lidar_diagnostics.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
 (folder/'lidar_diagnostic_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('directory',type=pathlib.Path);summarize(p.parse_args().directory)
