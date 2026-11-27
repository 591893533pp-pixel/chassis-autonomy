#!/usr/bin/env python3
"""Offline loop rates; optional passive CAN capture never sends frames."""
import argparse,collections,csv,json,select,socket,statistics,struct,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def stats(times):
    if len(times)<2:return {'samples':len(times),'hz':None}
    dt=[(b-a)*1000 for a,b in zip(times,times[1:])]
    v=sorted(dt)
    return {'samples':len(times),'hz':(len(times)-1)/(times[-1]-times[0]),'interval_ms_p50_p95_max':[statistics.median(v),v[min(len(v)-1,int(.95*len(v)))],max(v)],'over_15ms':sum(x>15 for x in v)}
def log_rates(folder):
    result={'log':str(folder),'note':'Loop rates are not proof of fresh motor feedback. Use passive CAN for wire rates.'}
    for name,key in [('commands.jsonl','t'),('lift_feedback.jsonl','t'),('lift_timing.jsonl','t')]:
        p=folder/name
        if p.exists():
            rows=[json.loads(s) for s in p.read_text().splitlines() if s.strip()]
            result[name]=stats([r[key] for r in rows]);result[name]['missed_slots_max']=max((r.get('missed_slots',0) for r in rows),default=0)
    p=folder/'feedback.csv'
    if p.exists():result['chassis_loop']=stats([float(r['monotonic_s']) for r in csv.DictReader(p.open())])
    return result
def capture(seconds,interface):
    streams=collections.defaultdict(list);counts=collections.Counter();estimated_bits=0
    with socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW) as s:
        s.bind((interface,));end=time.monotonic()+seconds
        while time.monotonic()<end:
            if not select.select([s],[],[],min(.1,max(0,end-time.monotonic())))[0]:continue
            cid,n,data=struct.unpack('=IB3x8s',s.recv(16));now=time.monotonic()
            ext=bool(cid&0x80000000);cid&=0x1fffffff;counts[hex(cid)]+=1
            estimated_bits+=(160 if ext else 135) # Conservative approximate full 8-byte frame incl stuffing/intermission.
            key=None
            if ext:
                kind=cid>>24&31
                if kind in (2,24):key=f'wheel_{cid>>8&255}_feedback_type_{kind}'
                elif kind==18 and data[:2]==b'\x0a\x70':key=f'wheel_{cid&255}_velocity_write'
            elif cid in (0x601,0x581):
                idx=int.from_bytes(data[1:3],'little')
                if cid==0x601 and idx==0x60ff and data[0]==0x23:key='lift_velocity_write'
                if cid==0x581 and data[0] in (0x43,0x47,0x4b,0x4f):key=f'lift_feedback_{idx:04x}'
            if key:streams[key].append(now)
    return {'duration_s':seconds,'interface':interface,'passive_only':True,'approx_bus_load_fraction':estimated_bits/(seconds*1e6),'assumed_bitrate':1000000,'streams':{k:stats(v) for k,v in streams.items()},'frame_counts':counts,'note':'Userspace arrival timing; estimated bus load, not hardware timestamp metrology.'}
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--log',type=Path);p.add_argument('--can-seconds',type=float,default=0);p.add_argument('--interface',default='can0');a=p.parse_args()
    if not 0<=a.can_seconds<=600:p.error('capture duration must be 0..600 seconds')
    result={}
    if a.can_seconds:result['can']=capture(a.can_seconds,a.interface)
    folder=a.log
    if folder is None and (ROOT/'LAST_LOG.txt').exists():folder=Path((ROOT/'LAST_LOG.txt').read_text().strip())
    if folder:result['logs']=log_rates(folder)
    dest=ROOT/'logs'/('rates_'+time.strftime('%Y%m%d_%H%M%S')+'.json');dest.parent.mkdir(exist_ok=True);dest.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));print('保存：',dest)
if __name__=='__main__':main()
