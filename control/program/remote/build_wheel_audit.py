"""Offline audit: unchanged logs -> normalized CSV, PNG/SVG figures, manifest and report."""
import csv,json,hashlib,html,math,sys,struct
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':9})
ROOT=Path(sys.argv[1]).resolve()
POSITIONS={1:'右后',2:'左后',3:'左前',4:'右前'}
items=[];stats=[];wire_stats=[];position_jumps=[]
def read(p):
 with p.open() as f:return list(csv.DictReader(f))
def savecsv(path,rows,fields=None):
 path.parent.mkdir(parents=True,exist_ok=True)
 with path.open('w') as f:
  w=csv.DictWriter(f,fieldnames=fields or list(rows[0]));w.writeheader();w.writerows(rows)
def identifier(p):return '__'.join(p.relative_to(ROOT/'raw').with_suffix('').parts)
def savefig(fig,name):
 fig.tight_layout(rect=(0,0,1,.965))
 for ext in ['png','svg']:fig.savefig(ROOT/'plots'/f'{name}.{ext}',dpi=160)
 plt.close(fig)
def comparison(p):
 rows=read(p);name=identifier(p);out=[]
 if not rows:
  fig,ax=plt.subplots(figsize=(9,3));ax.axis('off');ax.text(.05,.6,'无反馈样本：保留失败记录，不填造曲线。',fontsize=15);ax.set_title(name)
  savefig(fig,name);items.append((name,'无样本',str(p.relative_to(ROOT))));return
 isremote='requested_vx' in rows[0];single='selected_id' in rows[0]
 if not isremote and not single:return
 first=float(rows[0].get('monotonic_s',0));fig,axes=plt.subplots(4,2,figsize=(13,10),sharex=True)
 for i in range(1,5):
  ts=[];targets=[];feedback=[];valids=[]
  for r in rows:
   t=float(r['monotonic_s'])-first if isremote else float(r['elapsed_s'])
   target=float(r[f'wheel{i}_cmd_rad_s']) if isremote else (float(r['target_rad_s']) if int(r['selected_id'])==i else 0.)
   v=float(r[f'wheel{i}_feedback_rad_s'] if isremote else r[f'wheel{i}_rad_s']);valid=int(r['feedback_valid'] if isremote else r['valid'])
   ts.append(t);targets.append(target);feedback.append(v if valid else np.nan);valids.append(valid)
   out.append(dict(elapsed_s=t,motor_id=i,target_rad_s=target,feedback_rad_s=v,feedback_valid=valid,error_rad_s=v-target if valid else '',age_ms=r['feedback_age_ms'] if isremote else r['age_ms'],fault=r[f'wheel{i}_fault'],target_source='program_send_target' if isremote else 'selected_id_target; other wheels zero',source=str(p.relative_to(ROOT))))
  t=np.array(ts);u=np.array(targets);v=np.array(feedback);e=v-u
  a,b=axes[i-1];a.plot(t,u,color='black',lw=1.4,label='发送目标（程序记录）');a.plot(t,v,lw=.55,alpha=.75,label='原始速度反馈');b.plot(t,e,lw=.6,color='#c45037');b.axhline(0,color='black',lw=.6)
  a.set_ylabel(f'ID{i} {POSITIONS[i]}\nrad/s');b.set_ylabel('反馈−目标 rad/s')
  for ax in [a,b]:ax.grid(alpha=.25)
  mask=np.isfinite(v)
  if 'new_tests' in p.parts:mask &= (t>=1)&(t<9.9)
  if mask.any():stats.append(dict(dataset=name,motor=i,n=int(mask.sum()),target_mean=float(u[mask].mean()),feedback_mean=float(v[mask].mean()),feedback_std=float(v[mask].std()),error_mean=float(e[mask].mean()),error_rmse=float(np.sqrt(np.mean(e[mask]**2))),sample_window='1–9.9 s' if 'new_tests' in p.parts else 'all valid samples, may include stops/reversals',source=str(p.relative_to(ROOT))))
 axes[0,0].legend(loc='upper right');axes[0,0].set_title('发送 / 反馈');axes[0,1].set_title('跟踪误差波动（未经平滑）')
 for ax in axes[-1]:ax.set_xlabel('记录起点后的时间 / s')
 fig.suptitle(name+'\n无效反馈留空；图包含启动、停止及故障前数据',fontsize=10)
 savefig(fig,name);savecsv(ROOT/'processed'/f'{name}.csv',out)
 items.append((name,'程序目标与反馈',str(p.relative_to(ROOT))))
def decode_wire(p):
 decoded=[]
 for r in read(p):
  cid=int(r['can_id_hex'],16);dlc=int(r['dlc']);data=bytes.fromhex(r['data_hex']);typ=(cid>>24)&31
  if dlc!=8 or not cid&0x80000000 or cid&0x60000000:continue
  if typ in (2,24):
   i=(cid>>8)&255
   if i not in POSITIONS:continue
   decoded.append(dict(monotonic_s=float(r['monotonic_s']),motor=i,event='rx',type=typ,target_rad_s='',position_rad=int.from_bytes(data[:2],'big')*25.14/65535-12.57,velocity_rad_s=int.from_bytes(data[2:4],'big')*88/65535-44,fault=(cid>>16)&63))
  elif typ==18 and int.from_bytes(data[:2],'little')==0x700a:
   i=cid&255
   if i not in POSITIONS:continue
   decoded.append(dict(monotonic_s=float(r['monotonic_s']),motor=i,event='tx_observed',type=typ,target_rad_s=struct.unpack('<f',data[4:8])[0],position_rad='',velocity_rad_s='',fault=''))
 return decoded

def wire_plot(p,decoded,zero_history=False):
 if not decoded:return
 name=identifier(p)+'__position_check';savecsv(ROOT/'processed'/f'{name}__decoded.csv',decoded)
 base=decoded[0]['monotonic_s'];fig,axes=plt.subplots(4,2,figsize=(13,10),sharex=True);derived=[]
 for i in range(1,5):
  rx=[r for r in decoded if r['motor']==i and r['event']=='rx'];tx=[r for r in decoded if r['motor']==i and r['event']=='tx_observed']
  if not rx:continue
  t=np.array([r['monotonic_s']-base for r in rx]);pos=np.array([r['position_rad'] for r in rx]);v=np.array([r['velocity_rad_s'] for r in rx]);pos=np.unwrap(pos,period=25.14)
  jump=np.r_[False,np.abs(np.diff(pos))>.05]
  for k in np.flatnonzero(jump):position_jumps.append(dict(dataset=name,motor=i,elapsed_s=float(t[k]),previous_position_rad=float(pos[k-1]),position_rad=float(pos[k]),delta_rad=float(pos[k]-pos[k-1]),previous_type=rx[k-1]['type'],frame_type=rx[k]['type'],source=str(p.relative_to(ROOT))))
  u=np.full(len(t),np.nan)
  if zero_history:u[:]=0
  elif tx:
   tt=np.array([r['monotonic_s']-base for r in tx]);tu=np.array([r['target_rad_s'] for r in tx]);j=np.searchsorted(tt,t,side='right')-1;ok=j>=0;u[ok]=tu[j[ok]]
  # Causal 0.5-second position secant: average angular velocity, not instantaneous velocity.
  j=np.searchsorted(t,t-.5,side='right')-1;ok=j>=0;j=np.maximum(j,0);dt=t-t[j]
  gapcount=np.r_[0,np.cumsum(np.diff(t)>.1)];ok &= (dt>=.5)&(gapcount==gapcount[j])
  pv=np.full(len(t),np.nan);pv[ok]=(pos[ok]-pos[j[ok]])/dt[ok]
  a,b=axes[i-1];a.plot(t,u,color='black',lw=1.3,label='总线发送目标' if not zero_history else '零速目标（已知测试条件）');a.plot(t,v,lw=.5,alpha=.5,label='原始速度反馈');a.plot(t,pv,color='#15825d',lw=1.1,label='位置差分平均速度（0.5s）');b.plot(t,pos-pos[0],color='#3065a8',lw=1);a.set_ylabel(f'ID{i} {POSITIONS[i]}\nrad/s');b.set_ylabel('相对初始位置 / rad')
  if jump.any():b.scatter(t[jump],(pos-pos[0])[jump],s=18,color='red',label='相邻位置跳变 >0.05rad');b.legend(fontsize=7)
  for ax in [a,b]:ax.grid(alpha=.25)
  for k in range(len(t)):derived.append(dict(elapsed_s=t[k],motor=i,observed_target_rad_s=u[k],raw_speed_rad_s=v[k],unwrapped_position_rad=pos[k],position_average_speed_rad_s=pv[k],window_s=dt[k] if ok[k] else '',source=str(p.relative_to(ROOT))))
  moving=np.flatnonzero(np.isfinite(u)&(np.abs(u)>.19))
  if len(moving):
   mask=(t>=t[moving[0]]+1)&(t<=t[moving[-1]]-.1)&np.isfinite(pv)
   if mask.any():wire_stats.append(dict(dataset=name,motor=i,target_mean=float(u[mask].mean()),feedback_mean=float(v[mask].mean()),feedback_std=float(v[mask].std()),position_speed_mean=float(pv[mask].mean()),position_speed_std=float(pv[mask].std()),position_jump_count=int(jump.sum()),n=int(mask.sum())))
 axes[0,0].legend(fontsize=8);axes[0,0].set_title('总线目标 / 原始速度 / 位置平均速度');axes[0,1].set_title('原始位置解缠后的累计变化')
 for ax in axes[-1]:ax.set_xlabel('首个解码 CAN 帧后的时间 / s')
 fig.suptitle(name+'\n位置差分是 0.5 秒区间平均，不能与瞬时反馈噪声等同；超过 100ms 接收间隙不计算',fontsize=10)
 savefig(fig,name);savecsv(ROOT/'processed'/f'{name}__derived.csv',derived);items.append((name,'原始 CAN 与位置交叉验证',str(p.relative_to(ROOT))))

for p in sorted((ROOT/'raw').rglob('*.csv')):
 if p.name=='can_frames.csv':wire_plot(p,decode_wire(p));continue
 if p.name=='raw.csv':
  r=read(p)
  if r and 'position_raw' in r[0]:
   d=[dict(monotonic_s=float(x['monotonic_s']),motor=int(x['motor']),event='rx',type=int(x['type']),target_rad_s='',position_rad=int(x['position_raw'])*25.14/65535-12.57,velocity_rad_s=int(x['velocity_raw'])*88/65535-44,fault=int(x['fault'])) for x in r];wire_plot(p,d,True)
  continue
 comparison(p)
 print('PLOTTED',p.relative_to(ROOT),flush=True)
if stats:savecsv(ROOT/'processed/summary.csv',stats)
if wire_stats:savecsv(ROOT/'processed/position_speed_summary.csv',wire_stats)
if position_jumps:savecsv(ROOT/'processed/position_jumps.csv',position_jumps)
# One concise figure for all eight newly recorded motion segments.
fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
labels=[]
for k,s in enumerate(wire_stats):
 labels.append(f"ID{s['motor']} {POSITIONS[s['motor']]} {'正转' if s['target_mean']>0 else '反转'}"+(' 复测' if 'repeat' in s['dataset'] else '')+(' *' if s['position_jump_count'] else ''))
 for field,marker,color,label in [('target_mean','s','black','发送目标均值'),('feedback_mean','o','#4477aa','速度反馈均值'),('position_speed_mean','^','#228855','位置差分均值')]:axes[0].plot(k,s[field],marker=marker,color=color,label=label if k==0 else None)
 axes[1].plot(k,s['feedback_std'],'o',color='#4477aa',label='原始速度标准差' if k==0 else None);axes[1].plot(k,s['position_speed_std'],'^',color='#228855',label='0.5s位置平均速度标准差' if k==0 else None)
for a in axes:a.grid(alpha=.25);a.legend()
axes[0].set_ylabel('平均速度 / rad/s');axes[1].set_ylabel('标准差 / rad/s');axes[1].set_xticks(range(len(labels)),labels,rotation=20)
fig.suptitle('新采集 10 秒单轮测试对比\n两种反馈带宽不同；* 含原始位置跳变，未删点，标准差受异常影响')
savefig(fig,'00_overview')
# Browseable image index, per-record links and manifest.
links=['<!doctype html><meta charset="utf-8"><title>轮速测试审核</title><style>body{font-family:sans-serif;max-width:1200px;margin:30px auto}img{max-width:100%}section{border-top:1px solid #ccc;margin-top:30px}a{margin-right:15px}</style><h1>轮速发送与反馈审核</h1><p>原始日志只复制不改写；无效样本留空；空日志不补造数据。所有图片可下载 PNG 或 SVG。</p><img src="../plots/00_overview.png">']
for name,kind,source in items:
 links.append(f'<section><h2>{html.escape(name)}</h2><p>{kind}</p><a href="../{html.escape(source)}">原始数据</a><a href="../plots/{name}.svg">SVG</a><a href="../plots/{name}.png">PNG</a><img loading="lazy" src="../plots/{name}.png"></section>')
(ROOT/'reports/index.html').write_text('\n'.join(links))
savecsv(ROOT/'reports/plot_index.csv',[dict(name=n,kind=k,source=s,png=f'plots/{n}.png',svg=f'plots/{n}.svg') for n,k,s in items])
manifest=[]
for top in ['raw','source']:
 for p in sorted((ROOT/top).rglob('*')):
  if p.is_file():manifest.append(dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
savecsv(ROOT/'reports/raw_source_sha256.csv',manifest)
print('DONE',len(items),'per-record plot sets; wire summaries',len(wire_stats),flush=True)
