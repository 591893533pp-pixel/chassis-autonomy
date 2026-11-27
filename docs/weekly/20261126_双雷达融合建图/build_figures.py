from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
ROOT=Path(__file__).resolve().parent
SRC=ROOT.parents[1]/'移动建图准备_20261126/results/20261126_145615'
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,'savefig.facecolor':'white'})
r=json.loads((SRC/'full_export/export_report.json').read_text())
f=SRC/'full_export/registered_all_frames_xyzi.pcd'
with f.open('rb') as h:
 while h.readline().strip()!=b'DATA binary':pass
 offset=h.tell()
a=np.memmap(f,mode='r',dtype='<f4',offset=offset,shape=(r['points'],4))
p=np.array(a[::max(1,len(a)//250000),:3]);t=np.loadtxt(SRC/'full_export/trajectory.csv',delimiter=',',skiprows=1)
fig,axes=plt.subplots(1,2,figsize=(12,5.8),gridspec_kw={'width_ratios':[1.2,1]},layout='constrained')
ax=axes[0];lo=t[:,1:3].min(0)-7;hi=t[:,1:3].max(0)+7
mask=((p[:,:2]>=lo)&(p[:,:2]<=hi)).all(1);v=p[mask]
ax.scatter(v[:,0],v[:,1],s=.35,c='#429ca6',alpha=.38,rasterized=True)
ax.plot(t[:,1],t[:,2],c='#db7726',lw=2,label='估计轨迹')
ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),title='累计点云局部俯视图',xlabel='X / m',ylabel='Y / m')
ax.set_aspect('equal');ax.grid(alpha=.16);ax.legend(loc='upper left')
ax=axes[1];ax.plot(t[:,1],t[:,2],c='#db7726',lw=2)
ax.scatter(*t[0,1:3],s=80,c='#278466',marker='o',label='起点',zorder=4)
ax.scatter(*t[-1,1:3],s=100,c='#bb3c42',marker='x',label='终点',zorder=5)
ax.set(title='移动轨迹放大图',xlabel='X / m',ylabel='Y / m');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend()
fig.suptitle('双雷达融合移动建图｜2026-11-26',fontsize=17,fontweight='bold')
fig.supxlabel('记录 2955 帧配准点云 · 1416 万点 · 估计路程 10.92 m\n点云仅为抽样显示、局部裁切；坐标系 trial_map，未做回环优化。',fontsize=11)
fig.savefig(ROOT/'images/01_累计点云与轨迹.png',dpi=200);plt.close(fig)
(ROOT/'周报数据依据.json').write_text(json.dumps({'run_report':json.loads((SRC/'report.json').read_text()),'export_report':r,'source_local':str(SRC),'figure':{'sampling_stride':max(1,len(a)//250000),'xy_crop_min':lo.tolist(),'xy_crop_max':hi.tolist(),'all_heights_projected':True},'notes':['3064为监督节点接收融合帧；2955为录包配准输出帧，阶段不同，不是丢帧率。','路程为估计位姿累计长度，不是里程真值。','起终差不作为闭环精度。']},ensure_ascii=False,indent=2)+'\n')
