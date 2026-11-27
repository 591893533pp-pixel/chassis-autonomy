#!/usr/bin/env python3
import pathlib,json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=pathlib.Path(__file__).resolve().parent
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
def read(p):return json.loads((ROOT/p).read_text())
r1=read('部署准备/lidar_commission_20261125/corner_20261125_01/result/report.json');r2=read('部署准备/lidar_commission_20261125/corner_20261125_02/result/report.json');d=read('标定稳定性复核_20261125/common_planes_candidate_v1/holdout_D/report.json');g=read('车体外参核对_20261125/ground_audit.json')
spread=[r1['trial_translation_spread_m']*1000,r2['trial_translation_spread_m']*1000,d['training_scene_difference']['translation_m']*1000]
ground=[abs(v['front_left']['plane_z_at_provisional_origin_m']-v['rear_right']['plane_z_at_provisional_origin_m'])*1000 for v in g['scenes'].values()]
fig,axes=plt.subplots(1,3,figsize=(15,4.7));blue='#2378a5';green='#16877b';gray='#a6b6c3';orange='#d77926'
ax=axes[0];bars=ax.bar(['第一轮\n点云 ICP','第二轮\n点云 ICP','第二轮\n共同平面'],spread,color=[gray,blue,green],width=.6);ax.bar_label(bars,fmt='%.2f',padding=5);ax.axhline(10,c=orange,ls='--',label='原门限 10 mm');ax.set_ylim(0,27);ax.set_ylabel('训练解平移差异（mm）');ax.set_title('跨场景一致性逐步改善',loc='left',weight='bold');ax.legend(frameon=False,fontsize=9)
ax=axes[1];x=np.arange(2);a=[d['baseline']['plane_rmse_m']*1000,d['baseline']['plane_abs_p95_m']*1000];b=[d['candidate']['plane_rmse_m']*1000,d['candidate']['plane_abs_p95_m']*1000];bars=ax.bar(x-.18,a,.34,color=gray,label='原安装初值');ax.bar_label(bars,fmt='%.1f',padding=4);bars=ax.bar(x+.18,b,.34,color=green,label='冻结候选');ax.bar_label(bars,fmt='%.1f',padding=4);ax.set_xticks(x,['RMSE','P95']);ax.set_ylim(0,110);ax.set_ylabel('匹配平面残差（mm）');ax.set_title('新位置 D：独立验证',loc='left',weight='bold');ax.legend(frameon=False,fontsize=9)
ax=axes[2];bars=ax.bar(list(g['scenes']),ground,color=orange,width=.55);ax.bar_label(bars,fmt='%.1f',padding=4);ax.set_ylim(0,30);ax.set_ylabel('两台地面拟合高度差（mm）');ax.set_title('待解决：地面仍有分层',loc='left',weight='bold');ax.text(.02,.92,'同一暂定车体原点处比较',transform=ax.transAxes,fontsize=9,color='#666666')
for ax in axes:ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True)
fig.suptitle('独立底盘双雷达标定 · 2026-11-25',x=.06,ha='left',weight='bold',fontsize=17)
fig.text(.06,.02,'左图包含不同采集与算法条件，不代表单一因素的对照实验；整体残差合格不等于各平面或车体外参已验收。',fontsize=9,color='#555555')
fig.tight_layout(rect=(.02,.07,1,.91));fig.savefig(OUT/'images/01_标定指标与待解决问题.png',dpi=180);plt.close(fig)
sys.path.insert(0,str(ROOT/'双雷达标定工具'));import registration as reg
from calibrate import initial_transform
cfg=read('双雷达标定工具/config/initial_body_estimate.json');anchor=reg.pose(cfg['sensors']['rear_right']);T=np.array(d['T_rear_right_from_front_left']);clouds={}
for role in ['front_left','rear_right']:
 z=np.load(ROOT/'部署准备/lidar_commission_20261125/corner_20261125_02/D'/(role+'.npz'));p=np.concatenate([z['first'],z['second']]);p=p[np.isfinite(p).all(1)];r=np.linalg.norm(p,axis=1);clouds[role]=reg.voxel(p[(r>1)&(r<10)],.04)
fig,axes=plt.subplots(2,2,figsize=(11,10))
for col,M in enumerate([initial_transform(),T]):
 for role,trans,color,label in [('front_left',anchor@M,blue,'前左雷达'),('rear_right',anchor,orange,'后右雷达')]:
  p=reg.apply(clouds[role],trans)
  for row in [0,1]:
   q=p[(p[:,2]>.15)&(p[:,2]<2.5)] if row==0 else p[abs(p[:,1])<3]
   axes[row,col].scatter(q[:,0],q[:,1 if row==0 else 2],s=.45,c=color,alpha=.5,label=label,rasterized=True)
 for row in [0,1]:
  ax=axes[row,col];ax.set_aspect('equal');ax.set_xlim(-6.8,2.8);ax.set_ylim((-1.7,6.6) if row==0 else (-.35,3.7));ax.grid(alpha=.18);ax.set_xlabel('X（m）');ax.set_ylabel('Y（m）' if row==0 else 'Z（m）');ax.legend(markerscale=6,fontsize=9,frameon=False)
 axes[0,col].set_title(['原安装初值','冻结的共同平面候选'][col],fontsize=14,weight='bold')
fig.suptitle('D 组点云对齐对比：俯视与侧视',fontsize=17,weight='bold');fig.text(.07,.025,'蓝：前左；橙：后右。使用旧 CAD 锚点展示，非绝对车体标定；不同视野及遮挡区域不要求重合。\n共同墙面与顶部重影减小，细查地面仍有约 1.6–2.3 cm 分层，候选尚未部署。',fontsize=10)
fig.tight_layout(rect=(0,.08,1,.95));fig.savefig(OUT/'images/02_D组点云对齐对比.png',dpi=170);plt.close(fig)
data={'date':'2026-11-25','training_translation_spread_mm':spread,'training_rotation_spread_deg':[r1['trial_rotation_spread_deg'],r2['trial_rotation_spread_deg'],d['training_scene_difference']['rotation_deg']],'holdout_D':{k:d[k] for k in ['baseline','candidate','status']},'D_rmse_reduction_percent':100*(1-d['candidate']['plane_rmse_m']/d['baseline']['plane_rmse_m']),'ground_height_separation_mm':dict(zip(g['scenes'],ground)),'source_files':['部署准备/lidar_commission_20261125/corner_20261125_01/result/report.json','部署准备/lidar_commission_20261125/corner_20261125_02/result/report.json','标定稳定性复核_20261125/common_planes_candidate_v1/holdout_D/report.json','车体外参核对_20261125/ground_audit.json']}
(OUT/'周报数据依据.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');print(json.dumps(data,ensure_ascii=False,indent=2))
