#!/usr/bin/env python3
"""Reusable dual MID360S relative calibration. Never enables chassis or installs a result."""
import argparse, copy, hashlib, json, pathlib, re, subprocess, sys
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import registration as reg
ROOT=pathlib.Path(__file__).resolve().parent

def read(path): return json.loads(pathlib.Path(path).read_text())
def write(path,data): pathlib.Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def signature(path):return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
def session_path(value):
 p=pathlib.Path(value).expanduser()
 return p.resolve() if p.is_absolute() else (ROOT/'sessions'/p).resolve()
def label(value):
 if not re.fullmatch(r'[A-Za-z0-9_-]{1,48}',value): raise argparse.ArgumentTypeError('名称只能使用英文字母、数字、下划线、短横线，最多48字符')
 return value

def initial_transform():
 s=read(ROOT/'config/settings.json');c=read(ROOT/'config/initial_body_estimate.json')
 return np.linalg.inv(reg.pose(c['sensors'][s['anchor']]))@reg.pose(c['sensors'][s['moving']])

def evaluate(source,target,transform):
 """Symmetric nearest-neighbor planar residuals; report overlap to avoid cherry-picking."""
 s=reg.voxel(reg.apply(source,transform),.04);t=reg.voxel(target,.04)
 all_res=[];fractions=[];counts=[]
 for a,b in [(s,t),(t,s)]:
  n,g=reg.normals(b);d,idx=cKDTree(b).query(a,workers=2)
  keep=(d<.15)&g[idx]
  residual=np.sum((a[keep]-b[idx[keep]])*n[idx[keep]],axis=1)
  all_res.extend(residual.tolist());fractions.append(float(np.mean(d<.1)));counts.append(int(keep.sum()))
 r=np.asarray(all_res)
 if not len(r):raise ValueError('没有共同平面，不能评价')
 return {'matched_planar_points':min(counts),'plane_rmse_m':float(np.sqrt(np.mean(r*r))),
         'plane_abs_p95_m':float(np.percentile(abs(r),95)), 'mutual_overlap_within_10cm':min(fractions)}

def capture_data(folder,settings):
 m=read(folder/'metadata.json')
 if not m.get('health_ok'):
  from static_policy import qualify_static
  review=qualify_static(m,read(folder/'diagnostics.json'),settings)
  if not review['eligible']:raise ValueError(f'{folder.name}: 不能用于静止标定：'+str(review['errors']))
  m=dict(m,static_review=review)
 if m['sensors']!=settings['sensors']:raise ValueError(f'{folder.name}: 雷达身份与当前设置不一致')
 if m.get('settings_sha256')!=signature(ROOT/'config/settings.json'):raise ValueError('采集后settings.json变化，请使用匹配配置或重新采集')
 out={}
 for role in settings['sensors']:
  d=np.load(folder/(role+'.npz'))
  # Estimate motion between temporal halves; IMU alone cannot detect steady translation.
  halves=[]
  for key in ('first','second'):
   points=d[key];r=np.linalg.norm(points,axis=1)
   halves.append(reg.voxel(points[np.isfinite(points).all(axis=1)&(r>1)&(r<10)],.06))
  motion,history=reg.fit(halves[0],halves[1],np.eye(4))
  angle=float(np.degrees(Rotation.from_matrix(motion[:3,:3]).magnitude()))
  if np.linalg.norm(motion[:3,3])>.015 or angle>.3:
   raise ValueError(f'{folder.name}/{role}: 前后半段发生移动或场景变化，请重采')
  if min(history[-1]['normalized_information_eigenvalues'])<settings['limits']['min_normalized_information_eigenvalue']:
   raise ValueError(f'{folder.name}/{role}: 场景方向约束不足，请让墙角和地面进入视野')
  p=np.concatenate([d['first'],d['second']])
  p=p[np.isfinite(p).all(axis=1)];r=np.linalg.norm(p,axis=1)
  p=reg.voxel(p[(r>1)&(r<10)],.04)
  if len(p)<3000:raise ValueError(f'{folder.name}/{role}: 有效点不足')
  out[role]=p
 if m.get('static_review'):
  review=dict(m['static_review'],pointcloud_stationarity_passed=True,
              evidence_sha256={f:signature(folder/f) for f in ['metadata.json','diagnostics.json']+[role+'.npz' for role in settings['sensors']]})
  write(folder/'static_geometry_review.json',review)
 return m,out

def solve(args):
 settings=read(ROOT/'config/settings.json');limits=settings['limits'];session=session_path(args.session)
 if len(args.train)<2:raise ValueError('至少两个不同位置的训练采集，加一个独立验证采集')
 names=args.train+args.validation
 if len(set(names))!=len(names):raise ValueError('训练和验证必须使用不同的采集目录，名称不可重复')
 out=session/'result'
 if out.exists():raise ValueError('result 已存在，为保留历史请换一个会话名，或先重命名旧 result')
 captures={};meta={};hashes=set()
 for name in names:
  folder=session/label(name);meta[name],captures[name]=capture_data(folder,settings)
  if not meta[name].get('pose_note','').strip():raise ValueError('每组必须记录实际位置/朝向')
  for role in settings['sensors']:
   h=signature(folder/(role+'.npz'))
   if h in hashes:raise ValueError('发现重复点云文件，不能当作独立采集')
   hashes.add(h)
 if len({m['pose_note'].strip() for m in meta.values()})!=len(meta):raise ValueError('各组请记录不同的实际位置/朝向，不能复用同一姿态说明')
 moving,anchor=settings['moving'],settings['anchor'];initial=initial_transform()
 # Multiple stationary scenes are fitted separately, preventing unrelated scene surfaces from matching.
 trials=[]
 for yaw in (0,-2,2):
  init=initial.copy();init[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@init[:3,:3]
  for name in args.train:
   t,hist=reg.fit(captures[name][moving],captures[name][anchor],init)
   trials.append({'scene':name,'initial_yaw_deg':yaw,'transform':t.tolist(),'history':hist})
   print(f"拟合 {name} / 初值偏航{yaw:+}°：平面RMSE {hist[-1]['planar_residual_rmse_m']*100:.2f} cm",flush=True)
 # Select on all training scenes, never on validation scenes.
 for trial in trials:
  trial['train_scores']={n:evaluate(captures[n][moving],captures[n][anchor],np.array(trial['transform'])) for n in args.train}
  trial['score']=float(np.mean([v['plane_rmse_m'] for v in trial['train_scores'].values()]))
 best=min(trials,key=lambda x:x['score']);t=np.array(best['transform'])
 spread_t=max(float(np.linalg.norm(np.array(v['transform'])[:3,3]-t[:3,3])) for v in trials)
 spread_r=max(float(np.degrees(Rotation.from_matrix(np.array(v['transform'])[:3,:3]@t[:3,:3].T).magnitude())) for v in trials)
 correction=t@np.linalg.inv(initial)
 angle=float(np.degrees(Rotation.from_matrix(correction[:3,:3]).magnitude()))
 shift=float(np.linalg.norm(correction[:3,3]));reasons=[]
 if spread_t>limits['max_trial_translation_spread_m'] or spread_r>limits['max_trial_rotation_spread_deg']:reasons.append('不同场景/初值的解不一致')
 if angle>limits['max_initial_correction_angle_deg'] or shift>limits['max_initial_correction_translation_m']:reasons.append('修正超出旧安装初值的适用范围，需人工核对安装')
 for trial in trials:
  if min(trial['history'][-1]['normalized_information_eigenvalues'])<limits['min_normalized_information_eigenvalue']:reasons.append('共同平面方向不足，六自由度约束退化');break
 validation={}
 for name in args.validation:
  baseline=evaluate(captures[name][moving],captures[name][anchor],initial)
  after=evaluate(captures[name][moving],captures[name][anchor],t)
  validation[name]={'baseline':baseline,'candidate':after}
  if after['matched_planar_points']<limits['min_matches']:reasons.append(f'{name}: 共同平面匹配点不足')
  if after['mutual_overlap_within_10cm']<limits['min_mutual_overlap']:reasons.append(f'{name}: 共同视野不足')
  if after['plane_rmse_m']>limits['max_plane_rmse_m'] or after['plane_abs_p95_m']>limits['max_plane_p95_m']:reasons.append(f'{name}: 验证残差超出门限')
  if after['plane_rmse_m']>baseline['plane_rmse_m']+.003:reasons.append(f'{name}: 相对原参数明显变差')
 passed=not reasons
 report={'status':'RELATIVE_CHECKS_PASS_REVIEW_REQUIRED' if passed else 'REJECTED',
         'scope':'Static relative LiDAR alignment only; absolute body pose and dynamic deskew not certified.',
         'convention':f'p_{anchor} = R @ p_{moving} + t; meters; column vectors; rpy = scipy xyz degrees',
         'T_rear_right_from_front_left':t.tolist(),'translation_m':t[:3,3].tolist(),
         'rpy_deg':Rotation.from_matrix(t[:3,:3]).as_euler('xyz',degrees=True).tolist(),
         'correction_angle_deg':angle,'correction_translation_m':shift,
         'trial_translation_spread_m':spread_t,'trial_rotation_spread_deg':spread_r,
         'validation':validation,'reasons':reasons,'limits':limits,'trials':trials,
         'captures':{n:{'pose_note':meta[n]['pose_note'],'capture_id':meta[n]['capture_id'],'static_review':meta[n].get('static_review')} for n in names},
         'settings_sha256':signature(ROOT/'config/settings.json'),'initial_sha256':signature(ROOT/'config/initial_body_estimate.json')}
 out.mkdir();write(out/'report.json',report)
 write(out/'relative_transform.json',{'parent':'rear_right_optical','child':'front_left_optical','T_parent_from_child':t.tolist(),'translation_m':report['translation_m'],'rpy_deg_xyz':report['rpy_deg'],'checks_passed':passed,'review_required':True})
 body=read(ROOT/'config/initial_body_estimate.json');newpose=reg.pose(body['sensors'][anchor])@t
 body['sensors'][moving].update(xyz_m=newpose[:3,3].tolist(),rpy_deg=Rotation.from_matrix(newpose[:3,:3]).as_euler('xyz',degrees=True).tolist())
 for role in settings['sensors']:body['sensors'][role]['ip']=settings['sensors'][role]['ip']
 body.update(calibrated=False,relative_calibrated=False,calibration_report='report.json',independent_validation=args.validation,
             note='NOT DEPLOYED. Automated relative checks only; review plots. Rear-right body anchor is unverified CAD.',anchor='rear_right unverified CAD; no absolute body calibration',source='dual_lidar_calibration toolkit')
 write(out/'body_candidate_NOT_DEPLOYED.json',body)
 plot(captures,args.validation,initial,t,out)
 print(json.dumps({k:report[k] for k in ['status','translation_m','rpy_deg','validation','reasons']},ensure_ascii=False,indent=2))
 print('报告：',out/'report.json','\n不会自动覆盖导航/建图配置。')
 return 0 if passed else 2

def plot(captures,names,initial,t,out):
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 # Display in old rear-right body anchor for understandable top/side views, not an absolute calibration.
 anchor=reg.pose(read(ROOT/'config/initial_body_estimate.json')['sensors']['rear_right'])
 for name in names:
  fig,axes=plt.subplots(2,2,figsize=(12,10))
  for col,trans in enumerate([initial,t]):
   for role,cloud,color in [('front_left',reg.apply(captures[name]['front_left'],anchor@trans),'#087fce'),('rear_right',reg.apply(captures[name]['rear_right'],anchor),'#ed7518')]:
    q=cloud[(cloud[:,2]>.15)&(cloud[:,2]<2.5)]
    axes[0,col].scatter(q[:,0],q[:,1],s=.4,c=color,alpha=.5,label=role)
    q=cloud[abs(cloud[:,1])<3]
    axes[1,col].scatter(q[:,0],q[:,2],s=.4,c=color,alpha=.5,label=role)
   axes[0,col].set_title(['Initial','Candidate - not deployed'][col])
   for row in [0,1]:
    ax=axes[row,col];ax.set_aspect('equal');ax.set_xlim(-7,7);ax.set_ylim((-7,7) if row==0 else (-.5,4));ax.grid(alpha=.2);ax.legend(markerscale=8);ax.set_xlabel('X (m)');ax.set_ylabel(['Y (m)','Z (m)'][row])
  fig.suptitle('Independent validation '+name+' / approximate body display frame');fig.tight_layout();fig.savefig(out/f'validation_{name}.png',dpi=150);plt.close(fig)

def main():
 p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command')
 r=sub.add_parser('review-static',help='只读复核已有采集；另存静止几何资格报告，保留原始失败记录')
 r.add_argument('session');r.add_argument('label',type=label)
 sub.add_parser('check',help='只读检查地址、依赖、PTP和冲突进程；不启动雷达')
 c=sub.add_parser('capture',help='静止采集；自动启动/关闭本次雷达驱动，不启动车体')
 c.add_argument('session');c.add_argument('label',type=label);c.add_argument('--pose-note',required=True)
 f=sub.add_parser('fit',help='离线拟合和独立场景验证；不修改运行配置')
 f.add_argument('session');f.add_argument('--train',nargs='+',required=True,type=label);f.add_argument('--validation',nargs='+',required=True,type=label)
 args=p.parse_args()
 if args.command=='check':
  from capture import preflight
  preflight(read(ROOT/'config/settings.json'));print('预检通过。设备内部健康/点云/序列号将在采集时验证。');return 0
 if args.command=='capture':
  from capture import run
  return run(args,session_path(args.session),ROOT)
 if args.command=='review-static':
  folder=session_path(args.session)/args.label
  m,points=capture_data(folder,read(ROOT/'config/settings.json'))
  print(json.dumps({'static_geometry_eligible':True,'original_health_ok':m['health_ok'],'warnings':m.get('static_review',{}).get('warnings',[]),'points_1_to_10m':{k:len(v) for k,v in points.items()},'next':'Still needs independent scene validation; not navigation acceptance'},ensure_ascii=False,indent=2));return 0
 if args.command=='fit':return solve(args)
 p.print_help();return 0
if __name__=='__main__':
 try:sys.exit(main())
 except KeyboardInterrupt:print('已中断。',file=sys.stderr);sys.exit(130)
 except Exception as exc:print('失败：'+str(exc),file=sys.stderr);sys.exit(1)
