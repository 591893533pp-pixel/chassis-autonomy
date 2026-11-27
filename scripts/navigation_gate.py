"""Pure checks shared by the navigation supervisor and START-gated wheel process."""
import hashlib,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
CHECKS=('body_anchor','physical_envelope','nearfield_obstacles','ptp_motion','map_and_relocalization','nav2_plan_and_cancel','stop_and_watchdogs')
def configuration_hash():
 h=hashlib.sha256()
 for relative in ['config/nav2.yaml','config/localization_lio.yaml','model/body_geometry.json','mapping_trial/config/extrinsics_active.json']:
  h.update(relative.encode());h.update((ROOT/relative).read_bytes())
 return h.hexdigest()
def require_acceptance(path=None):
 d=json.loads((path or ROOT/'config/acceptance.json').read_text())
 if d.get('schema')!='chassis_nav_acceptance_v1':raise RuntimeError('Invalid acceptance schema')
 pending=[k for k in CHECKS if d.get('checks',{}).get(k,{}).get('passed') is not True or not d['checks'][k].get('evidence')]
 if pending:raise RuntimeError('现场验收尚未完成：'+', '.join(pending))
 if d.get('configuration_sha256')!=configuration_hash():raise RuntimeError('验收对应的配置已变化，需要重新核对')
 return d

def velocity_command(data,now,session,boot):
 if data.get('session')!=session or data.get('boot_id')!=boot:raise RuntimeError('navigation command identity mismatch')
 timestamp=data.get('monotonic');v=data.get('velocity')
 if not isinstance(timestamp,(int,float)) or not math.isfinite(timestamp) or not 0<=now-timestamp<.2:raise RuntimeError('navigation command stale')
 if not isinstance(v,list) or len(v)!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in v):raise RuntimeError('Invalid navigation velocity')
 if math.hypot(v[0],v[1])>.050001 or abs(v[2])>.040001:raise RuntimeError('Navigation speed exceeds commissioned limit')
 return tuple(v)
