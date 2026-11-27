"""Narrow retrospective qualification for stationary geometry, never navigation health.

Only HMS 0x04050002 (official: synchronization exception occurred in the past)
may be retained as an advisory. All live capture failures remain fatal.
Original metadata and raw diagnostics are never rewritten by review.
"""
import math
HISTORICAL_SYNC=0x04050002
POLICY='stationary_geometry_history_review_v1'

def qualify_static(meta,diagnostics,settings):
 errors=[];warnings=[];duration=meta.get('capture_seconds',0)
 def need(ok,reason):
  if not ok:errors.append(reason)
 def finite(v):return isinstance(v,(float,int)) and not isinstance(v,bool) and math.isfinite(v)
 allowed={role+':HMS告警 0x04050002' for role in settings['sensors']}
 original=meta.get('errors',[])
 need(bool(original) and set(original)<=allowed,'原采集存在历史0405以外的错误，或缺少可复核的拒绝原因')
 need(20<=duration<=60,'采集时长无效')
 need(meta.get('sensors')==settings['sensors'],'身份配置不一致')
 data=diagnostics.get('diagnostics',[]);sync=diagnostics.get('sync',[])
 need(len(sync)>=duration*.6,'包同步样本不足')
 for role,device in settings['sensors'].items():
  rows=[v.get('info',{}) for v in data if v.get('ip')==device['ip']]
  need(len(rows)>=duration*.6,role+':设备诊断样本不足')
  times=[v.get('local_time_now',0) for v in rows]
  if len(times)>1:
   gaps=[(b-a)/1e9 for a,b in zip(times,times[1:])]
   need((times[-1]-times[0])/1e9>=duration-4 and all(0<g<=2.5 for g in gaps),role+':设备诊断不连续/覆盖不足')
  for info in rows:
   need(info.get('sn')==device['serial'] and info.get('cur_work_state')==1 and info.get('lidar_diag_status')==0,role+':设备或身份异常')
   codes=info.get('hms_code')
   need(isinstance(codes,list) and all(c in (0,HISTORICAL_SYNC) for c in codes),role+':存在其他HMS或HMS缺失')
   if isinstance(codes,list) and HISTORICAL_SYNC in codes:warnings.append(role+':保留0x04050002历史同步告警')
   offset=info.get('time_offset');age=info.get('local_time_now',0)-info.get('last_sync_time',0)
   need(info.get('time_sync_type')==1 and finite(offset) and abs(offset)<=settings['max_device_offset_ns'] and 0<=age<3_000_000_000,role+':实时设备同步不合格')
  for row in sync:
   v=row.get(device['ip'],{});age=v.get('packet_age_ms')
   need(v.get('time_type')==1 and finite(age) and 0<=age<500,role+':包时间类型/新鲜度不合格')
  for kind,rate,gap in [('cloud',8,.3),('imu',150,.1)]:
   st=meta.get('stats',{}).get(role,{}).get(kind,{})
   need(all(finite(st.get(k)) for k in ('count','max_gap_s','min_age_s','max_age_s')),role+':流统计缺失/无效')
   if all(finite(st.get(k)) for k in ('count','max_gap_s','min_age_s','max_age_s')):
    need(st['count']>=duration*rate and st.get('backward')==0 and st['max_gap_s']<=gap and st['min_age_s']>=-.05 and st['max_age_s']<=.5,role+':频率/时间戳不合格')
  stat=meta.get('stationarity',{}).get(role,{})
  need(finite(stat.get('gyro_norm_p95')) and stat['gyro_norm_p95']<=.04 and finite(stat.get('acceleration_std_norm')) and stat['acceleration_std_norm']<=.025,role+':IMU静止性不合格')
 return {'policy':POLICY,'scope':'stationary relative geometry only; NOT navigation/PTP system acceptance',
         'eligible':not errors,'warnings':sorted(set(warnings)),'errors':sorted(set(errors)),
         'note':'Historical warning onset and root cause remain unknown. Existing capture recorded no host/stream/cleanup failures beyond the exact HMS warning. Point-cloud stationarity and independent-scene validation are still required.'}
