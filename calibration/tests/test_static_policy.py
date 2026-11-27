import copy,pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from static_policy import qualify_static,HISTORICAL_SYNC
class StaticPolicyTest(unittest.TestCase):
 def fixture(self):
  settings={'sensors':{'front_left':{'ip':'192.168.1.132','serial':'test'}},'max_device_offset_ns':500000}
  stat={'count':6000,'backward':0,'max_gap_s':.006,'min_age_s':0,'max_age_s':.12}
  meta={'capture_seconds':30,'sensors':settings['sensors'],'errors':['front_left:HMS告警 0x04050002'],'stats':{'front_left':{'cloud':dict(stat,count=300,max_gap_s=.101),'imu':stat}},'stationarity':{'front_left':{'gyro_norm_p95':.01,'acceleration_std_norm':.003}}}
  rows=[]
  for i in range(30):
   rows.append({'ip':'192.168.1.132','info':{'sn':'test','cur_work_state':1,'lidar_diag_status':0,'hms_code':[HISTORICAL_SYNC,0],'local_time_now':(100+i)*10**9,'last_sync_time':(100+i)*10**9-500000000,'time_offset':80000,'time_sync_type':1}})
  diag={'diagnostics':rows,'sync':[{'192.168.1.132':{'time_type':1,'packet_age_ms':1}} for _ in range(30)]}
  return meta,diag,settings
 def test_history_only_is_scoped_advisory(self):
  m,d,s=self.fixture();original=copy.deepcopy(m);r=qualify_static(m,d,s)
  self.assertTrue(r['eligible']);self.assertTrue(r['warnings']);self.assertEqual(m,original)
 def test_current_failure_never_bypassed(self):
  mutations=[
   lambda m,d:m['errors'].append('采集中PTP主钟重启'),
   lambda m,d:d['diagnostics'][10]['info'].update(time_offset=500001),
   lambda m,d:d['diagnostics'][10]['info'].update(time_sync_type=0),
   lambda m,d:d['diagnostics'][10]['info'].update(hms_code=[HISTORICAL_SYNC,0x04020002]),
   lambda m,d:d['diagnostics'][10]['info'].update(cur_work_state=4),
   lambda m,d:d['sync'][10]['192.168.1.132'].update(time_type=0),
   lambda m,d:m['stats']['front_left']['cloud'].update(backward=1),
   lambda m,d:m['stats']['front_left']['imu'].update(max_age_s=float('nan')),
   lambda m,d:m['stationarity']['front_left'].update(gyro_norm_p95=.05),
   lambda m,d:d.update(sync=[]),
   lambda m,d:d['diagnostics'][10]['info'].update(local_time_now=0),
  ]
  for i,mutation in enumerate(mutations):
   with self.subTest(case=i):
    m,d,s=self.fixture();mutation(m,d);self.assertFalse(qualify_static(m,d,s)['eligible'])
if __name__=='__main__':unittest.main()
