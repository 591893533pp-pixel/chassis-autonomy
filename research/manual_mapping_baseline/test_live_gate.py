import copy,unittest
from live_gate import validate
from timing_policy import POLICY,IPS
class Tests(unittest.TestCase):
 def setUp(self):
  self.d={'schema':POLICY,'boot_id':'boot','generation':'monitor','monotonic':10.,'ready':True,'fault':None,'capture_stats':{'drops':0},'streams':{f'{ip}:{port}':{'seen':10.,'age_ns':1_000_000} for ip in IPS for port in (56300,56400)}}
 def check(self,d,now=10.01):return validate(d,now,'boot','monitor')
 def test_healthy(self):self.assertEqual(self.check(self.d),self.d)
 def test_dead_monitor_and_future_time(self):
  for now in [10.16,9.]:
   with self.assertRaises(RuntimeError):self.check(self.d,now)
 def test_restart_or_fault_invalidates(self):
  for k,v in [('boot_id','newboot'),('generation','newmonitor'),('ready',False),('fault','sequence_gap')]:
   d=copy.deepcopy(self.d);d[k]=v
   with self.assertRaises(RuntimeError):self.check(d)
 def test_missing_stale_clock_and_drops(self):
  key=next(iter(self.d['streams']))
  for field,value in [('seen',9.),('age_ns',6_000_000),('age_ns',float('nan')),('seen',True)]:
   d=copy.deepcopy(self.d);d['streams'][key][field]=value
   with self.assertRaises(RuntimeError):self.check(d)
  d=copy.deepcopy(self.d);del d['streams'][key]
  with self.assertRaises(RuntimeError):self.check(d)
  d=copy.deepcopy(self.d);d['capture_stats']['drops']=1
  with self.assertRaises(RuntimeError):self.check(d)
if __name__=='__main__':unittest.main()
