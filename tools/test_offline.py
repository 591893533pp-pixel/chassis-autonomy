#!/usr/bin/env python3
"""Run explicit pure geometry/packet/gate suites. No ROS processes or physical CAN sockets."""
from pathlib import Path
import json,os,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[1]
def main():
 env=dict(os.environ,OPENBLAS_NUM_THREADS='2',MPLBACKEND='Agg',PYTHONDONTWRITEBYTECODE='1')
 suites=[('footprint_navigation',ROOT,['-m','unittest','discover','-s','tests','-p','test_*.py','-v']),
 ('calibration',ROOT,['-m','unittest','discover','-s','calibration/tests','-p','test_*.py','-v']),
 ('timing_gates',ROOT/'mapping_trial',['-m','unittest','test_raw_monitor','test_live_gate','test_run_options','-v']),
 ('fusion_coordinates',ROOT/'mapping_trial',['test_geometry.py']),
 ('gamepad_core',ROOT/'control/program/remote',['-m','pytest','-q','test_button_core.py','test_full_manual.py','vendor/test_upstream_core.py'])]
 out=ROOT/'.validation';out.mkdir(exist_ok=True);results=[]
 for name,cwd,args in suites:
  t=time.monotonic()
  with (out/(name+'.log')).open('w') as f:p=subprocess.run([sys.executable,*args],cwd=cwd,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=180)
  row={'suite':name,'passed':p.returncode==0,'seconds':round(time.monotonic()-t,2),'log':str((out/(name+'.log')).relative_to(ROOT))};results.append(row);print(json.dumps(row),flush=True)
  if p.returncode:print((out/(name+'.log')).read_text()[-5000:])
 (out/'offline_tests.json').write_text(json.dumps({'hardware_started':False,'results':results},indent=2)+'\n')
 return 0 if all(x['passed'] for x in results) else 1
if __name__=='__main__':raise SystemExit(main())
