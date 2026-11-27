"""Exercise the real native command gate in dry-run mode (no USB/CAN)."""
import csv
from pathlib import Path
import subprocess
import time
import pytest
BRIDGE=Path(__file__).resolve().parents[1]/'build/remote_bridge'

def launch(tmp_path):
 return subprocess.Popen([str(BRIDGE),'--log',str(tmp_path/'feedback.csv')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
def send(p, enabled=0, x=0, y=0, yaw=0, lift=0):
 p.stdin.write(f'{time.monotonic()} {enabled} {x} {y} {yaw} {lift}\n');p.stdin.flush()
def test_watchdog(tmp_path):
 p=launch(tmp_path);send(p,1);time.sleep(.08);send(p,1,.05)
 assert p.wait(timeout=3)==1
 assert 'watchdog expired' in p.stderr.read()
def test_non_neutral_enable(tmp_path):
 p=launch(tmp_path);send(p,1,.05)
 assert p.wait(timeout=3)==1
 assert 'first enable must be neutral' in p.stderr.read()
# Input gate is |v|<=1.500001 m/s since the 1.5 m/s policy (was 0.700001).
@pytest.mark.parametrize('payload',['nan 1 0 0 0 0','1 1 1.51 0 0 0','1 1 1.1 1.1 0 0','1 1 0 0 0.401 0','1 1 0 0 -0.401 0','1 2 0 0 0 0','1 1 0 0 0 0 extra'])
def test_invalid_input(tmp_path,payload):
 p=launch(tmp_path);p.stdin.write(payload+'\n');p.stdin.flush()
 assert p.wait(timeout=3)==1
 assert 'invalid command' in p.stderr.read()
def test_velocity_and_stop(tmp_path):
 p=launch(tmp_path);send(p,1);time.sleep(.06)
 for _ in range(8):send(p,1,.05);time.sleep(.03)
 for _ in range(5):send(p,0);time.sleep(.03)
 p.terminate();assert p.wait(timeout=3)==0
 rows=list(csv.DictReader((tmp_path/'feedback.csv').open()))
 assert any(float(r['applied_vx'])>0 for r in rows)
 assert float(rows[-1]['applied_vx'])==0
 assert rows[-1]['feedback_valid']=='0'
 assert all(abs(float(r[f'wheel{i}_cmd_rad_s']))<=1 for r in rows for i in range(1,5))
def test_eof_stops(tmp_path):
 p=launch(tmp_path);send(p);p.stdin.close()
 assert p.wait(timeout=3)==1
 assert 'pipe closed' in p.stderr.read()

@pytest.mark.parametrize('direction',[-1,1])
def test_manual_yaw_reaches_native_bridge_and_stops(tmp_path,direction):
 # Exercise Python manual policy through the real native parser, without CAN.
 from run_remote import manual_config
 from vendor.navslam_core import TeleopCore
 from test_button_core import arm,feed
 core=TeleopCore(manual_config(1.5,1));arm(core)
 command=feed(core,.1,{2:direction})
 assert command.yaw==pytest.approx(direction*.4)
 p=launch(tmp_path)
 try:
  send(p,1);time.sleep(.06)
  for _ in range(8):
   send(p,1,command.x,command.y,command.yaw);time.sleep(.03)
  stopped=feed(core,.2)
  for _ in range(4):
   send(p,1,stopped.x,stopped.y,stopped.yaw);time.sleep(.03)
  send(p,0);time.sleep(.04)
 finally:
  if p.poll() is None:p.terminate()
 assert p.wait(timeout=3)==0,p.stderr.read()
 rows=list(csv.DictReader((tmp_path/'feedback.csv').open()))
 assert any(float(r['applied_wz'])==pytest.approx(command.yaw) for r in rows)
 assert float(rows[-1]['applied_wz'])==0
 assert all(float(rows[-1][f'wheel{i}_cmd_rad_s'])==0 for i in range(1,5))
 assert all(r['hardware']=='0' for r in rows)

@pytest.mark.parametrize('x,y,yaw,signs', [
 (.02,0,0,(1,-1,-1,1)),(-.02,0,0,(-1,1,1,-1)),
 (0,.02,0,(-1,-1,1,1)),(0,-.02,0,(1,1,-1,-1)),
 (0,0,.04,(1,1,1,1)),(0,0,-.04,(-1,-1,-1,-1))])
def test_physical_direction_mapping(tmp_path,x,y,yaw,signs):
 p=launch(tmp_path);send(p,1);time.sleep(.06)
 for _ in range(5):send(p,1,x,y,yaw);time.sleep(.03)
 p.terminate();assert p.wait(timeout=3)==0
 rows=list(csv.DictReader((tmp_path/'feedback.csv').open()))
 moving=[r for r in rows if any(abs(float(r[k]))>0 for k in ('applied_vx','applied_vy','applied_wz'))]
 assert moving
 for r in moving:
  assert float(r['applied_vx'])==pytest.approx(x)
  assert float(r['applied_vy'])==pytest.approx(y)
  assert float(r['applied_wz'])==pytest.approx(yaw)
  for i,sign in enumerate(signs,1):assert float(r[f'wheel{i}_cmd_rad_s'])*sign>0

def test_metric_lift_config_reaches_transport_without_hardware(tmp_path):
 # Nonexistent socket interface: validation must pass, but no CAN can be opened/sent.
 p=subprocess.run([str(BRIDGE),'--hardware','socketcan:no_such_can','--lift',
                   '--log',str(tmp_path/'lift.csv')],capture_output=True,text=True,timeout=3)
 assert p.returncode==1
 assert 'interface not found' in p.stderr
 assert 'lift_travel_counts' not in p.stderr
 assert 'DISABLE FAILED' not in p.stderr


def test_diagnostic_no_hold_flag_keeps_default_explicit(tmp_path):
 for flag,expected in [([], 'Body hold ON'), (['--diagnostic-no-body-hold'], 'Body hold OFF (diagnostic)')]:
  p=subprocess.Popen([str(BRIDGE),*flag,'--log',str(tmp_path/'mode.csv')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
  send(p,0);time.sleep(.08);p.terminate()
  assert p.wait(timeout=3)==0
  log=p.stderr.read()
  assert expected in log
  assert 'acceleration shaping retained' in log


@pytest.mark.parametrize('speed',[.5,.7])
def test_expanded_velocity_request_and_stop(tmp_path,speed):
 p=launch(tmp_path);send(p,1);time.sleep(.06)
 for _ in range(5):send(p,1,speed);time.sleep(.03)
 for _ in range(4):send(p,0);time.sleep(.03)
 p.terminate();assert p.wait(timeout=3)==0
 rows=list(csv.DictReader((tmp_path/'feedback.csv').open()))
 assert any(float(r['applied_vx'])==pytest.approx(speed) for r in rows)
 assert all(abs(float(r[f'wheel{i}_cmd_rad_s']))<=10 for r in rows for i in range(1,5))
 assert all(float(rows[-1][f'wheel{i}_cmd_rad_s'])==0 for i in range(1,5))


def test_diagnostic_no_wheel_pi_flag(tmp_path):
 p=subprocess.Popen([str(BRIDGE),'--diagnostic-no-body-hold','--diagnostic-no-wheel-pi','--log',str(tmp_path/'open.csv')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
 send(p,0);time.sleep(.08);p.terminate()
 assert p.wait(timeout=3)==0
 log=p.stderr.read()
 assert 'Wheel PI OFF (diagnostic; motor velocity mode retained)' in log
 assert 'Body hold OFF' in log

@pytest.mark.parametrize('profile',['baseline','smooth-candidate','integral-gate'])
def test_pi_profile_logged_without_hardware(tmp_path,profile):
 p=subprocess.Popen([str(BRIDGE),'--wheel-pi-profile',profile,'--log',str(tmp_path/'profile.csv')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
 send(p,0);time.sleep(.10);p.terminate()
 assert p.wait(timeout=3)==0
 assert 'Wheel PI profile '+profile in p.stderr.read()
 rows=list(csv.DictReader((tmp_path/'profile.csv').open()))
 assert rows and all(r['wheel_pi_profile']==profile for r in rows)
 assert all(r['hardware']=='0' and r['wheel_balance_active']=='0' for r in rows)

def test_integral_gate_diagnostics_every_cycle_without_hardware(tmp_path):
 p=subprocess.Popen([str(BRIDGE),'--wheel-pi-profile','integral-gate','--log-every-cycle',
                     '--log',str(tmp_path/'gate.csv')],stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
 for _ in range(7):send(p,0);time.sleep(.03)
 p.terminate();assert p.wait(timeout=3)==0
 diagnostic=p.stderr.read()
 assert 'normal kp=0.2 ki=0.3' in diagnostic and 'Integral gate ON' in diagnostic
 rows=list(csv.DictReader((tmp_path/'gate.csv').open()))
 assert len(rows)>=7
 for r in rows:
  assert r['wheel_pi_profile']=='integral-gate' and r['hardware']=='0'
  for i in range(1,5):
   for k in ['p_term_rad_s','i_term_rad_s','integral_enabled','stable_s','target_rate_rad_s2']:
    assert float(r[f'wheel{i}_{k}'])==0

def test_unknown_pi_profile_rejected_before_transport(tmp_path):
 p=subprocess.run([str(BRIDGE),'--wheel-pi-profile','typo','--log',str(tmp_path/'bad.csv')],capture_output=True,text=True,timeout=3)
 assert p.returncode==2 and 'invalid wheel PI profile' in p.stderr
