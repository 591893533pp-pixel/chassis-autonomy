import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from remote_session import route_motion

def command(x=.05,y=.02,yaw=.03):
    return SimpleNamespace(x=x,y=y,yaw=yaw)

@pytest.mark.parametrize('direction',[-1,1,2])
def test_simultaneous_preserves_both_requests(direction):
    xyz,lift,block=route_motion(command(),direction,True,True,False,True)
    assert xyz==(.05,.02,.03) and lift==direction and not block

def test_releasing_one_axis_does_not_stop_other():
    assert route_motion(command(),0,True,True,False,True)==((.05,.02,.03),0,False)
    assert route_motion(command(0,0,0),1,True,True,False,True)==((0,0,0),1,False)

def test_waits_for_lift_arming_before_chassis_motion():
    assert route_motion(command(),1,True,False,True,True)[0]==(0,0,0)

def test_legacy_exclusive_mode_still_interlocks():
    assert route_motion(command(),1,True,True,False,False)==((0,0,0),0,True)
    assert route_motion(command(),0,True,True,False,False)[2]

def test_chassis_only_does_not_require_lift_arming():
    assert route_motion(command(),0,False,False,True,False)==((.05,.02,.03),0,False)

@pytest.mark.parametrize('flags,expected',[([],True),(['--exclusive-motion'],False),(['--chassis-only'],False)])
def test_entry_mode_without_hardware(flags,expected):
    p=subprocess.run([str(ROOT/'start.sh'),'--dry-run',*flags],check=True,text=True,capture_output=True)
    data=json.loads(p.stdout)
    assert data['simultaneous_motion']==expected and not data['hardware_started']
    assert data['lift_range_mm']==[0,395] and data['lift_max_mm_s']==150
