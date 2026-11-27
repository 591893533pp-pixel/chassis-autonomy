import json,math,sys,subprocess
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from lift_policy import LiftPolicy,COUNTS_MM
from remote_session import WheelFeedback
@pytest.mark.parametrize('v',[0,-1,150.001,float('inf'),float('nan')])
def test_speed_rejected(v):
    with pytest.raises(ValueError):LiftPolicy(v)
def test_ceiling_quantization():
    g=LiftPolicy(50)
    assert 49.9<g.target(1,100)/COUNTS_MM<=50
    assert abs(LiftPolicy(50).target(-1,100))/COUNTS_MM<=50
@pytest.mark.parametrize('position',[-1.01,396.01,float('nan')])
def test_position_reject(position):
    with pytest.raises(RuntimeError):LiftPolicy(10).target(1,position)
def test_endpoints_and_release():
    g=LiftPolicy(50);assert g.target(1,394.6)==0
    assert g.target(-1,394.6)==0
    assert g.target(0,394.6)==0
    assert g.target(-1,394.6)<0
    assert abs(LiftPolicy(50).target(-1,1))<abs(LiftPolicy(50).target(-1,100))
def test_both_and_reverse():
    g=LiftPolicy(10);assert g.target(1,100)>0
    assert g.target(-1,100)==0
    assert g.target(1,100)==0
    g.target(0,100);assert g.target(2,100)==0
    assert g.target(1,100)==0
    g.target(0,100);assert g.target(1,100)>0
def test_missing_feedback_blocks(tmp_path):
    assert not WheelFeedback(tmp_path/'absent.csv').stationary()
def test_no_hardware_preview():
    r=subprocess.run([str(ROOT/'start.sh'),'--dry-run','--lift-speed-mm-s','50'],capture_output=True,text=True)
    assert r.returncode==0
    d=json.loads(r.stdout);assert not d['hardware_started'] and d['lift_max_mm_s']==150
