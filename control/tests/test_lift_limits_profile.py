import io
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from lift_profile import VelocityProfile
from lift_policy import LiftPolicy,COUNTS_MM
from lift_limits import LIMITS,MAX_POSITION_MM
from remote_session import record_and_validate

@pytest.mark.parametrize('target',[10.,50.,150.,-150.])
def test_velocity_ramp_acceleration_and_jerk(target):
    p=VelocityProfile();last_a=0.;last_v=0.
    for i in range(200):
        dt=[.02,.025,.033][i%3]
        v=p.step(target,dt)
        assert abs(v)<=abs(target)+1e-9
        assert abs(v-last_v)/dt<=300+1e-7
        assert abs(p.acceleration)<=300+1e-7
        assert abs(p.acceleration-last_a)/dt<=1500+1e-6
        last_a=p.acceleration;last_v=v
    assert v==pytest.approx(target)

def test_release_overrides_smoothing():
    p=VelocityProfile()
    for _ in range(40):p.step(150,.025)
    assert p.step(0,.025)==0 and p.acceleration==0

def test_invalid_time_rejected():
    p=VelocityProfile()
    for dt in [0,-1,.301,float('nan')]:
        with pytest.raises(RuntimeError):p.step(100,dt)

def test_new_range_and_speed_and_csp_inactive():
    assert MAX_POSITION_MM==395 and not LIMITS['csp_parameters_active']
    assert LIMITS['lift_csp_max_step_mm']==2
    assert LIMITS['lift_csp_following_error_mm']==25
    assert LIMITS['lift_csp_following_error_timeout_ms']==20
    g=LiftPolicy(150)
    assert g.target(1,300)/COUNTS_MM==pytest.approx(150)
    assert g.target(1,394.6)==0
    with pytest.raises(RuntimeError):g.target(1,396.001)
    with pytest.raises(ValueError):LiftPolicy(150.001)

def test_feedback_validation_accepts_new_range_but_not_overrun():
    state=dict(statusword=0x1637,mode=3,position_mm=300.,velocity_counts_s=0)
    record_and_validate(io.StringIO(),lambda *a,**k:None,state,0,0,position_max_mm=395)
    state['position_mm']=395.01
    with pytest.raises(RuntimeError):
        record_and_validate(io.StringIO(),lambda *a,**k:None,state,0,0,position_max_mm=395)


def test_feedback_fault_margin_preserves_directional_bounds():
    for pos in (-1.,-.01,395.01,396.):
        state=dict(statusword=0x1637,mode=3,position_mm=pos,velocity_counts_s=0)
        record_and_validate(io.StringIO(),lambda *a,**k:None,state,0,0,position_max_mm=395,position_fault_margin_mm=1)
    for pos in (-1.001,396.001,float('nan')):
        state=dict(statusword=0x1637,mode=3,position_mm=pos,velocity_counts_s=0)
        with pytest.raises(RuntimeError):
            record_and_validate(io.StringIO(),lambda *a,**k:None,state,0,0,position_max_mm=395,position_fault_margin_mm=1)
    assert LiftPolicy(10).target(-1,-1.)==0
    assert LiftPolicy(10).target(1,396.)==0
    assert LiftPolicy(10).target(1,-1.)>0
    assert LiftPolicy(10).target(-1,396.)<0
