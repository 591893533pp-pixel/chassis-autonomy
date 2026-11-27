"""Offline regression: quantized idle feedback, real motion and stale data."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stationary_gate import StationaryGate

def test_lift_quantization_does_not_chop_chassis():
    g=StationaryGate(10)
    for i in range(100):
        t=1+i*.025
        good=g.observe(t,[-16 if i%5==0 else 0],t)
        if i>=8: assert good

def test_wheel_noise_and_repolling_do_not_chop_lift():
    g=StationaryGate(.1,dimensions=4)
    for i in range(100):
        t=1+i*.06
        speeds=[.13 if (i+k)%5==0 else .04 for k in range(4)]
        good=g.observe(t,speeds,t)
        if i>=4:
            assert good
            assert g.observe(t,speeds,t+.033)

@pytest.mark.parametrize('speed',[.11,-.11])
def test_sustained_wheel_motion_rejected(speed):
    g=StationaryGate(.1)
    for i in range(15):g.observe(1+i*.02,[0],1+i*.02)
    for i in range(1,12): result=g.observe(1.28+i*.02,[speed],1.28+i*.02)
    assert not result and g.reason=='sustained_motion'

def test_alternating_real_motion_is_not_cancelled():
    g=StationaryGate(.1)
    for i in range(20):result=g.observe(1+i*.02,[.15*(-1)**i],1+i*.02)
    assert not result

@pytest.mark.parametrize('kwargs', [dict(valid=False),dict(zero_command=False)])
def test_invalid_or_commanded_motion_clears_evidence(kwargs):
    g=StationaryGate(.1)
    for i in range(20):g.observe(1+i*.02,[0],1+i*.02)
    assert not g.observe(1.4,[0],1.4,**kwargs)
    assert not g.observe(1.42,[0],1.42)

def test_large_motion_stale_nan_regression_fail_closed():
    for stamp,speed,now in [(1.4,.31,1.4),(1.38,0,1.6),(1.4,float('nan'),1.4),(1.37,0,1.4)]:
        g=StationaryGate(.1)
        for i in range(20):g.observe(1+i*.02,[0],1+i*.02)
        assert not g.observe(stamp,[speed],now)

def test_duplicate_samples_cannot_prove_stationary():
    g=StationaryGate(.1)
    for i in range(20):assert not g.observe(1,[0],1+i*.005)

def test_wheel_hysteresis_needs_prior_stop_and_rejects_motion():
    g=StationaryGate(.1,release_limit=.15)
    for i in range(20):
        assert not g.observe(1+i*.02,[.12],1+i*.02)
    for i in range(20):result=g.observe(2+i*.02,[0],2+i*.02)
    assert result
    for i in range(20):assert g.observe(2.4+i*.02,[.12],2.4+i*.02)
    for i in range(20):result=g.observe(2.8+i*.02,[.16],2.8+i*.02)
    assert not result
