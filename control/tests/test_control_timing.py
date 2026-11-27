import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from control_timing import Periodic

def test_work_time_does_not_accumulate_and_overrun_skips():
    now=[0.]
    def sleep(dt): now[0]+=dt
    p=Periodic(clock=lambda:now[0],sleep=sleep)
    for i in range(100):
        now[0]+=.003
        p.wait()
    assert abs(now[0]-1.)<1e-9
    assert p.missed==0
    now[0]+=.035
    p.wait()
    assert abs(now[0]-1.04)<1e-9
    assert p.missed==3
    p.wait()
    assert abs(now[0]-1.05)<1e-9

def test_small_lift_overrun_does_not_discard_a_whole_period():
    now=[0.]
    def sleep(dt): now[0]+=dt
    p=Periodic(clock=lambda:now[0],sleep=sleep,late_tolerance=.002)
    now[0]=.0116
    p.wait()
    assert abs(now[0]-.0116)<1e-9
    assert p.missed==0 and p.late_cycles==1
    now[0]+=.0054
    p.wait()
    assert abs(now[0]-.02)<1e-9
    # A larger overrun still skips; it never drains missed commands in a burst.
    now[0]=.065
    p.wait()
    assert abs(now[0]-.07)<1e-9
    assert p.missed==4

def test_late_tolerance_cannot_allow_a_full_cycle_backlog():
    import pytest
    for invalid in [-.001,.01,.02]:
        with pytest.raises(ValueError): Periodic(late_tolerance=invalid)
