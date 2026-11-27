"""Real worker lifecycle against a fake CAN bus: never opens CAN/joystick."""
import sys,time,io,threading
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import remote_session as rs

class FakeBus:
    enabled_at_start=False
    instances=[]
    def __init__(self,folder):
        self.enabled=self.enabled_at_start;self.target=0;self.writes=[]
        self.sock=io.StringIO();self.log=io.StringIO();self.instances.append(self)
    def snapshot(self):
        return dict(position_mm=100,velocity_counts_s=self.target,statusword=0x237 if self.enabled else 0x250,mode=3)
    def sdo(self,idx,value,size=4):
        self.writes.append((idx,value));
        if idx==0x60ff:self.target=value
        if idx==0x6040 and value==0:self.enabled=False
    def zero(self):self.sdo(0x60ff,0)
    def send(self,cid,data):self.writes.append((cid,bytes(data)))

def until(w,predicate,command=(True,0,True)):
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        w.update(*command)
        if predicate():return
        if w.error:raise AssertionError(w.error)
        time.sleep(.005)
    raise AssertionError('mock worker timed out')

@pytest.fixture
def fake(monkeypatch):
    FakeBus.instances=[];FakeBus.enabled_at_start=False
    monkeypatch.setattr(rs,'Bus',FakeBus)
    def enable(bus,check):check();bus.enabled=True;bus.writes.append(('enable',3))
    monkeypatch.setattr(rs,'enable_pv',enable)
    return FakeBus

def test_lift_motion_release_and_cleanup(tmp_path,fake):
    w=rs.LiftWorker(tmp_path,50)
    try:
        until(w,lambda:w.ready)
        b=fake.instances[0];until(w,lambda:b.enabled)
        until(w,lambda:b.target>0,(True,1,True))
        assert b.target/rs.COUNTS_MM<=50
        until(w,lambda:b.target==0 and w.stopped,(True,0,True))
        assert (0x60ff,0) in b.writes
    finally:w.stop()
    assert (0x6040,0) in b.writes and (0,bytes([128,1])) in b.writes
    assert b.sock.closed

def test_existing_enabled_lift_refuses_takeover(tmp_path,fake):
    fake.enabled_at_start=True;w=rs.LiftWorker(tmp_path,10);w.thread.join(2)
    assert '拒绝接管' in w.error
    assert not fake.instances[0].writes
    with pytest.raises(RuntimeError):w.stop()

def test_stale_heartbeat_zeroes_lift(tmp_path,fake):
    w=rs.LiftWorker(tmp_path,10)
    until(w,lambda:w.ready);b=fake.instances[0]
    until(w,lambda:b.enabled);until(w,lambda:b.target>0,(True,1,True))
    w.thread.join(2)
    assert not w.thread.is_alive() and '心跳' in w.error
    assert b.target==0 and not b.enabled
    with pytest.raises(RuntimeError):w.stop()

def test_moving_wheels_prevent_lift_enable(tmp_path,fake):
    w=rs.LiftWorker(tmp_path,10)
    try:
        until(w,lambda:w.ready,command=(True,0,False))
        for _ in range(10):w.update(True,1,False);time.sleep(.01)
        assert not fake.instances[0].enabled and not fake.instances[0].writes
    finally:w.stop()

def test_idle_lift_quantization_preserves_stop_confirmation(tmp_path,fake,monkeypatch):
    original=fake.snapshot
    calls=[0]
    def quantized(bus):
        state=original(bus);calls[0]+=1
        if bus.enabled and bus.target==0 and calls[0]%5==0:
            state['velocity_counts_s']=-16
        return state
    monkeypatch.setattr(fake,'snapshot',quantized)
    w=rs.LiftWorker(tmp_path,10)
    try:
        until(w,lambda:w.ready);bus=fake.instances[0]
        until(w,lambda:bus.enabled)
        until(w,lambda:w.stopped and w.stop_reason=='stationary')
        deadline=time.monotonic()+.35
        while time.monotonic()<deadline:
            w.update(True,0,True)
            assert w.stopped and not w.error
            time.sleep(.005)
        until(w,lambda:bus.target>0,(True,1,True))
        until(w,lambda:bus.target==0 and w.stopped,(True,0,True))
        assert not w.error
    finally:w.stop()

def test_simultaneous_lift_keeps_moving_and_releases_with_wheels_moving(tmp_path,fake):
    w=rs.LiftWorker(tmp_path,10,allow_simultaneous=True)
    try:
        until(w,lambda:w.ready,command=(False,0,False));bus=fake.instances[0]
        # Enabling still requires both axes neutral/stationary.
        for _ in range(10):w.update(True,1,False);time.sleep(.005)
        assert not w.armed and not bus.enabled
        until(w,lambda:w.armed)
        until(w,lambda:bus.target>0,(True,1,False))
        for _ in range(30):
            w.update(True,1,False);time.sleep(.005)
            assert bus.target>0 and not w.error
        until(w,lambda:bus.target==0 and w.stopped,(True,0,False))
        until(w,lambda:bus.target<0,(True,-1,False))
        until(w,lambda:bus.target==0 and w.stopped,(False,0,False))
    finally:w.stop()
    assert not bus.enabled and bus.target==0

def test_simultaneous_stale_heartbeat_still_disables_lift(tmp_path,fake):
    w=rs.LiftWorker(tmp_path,10,allow_simultaneous=True)
    until(w,lambda:w.ready);bus=fake.instances[0]
    until(w,lambda:w.armed)
    until(w,lambda:bus.target>0,(True,1,False))
    w.thread.join(2)
    assert not w.thread.is_alive() and '心跳' in w.error
    assert bus.target==0 and not bus.enabled
    with pytest.raises(RuntimeError):w.stop()
