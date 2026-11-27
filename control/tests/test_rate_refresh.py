"""Continuous target refresh under a fake CAN endpoint, including constant zero."""
from test_worker import fake,until
import remote_session as rs
import time

def test_enabled_idle_refreshes_target_without_rearming(tmp_path,fake):
    w=rs.LiftWorker(tmp_path,10,allow_simultaneous=True)
    try:
        until(w,lambda:w.ready);until(w,lambda:w.armed)
        bus=fake.instances[0];bus.writes.clear()
        end=time.monotonic()+.25
        while time.monotonic()<end:
            w.update(True,0,True);time.sleep(.002)
        writes=[v for idx,v in bus.writes if idx==0x60ff]
        assert len(writes)>=15
        assert all(v==0 for v in writes)
        assert not w.error
    finally:w.stop()
