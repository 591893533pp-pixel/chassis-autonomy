import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import recover_lift_stop as recovery

class Bus:
    def __init__(self):self.enabled=True;self.target=100;self.mode=3;self.writes=[];self.bad_readback=False
    def snapshot(self):
        return dict(position_mm=213.45,velocity_counts_s=self.target,mode=self.mode,
                    statusword=0x1637 if self.enabled else 0x250)
    def zero(self):self.sdo(0x60ff,0)
    def sdo(self,index,value=None,size=4,signed=False):
        if value is None:return 1 if self.bad_readback else self.target
        self.writes.append((index,value))
        if index==0x60ff:self.target=value
        if index==0x6040:self.enabled=False
    def send(self,index,value):self.writes.append((index,value))

def test_recovery_only_zeroes_then_disables():
    bus=Bus();result=recovery.recover(bus,lambda *a,**k:None)
    assert result['disabled'] and not bus.enabled and bus.target==0
    assert bus.writes==[(0x60ff,0),(0x6040,0),(0,bytes([128,1]))]

def test_wrong_mode_refused_without_writes():
    bus=Bus();bus.mode=1
    with pytest.raises(RuntimeError):recovery.recover(bus,lambda *a,**k:None)
    assert not bus.writes

def test_wrong_zero_readback_does_not_disable():
    bus=Bus();bus.bad_readback=True
    with pytest.raises(RuntimeError):recovery.recover(bus,lambda *a,**k:None)
    assert bus.writes==[(0x60ff,0)] and bus.enabled
