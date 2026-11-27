"""Offline only: no CAN socket, no joystick device, no motor writes."""
import unittest
import io
import json
from lift_pv_remote import Gate, enable_pv, operation_enabled, neutral, record_and_validate

class FakeBus:
    def __init__(self, states=(0x250,0x231,0x233,0x237), mode=3):
        self.states=iter(states);self.mode=mode;self.writes=[]
    def sdo(self, idx, value=None, size=4, signed=False):
        if value is not None:self.writes.append((idx,value,size));return
        if idx==0x6061:return self.mode
        if idx==0x6041:return next(self.states)
        raise AssertionError(idx)
    def zero(self):self.sdo(0x60ff,0)
    def send(self,cid,data):self.writes.append((cid,data))

class Tests(unittest.TestCase):
    def keys(self,up=0,down=0):
        b=[0]*20;b[3]=up;b[4]=down;return b
    def test_release_and_second_press(self):
        g=Gate();b=self.keys(down=1)
        self.assertEqual(g.target(b,220,220,1),-1167)
        self.assertEqual(g.target(self.keys(),219,220,1.5),0)
        self.assertEqual(g.target(b,219,220,2),-1167)
    def test_continuous_hold(self):
        g=Gate();b=self.keys(up=1)
        self.assertEqual(g.target(b,100,100,1),1167)
        self.assertEqual(g.target(b,150,100,30),1167)
        self.assertEqual(g.target(b,220,100,400),1167)
        self.assertEqual(g.target(self.keys(),220,100,401),0)
    def test_conflicting_keys_require_release(self):
        g=Gate();self.assertEqual(g.target(self.keys(1,1),220,220,1),0)
        self.assertEqual(g.target(self.keys(1,0),220,220,2),0)
    def test_reverse_requires_release(self):
        g=Gate();g.target(self.keys(1,0),220,220,1)
        self.assertEqual(g.target(self.keys(0,1),220,220,1.1),0)
    def test_limits(self):
        self.assertEqual(Gate().target(self.keys(1,0),249.5,100,0),0)
        self.assertEqual(Gate().target(self.keys(0,1),.5,100,0),0)
        self.assertEqual(Gate().target(self.keys(1,0),227,100,0),1167)
        self.assertGreater(Gate().target(self.keys(1,0),0,0,0),0)
        self.assertLess(Gate().target(self.keys(0,1),250,250,0),0)
        self.assertTrue(0<Gate().target(self.keys(1,0),249,100,0)<1167)
        with self.assertRaises(RuntimeError):Gate().target(self.keys(0,1),-1,2,0)
    def test_full_speed_and_deceleration_feedback(self):
        state=dict(position_mm=120,velocity_counts_s=-1167,statusword=0x237,mode=3)
        record_and_validate(io.StringIO(),lambda *a,**k:None,state,-1167,200)
        log=io.StringIO()
        record_and_validate(log,lambda *a,**k:None,state,0,200,speed_reference=-1167)
        self.assertEqual(json.loads(log.getvalue())['target_counts_s'],0)
        state['velocity_counts_s']=-2400
        with self.assertRaisesRegex(RuntimeError,'overspeed'):
            record_and_validate(io.StringIO(),lambda *a,**k:None,state,-1167,200)
    def test_failing_sample_preserved(self):
        log=io.StringIO();events=[]
        state=dict(position_mm=222.0942857,velocity_counts_s=-633,statusword=0x237,mode=3)
        with self.assertRaisesRegex(RuntimeError,'overspeed'):
            record_and_validate(log,lambda name,**kw:events.append((name,kw)),state,-250,223.02)
        row=json.loads(log.getvalue())
        self.assertEqual(row['velocity_counts_s'],-633)
        self.assertEqual(row['target_counts_s'],-250)
        self.assertEqual(events[0][0],'fault_snapshot')
        self.assertEqual(events[0][1]['statusword'],0x237)
    def test_normal_sample(self):
        log=io.StringIO()
        record_and_validate(log,lambda *a,**kw:self.fail('unexpected fault'),
            dict(position_mm=220,velocity_counts_s=-125,statusword=0x237,mode=3),-125,220)
        self.assertEqual(json.loads(log.getvalue())['target_counts_s'],-125)
    def test_other_faults_preserved(self):
        for change in [dict(statusword=0x250),dict(mode=1),dict(position_mm=251)]:
            state=dict(position_mm=220,velocity_counts_s=0,statusword=0x237,mode=3)
            state.update(change);log=io.StringIO()
            with self.assertRaises(RuntimeError):
                record_and_validate(log,lambda *a,**kw:None,state,0,220)
            self.assertTrue(log.getvalue())
    def test_state_decode(self):
        self.assertTrue(operation_enabled(0x1237));self.assertFalse(operation_enabled(0x250))
    def test_neutral_gate(self):
        a=[0,0,0,0,1,1,0,0];b=self.keys()
        self.assertTrue(neutral(a,b));b[11]=1
        self.assertFalse(neutral(a,b));self.assertTrue(neutral(a,b,True))
        b[4]=1;self.assertFalse(neutral(a,b,True))
    def test_enable_sequence(self):
        b=FakeBus();enable_pv(b,lambda:None)
        self.assertEqual(b.writes[:2],[(0x60ff,0,4),(0x6060,3,1)])
        self.assertEqual([v for i,v,*_ in b.writes if i==0x6040],[6,7,15])
    def test_no_fault_reset(self):
        b=FakeBus(states=(0x218,))
        with self.assertRaises(RuntimeError):enable_pv(b,lambda:None)
        self.assertFalse(any(w[0]==0x6040 for w in b.writes))
    def test_mode_mismatch_no_enable(self):
        b=FakeBus(mode=1)
        with self.assertRaises(RuntimeError):enable_pv(b,lambda:None)
        self.assertFalse(any(w[0]==0x6040 for w in b.writes))
    def test_enable_input_failure(self):
        b=FakeBus()
        def fail():raise RuntimeError('input stale')
        with self.assertRaises(RuntimeError):enable_pv(b,fail)
        self.assertFalse(any(w[0]==0x6040 for w in b.writes))

if __name__=='__main__':unittest.main(verbosity=2)
