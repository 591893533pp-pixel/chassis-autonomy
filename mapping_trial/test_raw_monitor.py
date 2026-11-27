"""Packet identity and fault injection; no sockets or motor access."""
import socket,struct,unittest
from raw_time_monitor import MACS,RawTracker,decode
from timing_policy import recoverable_ptp_fault

def packet(ip='192.168.1.132',port=56300,seq=1,stamp=1_000_000_000):
    b=bytearray(36);struct.pack_into('<H',b,1,len(b));struct.pack_into('<H',b,7,seq)
    b[10]=0 if port==56400 else 1;b[11]=1;struct.pack_into('<Q',b,28,stamp)
    eth=bytes(6)+bytes.fromhex(MACS[ip].replace(':',''))+bytes.fromhex('0800')
    hdr=bytearray(20);hdr[0]=0x45;hdr[9]=17;hdr[12:16]=socket.inet_aton(ip);hdr[16:20]=socket.inet_aton('192.168.1.4')
    struct.pack_into('!H',hdr,2,20+8+len(b))
    return eth+hdr+struct.pack('!HHHH',port,port+1,8+len(b),0)+b

class Tests(unittest.TestCase):
    def test_all_four_streams_and_vlan(self):
        for ip in MACS:
            for port in (56300,56400):
                frame=packet(ip,port)
                self.assertEqual(decode(frame)[:2],(ip,port))
                vlan=frame[:12]+bytes.fromhex('810000010800')+frame[14:]
                self.assertEqual(decode(vlan),decode(frame))
    def test_wrong_mac_fragment_and_truncation(self):
        for offset, value in [(6,255),(20,32)]:
            frame=bytearray(packet());frame[offset]=value
            with self.assertRaises(ValueError):decode(frame)
        with self.assertRaises(ValueError):decode(packet()[:-1])
    def test_old_address_is_not_accepted(self):
        frame=bytearray(packet());frame[26:30]=socket.inet_aton('192.168.16.84')
        self.assertIsNone(decode(frame))
    def test_packet_loss_latches_even_single_point_loss(self):
        tracker=RawTracker();tracker.armed=True
        ip,port,b=decode(packet());tracker.observe(ip,port,b,1_000_000_000,1.,1_000_000_000)
        _,_,b=decode(packet(seq=3,stamp=1_001_000_000));tracker.observe(ip,port,b,1_001_000_000,1.001,1_001_000_000)
        self.assertIn('sequence_gap',tracker.fault)
        self.assertFalse(tracker.ready(1.001,1.001))
        self.assertEqual(tracker.tolerated_point_gaps,0)
    def test_time_jump_and_host_step(self):
        tracker=RawTracker();tracker.armed=True
        ip,port,b=decode(packet());tracker.observe(ip,port,b,1_500_000_000,1.5,1_500_000_000)
        self.assertIn('clock_age_ns',tracker.fault)
        tracker=RawTracker();tracker.armed=True;tracker.clock_base=0
        self.assertFalse(tracker.ready(1,1.01));self.assertEqual(tracker.fault,'host_clock_step')
    def test_no_automatic_ptp_recovery(self):
        self.assertFalse(recoverable_ptp_fault('ptp_clock:software_ptp_not_master'))
        tracker=RawTracker();tracker.armed=True;tracker.fail('ptp_clock:software_ptp_not_master')
        self.assertFalse(tracker.recover_ptp(100,True,True));self.assertIsNotNone(tracker.fault)
    def test_ready_requires_all_four_streams_and_warmup(self):
        tracker=RawTracker(warmup_seconds=.01)
        for i in range(20):
            now=1.+i*.001;stamp=1_000_000_000+i*1_000_000
            for ip in MACS:
                for port in (56300,56400):
                    _,_,b=decode(packet(ip,port,i,stamp));tracker.observe(ip,port,b,stamp,now,stamp)
            ready=tracker.ready(now,now)
            if i<10:self.assertFalse(ready)
        self.assertTrue(ready)
        self.assertFalse(tracker.ready(1.1,1.1));self.assertEqual(tracker.fault,'raw_stream_stale')

if __name__=='__main__':unittest.main()
