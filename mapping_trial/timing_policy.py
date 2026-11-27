"""Strict isolated trial: no active PTP fault grace or automatic recovery."""
from pathlib import Path
IPS = ('192.168.1.132', '192.168.1.159')
POLICY = 'chassis_raw_strict_20261126_v1'
RAW_PATH = Path('/run/chassis-mapping-time/status.json')
PTP_TRANSIENT = frozenset()
def recoverable_ptp_fault(reason):
    return False
