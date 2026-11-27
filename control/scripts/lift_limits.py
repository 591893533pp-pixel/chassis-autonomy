"""One source for the integrated PV limits; CSP fields are inactive metadata."""
import json
import math
from pathlib import Path
LIMITS=json.loads((Path(__file__).resolve().parents[1]/'config/lift_limits.json').read_text())
for key,value in LIMITS.items():
    if key.startswith('lift_') and (not isinstance(value,(int,float)) or not math.isfinite(value) or value<0):
        raise ValueError('Invalid lift limit: '+key)
MAX_SPEED_MM_S=LIMITS['lift_max_profile_velocity_mm_s']
MAX_POSITION_MM=min(LIMITS['lift_max_position_mm'],LIMITS['lift_teleop_max_position_mm'])
if not 0<MAX_SPEED_MM_S<=150 or not 0<MAX_POSITION_MM<=395:
    raise ValueError('Lift limits exceed the supplied 150mm/s, 395mm envelope')
if LIMITS['lift_min_position_mm']!=0 or LIMITS['control_mode']!='PV' or LIMITS['csp_parameters_active']:
    raise ValueError('This entry supports zero-origin PV only')
if not 0<LIMITS['lift_max_acceleration_m_s2']<=.30 or not 0<LIMITS['lift_max_jerk_m_s3']<=1.50:
    raise ValueError('Invalid acceleration/jerk limit')

# Feedback fault tolerance only; commanded travel and directional margins stay unchanged.
POSITION_FAULT_MARGIN_MM=1.0
def within_position_fault_range(position_mm, maximum_mm=395.):
    return (math.isfinite(position_mm) and
            -POSITION_FAULT_MARGIN_MM <= position_mm <= maximum_mm+POSITION_FAULT_MARGIN_MM)
