"""PV command shaping; release/fault/endpoint stops override the comfort ramp."""
import ctypes
import math
from pathlib import Path
from lift_limits import LIMITS

class VelocityProfile:
    def __init__(self):
        path=Path(__file__).resolve().parents[1]/'program/build/liblift_profile.so'
        self.lib=ctypes.CDLL(str(path))
        self.fn=self.lib.lift_velocity_step
        self.fn.argtypes=[ctypes.c_double]*6+[ctypes.POINTER(ctypes.c_double)]
        self.fn.restype=ctypes.c_int
        self.reset()
    def reset(self):self.velocity=0.;self.acceleration=0.
    def step(self,target,dt):
        if not math.isfinite(target) or not math.isfinite(dt) or not 0<dt<=.3:
            raise RuntimeError('Invalid PV profile input/time interval')
        if target==0:
            self.reset();return 0.
        out=(ctypes.c_double*2)()
        status=self.fn(self.velocity,self.acceleration,target,dt,
            LIMITS['lift_max_acceleration_m_s2']*1000,
            LIMITS['lift_max_jerk_m_s3']*1000,out)
        if status!=0:raise RuntimeError('PV trajectory calculation failed: '+str(status))
        self.velocity,self.acceleration=out
        # A shrinking endpoint speed cap takes precedence over smoothness.
        if abs(self.velocity)>abs(target) or self.velocity*target<0:
            self.velocity=target;self.acceleration=0.
        return self.velocity
