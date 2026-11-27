"""Requested velocity ceiling, metric travel and release-required PV gate."""
import math
COUNTS_MM=800*7/48
from lift_limits import MAX_SPEED_MM_S,MAX_POSITION_MM,within_position_fault_range
class LiftPolicy:
    def __init__(self,speed):
        if not math.isfinite(speed) or not 0<speed<=MAX_SPEED_MM_S:
            raise ValueError(f'升降请求速度必须在(0,{MAX_SPEED_MM_S:g}] mm/s')
        self.speed=math.floor(speed*COUNTS_MM)
        if self.speed<1:raise ValueError('速度低于一个编码器计数/秒')
        self.direction=0;self.blocked=False
    def target(self,direction,position):
        if not within_position_fault_range(position,MAX_POSITION_MM):raise RuntimeError(f'升降反馈超出-1–{MAX_POSITION_MM+1:g}mm')
        if direction==2:self.blocked=True;return 0
        if direction not in (-1,0,1):raise ValueError('invalid lift direction')
        if direction==0:self.direction=0;self.blocked=False;return 0
        if self.blocked:return 0
        if self.direction and direction!=self.direction:self.blocked=True;return 0
        self.direction=direction
        distance=MAX_POSITION_MM-position if direction>0 else position
        if distance<=.5:self.blocked=True;return 0
        return direction*min(self.speed,max(0,math.floor((distance-.5)/.5*COUNTS_MM)))
