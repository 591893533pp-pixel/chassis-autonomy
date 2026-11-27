"""Robot collision-box exclusion in the common base frame, preserving point fields."""
import numpy as np

class RobotSelfFilter:
    def __init__(self, config, frame):
        self.enabled = config.get('enabled', False)
        if config.get('frame') != frame:
            raise ValueError('Self-filter frame must match fusion frame')
        padding = float(config.get('padding_m', 0))
        if not np.isfinite(padding) or padding < 0:
            raise ValueError('Invalid self-filter padding')
        self.boxes = []
        for box in config.get('boxes', []):
            lo, hi = np.asarray(box['min_m'], dtype=float), np.asarray(box['max_m'], dtype=float)
            if lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all() or (lo > hi).any():
                raise ValueError('Invalid robot collision box')
            self.boxes.append((lo-padding, hi+padding))
        if self.enabled and not self.boxes:
            raise ValueError('Enabled self-filter requires boxes')

    def mask(self, xyz):
        remove = np.zeros(len(xyz), dtype=bool)
        if self.enabled:
            for lo, hi in self.boxes:
                remove |= ((xyz[:,0]>=lo[0]) & (xyz[:,0]<=hi[0]) &
                           (xyz[:,1]>=lo[1]) & (xyz[:,1]<=hi[1]) &
                           (xyz[:,2]>=lo[2]) & (xyz[:,2]<=hi[2]))
        return remove

    def filter(self, points):
        remove = self.mask(np.column_stack([points[n] for n in ('x', 'y', 'z')]))
        return points[~remove], int(remove.sum())
