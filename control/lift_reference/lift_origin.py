"""Explicit operator-confirmed lower-stop reference; never writes the drive.

Keep a 0.1 mm measurement reserve below the observed lower stop. Existing
0.5 mm downward margins remain effective. Negative-only origin correction
makes the original upper raw-count limit more conservative, never larger.
"""
import json
import math
from pathlib import Path

COUNTS_MM = 800 * 7 / 48
ORIGIN_PATH = Path(__file__).resolve().parents[1] / 'config/lift_origin.json'
RESERVE_COUNTS = math.ceil(.1 * COUNTS_MM)
MAX_CORRECTION_COUNTS = math.floor(.5 * COUNTS_MM)


def validate_origin(data):
    if (data.get('schema') != 1 or data.get('can_interface') != 'can0'
            or data.get('node_id') != 1 or data.get('counts_per_mm') != COUNTS_MM
            or data.get('operator_confirmed_lower_stop') is not True):
        raise ValueError('invalid_lift_origin_metadata')
    origin = data.get('origin_counts')
    observed = data.get('observed_min_counts')
    if (type(origin) is not int or type(observed) is not int
            or not -MAX_CORRECTION_COUNTS <= origin < 0
            or not -MAX_CORRECTION_COUNTS <= observed <= 0
            or origin != observed - RESERVE_COUNTS):
        raise ValueError('invalid_lift_origin_counts')
    return origin


def load_origin(path=ORIGIN_PATH):
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        return 0
    return validate_origin(data)


def calibrate_samples(samples):
    if len(samples) < 20 or samples[-1]['elapsed_s'] - samples[0]['elapsed_s'] < 1.:
        raise ValueError('lift_origin_insufficient_samples')
    previous = -1.
    for s in samples:
        if not all(math.isfinite(s[k]) for k in ('elapsed_s','raw_counts','velocity_counts_s')):
            raise ValueError('lift_origin_invalid_feedback')
        if s['elapsed_s'] <= previous:
            raise ValueError('lift_origin_timestamp_regression')
        previous = s['elapsed_s']
        if (s['statusword'] & 0x4f != 0x40 or s['mode'] != 3
                or s['error_code'] != 0 or s['target_velocity_counts_s'] != 0):
            raise ValueError('lift_origin_drive_not_disabled_idle_pv')
        if type(s['raw_counts']) is not int or not -MAX_CORRECTION_COUNTS <= s['raw_counts'] <= 0:
            raise ValueError('lift_origin_not_near_lower_stop')
    positions = [s['raw_counts'] for s in samples]
    speeds = [abs(s['velocity_counts_s']) for s in samples]
    if max(positions)-min(positions) > 3 or max(speeds) >= 30 or sum(speeds)/len(speeds) > 10:
        raise ValueError('lift_origin_not_stationary')
    data = dict(schema=1,can_interface='can0',node_id=1,counts_per_mm=COUNTS_MM,
                operator_confirmed_lower_stop=True,observed_min_counts=min(positions),
                observed_max_counts=max(positions),reserve_counts=RESERVE_COUNTS,
                origin_counts=min(positions)-RESERVE_COUNTS)
    validate_origin(data)
    return data
