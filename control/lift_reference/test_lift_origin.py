import copy
import json
import pytest
from lift_origin import COUNTS_MM, RESERVE_COUNTS, calibrate_samples, load_origin, validate_origin
from lift_pv_remote import Bus


def samples():
    return [dict(elapsed_s=i*.025,raw_counts=-17,velocity_counts_s=0,statusword=0x1250,
                 mode=3,error_code=0,target_velocity_counts_s=0) for i in range(50)]


def test_confirmed_minimum_preserves_downward_margin_and_upper_limit():
    data=calibrate_samples(samples());origin=validate_origin(data)
    assert origin==-29
    corrected=(-17-origin)/COUNTS_MM
    assert 0<corrected<.5  # downward commands remain prohibited by existing guard
    assert 395*COUNTS_MM+origin < 395*COUNTS_MM  # never expands upper travel


@pytest.mark.parametrize('change',[
    dict(raw_counts=-100),dict(raw_counts=1),dict(velocity_counts_s=30),
    dict(statusword=0x27),dict(statusword=0x8),dict(mode=1),dict(error_code=1),
    dict(target_velocity_counts_s=-1),dict(velocity_counts_s=float('nan')),
    dict(elapsed_s=float('inf')),dict(raw_counts=-17.0),
])
def test_invalid_or_unsafe_sample_refuses_calibration(change):
    rows=samples();rows[20].update(change)
    with pytest.raises(ValueError):calibrate_samples(rows)


def test_drift_and_missing_evidence_rejected():
    rows=samples();rows[20]['raw_counts']=-21
    with pytest.raises(ValueError):calibrate_samples(rows)
    with pytest.raises(ValueError):calibrate_samples(samples()[:19])
    rows=samples();rows[20]['elapsed_s']=rows[19]['elapsed_s']
    with pytest.raises(ValueError):calibrate_samples(rows)


@pytest.mark.parametrize('change',[
    dict(origin_counts=1000),dict(origin_counts=-1000),dict(origin_counts=-29.0),
    dict(observed_min_counts=-19),dict(operator_confirmed_lower_stop=False),
    dict(node_id=2),dict(counts_per_mm=1),dict(schema=2),dict(can_interface='can1'),
])
def test_invalid_persistent_origin_fails_closed(tmp_path,change):
    data=calibrate_samples(samples());data.update(change)
    path=tmp_path/'origin.json';path.write_text(json.dumps(data))
    with pytest.raises(ValueError):load_origin(path)


def test_absent_or_corrupt_file(tmp_path):
    path=tmp_path/'origin.json'
    assert load_origin(path)==0
    path.write_text('{broken')
    with pytest.raises(ValueError):load_origin(path)


def test_snapshot_reports_actual_raw_value_and_never_clamps():
    bus=Bus.__new__(Bus);bus.origin_counts=-29
    raw={0x6064:-17,0x606c:0,0x6041:0x1250,0x6061:3}
    bus.sdo=lambda idx,**kwargs:raw[idx]
    state=bus.snapshot()
    assert state['position_mm']==12/COUNTS_MM
    assert state['raw_position_mm']==pytest.approx(-17/COUNTS_MM)
    assert state['origin_counts']==-29
    raw[0x6064]=-100
    assert bus.snapshot()['position_mm']<0  # not made into a safe zero
    raw[0x6064]=int(396*COUNTS_MM)
    assert bus.snapshot()['position_mm']>395
