#!/usr/bin/env python3
"""Evaluate a frozen relative transform on new static data; never refit or deploy."""
import argparse
import hashlib
import json
import pathlib
import sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def signature(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolkit', type=pathlib.Path, required=True)
    parser.add_argument('--candidate-dir', type=pathlib.Path, required=True)
    parser.add_argument('--session', type=pathlib.Path, required=True)
    parser.add_argument('--label', default='D')
    parser.add_argument('--output', type=pathlib.Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.toolkit.resolve()))
    import calibrate as cal
    import registration as reg
    if args.output.exists():
        raise ValueError('Refuse to overwrite an existing validation report')
    cal.label(args.label)
    frozen = cal.read(args.candidate_dir / 'freeze_before_D.json')
    for name, expected in frozen['sha256'].items():
        basename = pathlib.Path(name).name
        if basename in ('candidate.json', 'seed_stability.json'):
            path = args.candidate_dir / basename
        elif basename in ('settings.json', 'initial_body_estimate.json'):
            path = args.toolkit / 'config' / basename
        else:
            continue  # Solver sources are provenance; no solver runs here.
        if signature(path) != expected:
            raise ValueError('Frozen evidence/configuration hash mismatch: ' + basename)
    if frozen['fresh_holdout'] != args.session.name + '/' + args.label:
        raise ValueError('This is not the predeclared holdout')
    candidate = cal.read(args.candidate_dir / 'candidate.json')
    settings = cal.read(args.toolkit / 'config/settings.json')
    limits = settings['limits']
    if candidate['limits_unchanged'] != limits:
        raise ValueError('Acceptance limits changed')
    transform = np.array(candidate['T_rear_right_from_front_left'])
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('Invalid transform')
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(rotation), 1) or not np.allclose(transform[3], [0, 0, 0, 1]):
        raise ValueError('Transform is not a rigid SE(3) matrix')
    folder = args.session / args.label
    evidence = {f.name: signature(f) for f in folder.iterdir() if f.name in ('metadata.json', 'diagnostics.json', 'front_left.npz', 'rear_right.npz')}
    for name in candidate['training_scenes'] + ['C', 'B_repeat']:
        for role in settings['sensors']:
            prior = args.session / name / (role + '.npz')
            if prior.exists() and signature(prior) in evidence.values():
                raise ValueError('Holdout reuses a previous point cloud')
    meta, cloud = cal.capture_data(folder, settings)
    if not meta.get('pose_note', '').strip():
        raise ValueError('Missing holdout pose note')
    source, target = cloud[settings['moving']], cloud[settings['anchor']]
    initial = cal.initial_transform()
    baseline = cal.evaluate(source, target, initial)
    score = cal.evaluate(source, target, transform)
    correction = transform @ np.linalg.inv(initial)
    shift = float(np.linalg.norm(correction[:3, 3]))
    angle = float(np.degrees(Rotation.from_matrix(correction[:3, :3]).magnitude()))
    # Inspect local information at the fixed transform; do not optimize it on D.
    sn, sg = reg.normals(source)
    tn, tg = reg.normals(target)
    moved = reg.apply(source, transform)
    distance, idx = cKDTree(target).query(moved, workers=2)
    keep = (distance < .12) & sg & tg[idx] & (abs(np.sum((sn @ rotation.T) * tn[idx], axis=1)) > .85)
    x, n = moved[keep], tn[idx[keep]]
    residual = np.sum((x - target[idx[keep]]) * n, axis=1)
    matrix = np.column_stack((np.cross(x, n), n)) / np.sqrt(1 + (residual[:, None] / .035) ** 2)
    normalized = matrix / np.maximum(np.linalg.norm(matrix, axis=0), 1e-12)
    eigenvalues = np.linalg.eigvalsh(normalized.T @ normalized)
    reasons = []
    checks = [
        (score['matched_planar_points'] >= limits['min_matches'], '匹配点不足'),
        (score['mutual_overlap_within_10cm'] >= limits['min_mutual_overlap'], '共同视野不足'),
        (score['plane_rmse_m'] <= limits['max_plane_rmse_m'], 'RMSE超限'),
        (score['plane_abs_p95_m'] <= limits['max_plane_p95_m'], 'p95超限'),
        (score['plane_rmse_m'] <= baseline['plane_rmse_m'] + .003, '相对初值明显变差'),
        (shift <= limits['max_initial_correction_translation_m'] and angle <= limits['max_initial_correction_angle_deg'], '修正超出初值范围'),
        (len(x) >= limits['min_matches'] and min(eigenvalues) >= limits['min_normalized_information_eigenvalue'], '六自由度方向约束不足'),
    ]
    seeds = cal.read(args.candidate_dir / 'seed_stability.json')
    for name, result in [('frozen', {'scene_difference': candidate['training_scene_difference']})] + list(seeds.items()):
        spread = result['scene_difference']
        checks.append((spread['translation_m'] <= limits['max_trial_translation_spread_m'] and spread['rotation_deg'] <= limits['max_trial_rotation_spread_deg'], '训练场景不一致: ' + name))
    reasons = [reason for ok, reason in checks if not ok]
    report = dict(status='RELATIVE_CHECKS_PASS_REVIEW_REQUIRED' if not reasons else 'REJECTED',
                  checks_passed=not reasons, deployed=False, calibrated=False,
                  scope='Static relative LiDAR extrinsics only; no internal, body-anchor, IMU or dynamic timing calibration.',
                  candidate_sha256=signature(args.candidate_dir / 'candidate.json'),
                  validator_sha256=signature(pathlib.Path(__file__)),
                  freeze=frozen, holdout=args.session.name + '/' + args.label,
                  evidence_sha256=evidence, pose_note=meta['pose_note'],
                  static_review=meta.get('static_review'),
                  T_rear_right_from_front_left=transform.tolist(),
                  rpy_deg=Rotation.from_matrix(rotation).as_euler('xyz', degrees=True).tolist(),
                  baseline=baseline, candidate=score, correction_translation_m=shift,
                  correction_rotation_deg=angle, normalized_information_eigenvalues=eigenvalues.tolist(),
                  training_scene_difference=candidate['training_scene_difference'],
                  limits=limits, reasons=reasons)
    args.output.mkdir(parents=True)
    cal.write(args.output / 'report.json', report)
    cal.plot({args.label: cloud}, [args.label], initial, transform, args.output)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if not reasons else 2


if __name__ == '__main__':
    sys.exit(main())
