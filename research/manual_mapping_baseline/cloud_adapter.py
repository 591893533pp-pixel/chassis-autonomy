#!/usr/bin/env python3
"""Adapt existing gated, un-deskewed dual clouds to the primary LiDAR frame."""
import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def adapt_points(points, extrinsic):
    required = {'x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp', 'sensor_id'}
    if not required.issubset(points.dtype.names or ()):
        raise ValueError('Missing Livox point fields')
    xyz = np.column_stack([points[n] for n in ('x', 'y', 'z')])
    valid = np.isfinite(xyz).all(axis=1) & np.isfinite(points['timestamp'])
    out = points[valid].copy()
    if not len(out) or set(np.unique(out['sensor_id'])) != {0, 1}:
        raise ValueError('Both sensors must contribute valid points')
    out = out[np.argsort(out['timestamp'], kind='stable')]
    # Driver timestamps are absolute nanoseconds stored as float64.
    start_ns = int(out['timestamp'][0])
    if start_ns <= 0 or out['timestamp'][-1] - start_ns > 150_000_000:
        raise ValueError('Invalid timestamp or scan window exceeds 150 ms')
    rot = Rotation.from_euler('xyz', extrinsic['rpy_deg'], degrees=True).as_matrix()
    xyz = np.column_stack([out[n] for n in ('x', 'y', 'z')])
    xyz = (xyz - np.asarray(extrinsic['xyz_m'])) @ rot
    for i, name in enumerate(('x', 'y', 'z')):
        out[name] = xyz[:, i]
    return out, start_ns


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    class Adapter(Node):
        def __init__(self):
            super().__init__('dual_lidar_slam_adapter')
            self.declare_parameter('extrinsics', str(Path(__file__).resolve().parent/'config/extrinsics_active.json'))
            cfg = json.loads(Path(self.get_parameter('extrinsics').value).read_text())
            if cfg.get('coordinate_revision') != 'actual_body_20260917' or cfg['frame'] != 'base_link_trial':
                raise ValueError('Unexpected source coordinate revision')
            self.extrinsic = cfg['sensors']['rear_right']
            self.last_ns = 0
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.pub = self.create_publisher(PointCloud2, '/slam/points', qos)
            self.sub = self.create_subscription(PointCloud2, '/livox/fused_preview', self.receive, qos)

        def receive(self, msg):
            try:
                if msg.header.frame_id != 'base_link_trial':
                    raise ValueError('Expected base_link cloud')
                arr, ns = adapt_points(point_cloud2.read_points(msg).reshape(-1), self.extrinsic)
                if abs(ns - (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)) > 150_000_000:
                    raise ValueError('Point/header timestamps disagree')
                if ns <= self.last_ns:
                    raise ValueError('Nonmonotonic scan; restart SLAM after clock reset')
                header = copy.deepcopy(msg.header)
                header.frame_id = 'livox_192_168_1_159'
                header.stamp.sec, header.stamp.nanosec = divmod(ns, 1_000_000_000)
                self.pub.publish(point_cloud2.create_cloud(header, msg.fields, arr))
                self.last_ns = ns
            except ValueError as exc:
                self.get_logger().warning(str(exc), throttle_duration_sec=5)

    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
