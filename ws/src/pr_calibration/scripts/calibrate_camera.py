#!/usr/bin/env python3
"""Camera->robot extrinsic calibration.

Looks up the transform from the camera optical frame to the AprilTag (published by
apriltag_ros), composes it with the KNOWN tag->base_link transform from the printed
sheet (config/layout.yaml), averages over several samples, and writes the resulting
camera pose to a YAML file as a static transform ``base_link -> camera``.

Run via the launch file, or standalone once apriltag_ros is publishing TF:

    ros2 run pr_calibration calibrate_camera.py
"""
import os

import numpy as np
import yaml
import tf_transformations as tft

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException
from ament_index_python.packages import get_package_share_directory


def _mat(translation, quat_xyzw):
    m = tft.quaternion_matrix(quat_xyzw)
    m[:3, 3] = translation
    return m


class CalibrateCamera(Node):
    def __init__(self):
        super().__init__('calibrate_camera')

        default_layout = os.path.join(
            get_package_share_directory('pr_calibration'), 'config', 'layout.yaml')
        self.declare_parameter('layout_file', default_layout)
        layout_file = self.get_parameter('layout_file').get_parameter_value().string_value
        with open(layout_file) as f:
            cfg = yaml.safe_load(f)

        self.declare_parameter('samples', int(cfg.get('samples', 30)))
        self.declare_parameter('output', os.path.join(
            os.path.dirname(layout_file), cfg.get('output', 'camera_extrinsics.yaml')))
        # Set true if the detected tag z-axis points down (into the table) rather
        # than up toward the camera (flip discovered from the logged camera height).
        self.declare_parameter('invert_tag_z', False)

        self._samples = self.get_parameter('samples').get_parameter_value().integer_value
        self._output = self.get_parameter('output').get_parameter_value().string_value
        invert = self.get_parameter('invert_tag_z').get_parameter_value().bool_value

        self._camera_frame = cfg['frames']['camera']
        self._tag_frame = cfg['frames']['tag']
        self._base_frame = cfg['frames']['base']

        # Known tag pose in base_link: flat on the table (z = 0), rotated tag_yaw
        # about +z; tag z-axis points up (toward the overhead camera).
        tx = float(cfg['tag_center_in_base_mm']['x']) / 1000.0
        ty = float(cfg['tag_center_in_base_mm']['y']) / 1000.0
        yaw = np.radians(float(cfg.get('tag_yaw_deg', 0.0)))
        self._T_base_tag = tft.euler_matrix(0.0, 0.0, yaw)
        if invert:
            self._T_base_tag = self._T_base_tag @ tft.euler_matrix(np.pi, 0.0, 0.0)
        self._T_base_tag[:3, 3] = (tx, ty, 0.0)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._trans = []
        self._quats = []
        self._warned = False
        self._timer = self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f'calibrating {self._base_frame} <- {self._camera_frame} via tag '
            f"'{self._tag_frame}'; collecting {self._samples} samples"
        )

    def _tick(self):
        try:
            tf = self._buffer.lookup_transform(
                self._camera_frame, self._tag_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self._warned:
                self.get_logger().warn(
                    f"waiting for TF {self._camera_frame} -> {self._tag_frame} "
                    '(is apriltag_ros seeing the tag?)')
                self._warned = True
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        self._trans.append([t.x, t.y, t.z])
        self._quats.append([q.x, q.y, q.z, q.w])
        if len(self._trans) % 10 == 0:
            self.get_logger().info(f'  {len(self._trans)}/{self._samples} samples')
        if len(self._trans) >= self._samples:
            self._timer.cancel()
            self._finish()

    def _finish(self):
        trans = np.mean(np.array(self._trans), axis=0)
        quats = np.array(self._quats)
        ref = quats[0]
        quats[np.dot(quats, ref) < 0] *= -1.0          # hemisphere-align
        q_mean = quats.mean(axis=0)
        q_mean /= np.linalg.norm(q_mean)

        t_cam_tag = _mat(trans, q_mean)                # tag pose in camera frame
        t_base_cam = self._T_base_tag @ np.linalg.inv(t_cam_tag)
        p = t_base_cam[:3, 3]
        q = tft.quaternion_from_matrix(t_base_cam)     # xyzw

        data = {
            'parent_frame': self._base_frame,
            'child_frame': self._camera_frame,
            'translation': {'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2])},
            'rotation': {'x': float(q[0]), 'y': float(q[1]),
                         'z': float(q[2]), 'w': float(q[3])},
            'static_transform_publisher_args': (
                f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} '
                f'{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} '
                f'{self._base_frame} {self._camera_frame}'),
        }
        with open(self._output, 'w') as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(
            f'camera in {self._base_frame}: pos=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m')
        if p[2] <= 0.0:
            self.get_logger().warn(
                'camera height (z) is <= 0 — the tag z-axis convention is probably '
                'flipped; re-run with -p invert_tag_z:=true')
        self.get_logger().info(f'wrote {self._output}')
        self.get_logger().info(
            'use it with:  ros2 run tf2_ros static_transform_publisher '
            + data['static_transform_publisher_args'])
        self._timer = None
        raise SystemExit  # done — leave the spin loop cleanly


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
