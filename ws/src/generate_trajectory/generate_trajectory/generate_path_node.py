#!/usr/bin/env python3
"""Connect detected objects with a collision-free path the arm draws -- on demand.

Brought up once and left running: it subscribes to YOLO detections + camera_info
and uses the FIXED camera<->base_link transform from TF (the static calibration in
pr_calibration -- no runtime marker). Each call to the ConnectObjects service plans
an A* path from a 'start' object to a 'goal' object (avoiding an 'obstacle' object)
using the LATEST detections, converts it to base_link via ray/plane intersection
with the table (z = table_z), and publishes it on /trajectory_path and the arm's
draw topic (default /inverse_kinematics_control/draw_path).

    ros2 service call /connect_objects pr_interfaces/srv/ConnectObjects \
        "{start: apple, goal: cup, obstacle: bottle}"
"""
import numpy as np
import tf_transformations as tft

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray
from tf2_ros import Buffer, TransformListener

from generate_trajectory.path_planning import PathPlanner
from pr_interfaces.srv import ConnectObjects
from pr_interfaces.msg import DetectedClasses


class ConnectServer(Node):

    def __init__(self):
        super().__init__('connect_objects_server')

        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('table_z', 0.8)          # object/draw plane in base_link (m)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('grid_resolution', 5)
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('draw_topic', '/inverse_kinematics_control/draw_path')
        self.declare_parameter('service_name', '/connect_objects')
        self.declare_parameter('detected_objects_topic', '/detected_objects')
        self.declare_parameter('detected_classes_topic', '/detected_classes')
        # Pen-up clearance above the draw plane for travel moves, and the safe
        # "crane" park pose (x, y, z in base_link) to return to after each draw.
        self.declare_parameter('lift_clearance', 0.02)
        self.declare_parameter('crane_position', [0.19, -0.05, 0.19])
        # Press the pen this far BELOW the paper plane while drawing, so it reliably
        # contacts despite small calibration/FK/tracking error. Draw height is thus
        # table_z - contact_depth: a fixed base_link value that does NOT depend on the
        # camera calibration (calibration only affects the object x/y, never z).
        self.declare_parameter('contact_depth', 0.0)
        # Keep recently-seen detections this long, so an object that flickers out of a
        # single (unstable) YOLO frame can still be connected.
        self.declare_parameter('detection_buffer_sec', 1.0)

        g = self.get_parameter
        self.camera_frame = g('camera_frame').value
        self.base_frame = g('base_frame').value
        self.table_z = g('table_z').value
        self.lift_clearance = g('lift_clearance').value
        self.crane = list(g('crane_position').value)
        self.contact_depth = g('contact_depth').value
        self.detection_buffer_sec = g('detection_buffer_sec').value
        # Allow the drawing/detection params to be changed live:
        #   ros2 param set /connect_objects_server table_z 0.03
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.planner = PathPlanner(
            workspace_size=(g('image_width').value, g('image_height').value),
            resolution=g('grid_resolution').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._K = None
        self._recent = {}   # class_name -> (u, v, size_x, stamp_sec); recent-detection buffer
        self.create_subscription(CameraInfo, g('camera_info_topic').value, self._caminfo_cb, 10)
        self.create_subscription(DetectionArray, g('detections_topic').value, self._dets_cb, 10)
        self.pub = self.create_publisher(Path, '/trajectory_path', 10)
        self.draw_pub = self.create_publisher(Path, g('draw_topic').value, 10)
        self.classes_pub = self.create_publisher(
            DetectedClasses, g('detected_classes_topic').value, 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, g('detected_objects_topic').value, 10)
        self.srv = self.create_service(ConnectObjects, g('service_name').value, self._on_connect)

        self.get_logger().info(
            f"connect server ready on '{g('service_name').value}'. Waiting for "
            f"{g('camera_info_topic').value} + {g('detections_topic').value} + "
            f'TF {self.base_frame}<-{self.camera_frame}.')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'table_z':
                self.table_z = p.value
            elif p.name == 'lift_clearance':
                self.lift_clearance = p.value
            elif p.name == 'crane_position':
                self.crane = list(p.value)
            elif p.name == 'contact_depth':
                self.contact_depth = p.value
            elif p.name == 'detection_buffer_sec':
                self.detection_buffer_sec = p.value
        return SetParametersResult(successful=True)

    def _caminfo_cb(self, msg: CameraInfo):
        self._K = np.array(msg.k, dtype=float).reshape(3, 3)

    def _dets_cb(self, msg: DetectionArray):
        # Update the recent-detection buffer (latest sighting per class).
        now = self.get_clock().now().nanoseconds * 1e-9
        for det in msg.detections:
            self._recent[det.class_name] = (
                det.bbox.center.position.x, det.bbox.center.position.y,
                det.bbox.size.x, now)
        fresh = self._fresh_detections(now)
        # Publish the currently-connectable class names (seen within the buffer window).
        self.classes_pub.publish(DetectedClasses(classes=sorted(fresh.keys())))
        # Publish RViz markers (label + table position) when calibration is available.
        self._publish_markers(msg)

    def _fresh_detections(self, now):
        """{class_name: (u, v, size_x)} for detections seen within
        detection_buffer_sec; also drops stale entries from the buffer."""
        fresh = {}
        stale = []
        for name, (u, v, sz, t) in self._recent.items():
            if now - t <= self.detection_buffer_sec:
                fresh[name] = (u, v, sz)
            else:
                stale.append(name)
        for name in stale:
            del self._recent[name]
        return fresh

    def _publish_markers(self, msg: DetectionArray):
        if self._K is None:
            return
        T = self._lookup_T_base_cam()
        if T is None:
            return
        K_inv = np.linalg.inv(self._K)
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, det in enumerate(msg.detections):
            xy = self._pixel_to_base(det.bbox.center.position.x,
                                     det.bbox.center.position.y, T, K_inv)
            if xy is None:
                continue
            dot = Marker()
            dot.header.frame_id = self.base_frame
            dot.header.stamp = now
            dot.ns = 'detected'
            dot.id = 2 * i
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x, dot.pose.position.y = xy
            dot.pose.position.z = float(self.table_z)
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.02
            dot.color.g = 1.0
            dot.color.a = 0.9
            text = Marker()
            text.header.frame_id = self.base_frame
            text.header.stamp = now
            text.ns = 'detected_labels'
            text.id = 2 * i + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = xy
            text.pose.position.z = float(self.table_z) + 0.03
            text.pose.orientation.w = 1.0
            text.scale.z = 0.02
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = det.class_name
            arr.markers.extend([dot, text])
        self.markers_pub.publish(arr)

    def _lookup_T_base_cam(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, Time())
        except Exception as exc:
            self.get_logger().warn(f'no TF {self.base_frame} <- {self.camera_frame}: {exc}')
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _pixel_to_base(self, u, v, T_base_cam, K_inv):
        ray_cam = K_inv @ np.array([u, v, 1.0])
        origin = T_base_cam[:3, 3]
        ray_base = T_base_cam[:3, :3] @ ray_cam
        if abs(ray_base[2]) < 1e-9:
            return None
        s = (self.table_z - origin[2]) / ray_base[2]
        p = origin + s * ray_base
        return float(p[0]), float(p[1])

    @staticmethod
    def _pose(header, x, y, z):
        p = PoseStamped()
        p.header = header
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation.w = 1.0
        return p

    def _on_connect(self, request, response):
        if self._K is None:
            return self._fail(response, 'no /camera_info received yet')
        T = self._lookup_T_base_cam()
        if T is None:
            return self._fail(response, f'TF {self.base_frame}<-{self.camera_frame} unavailable')

        # Use every object seen within the last detection_buffer_sec, not just the
        # single latest frame -- so a briefly-missed (flickery) detection still counts.
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = self._fresh_detections(now)
        if not fresh:
            return self._fail(response, f'no detections in the last {self.detection_buffer_sec:.1f}s')

        start_l = request.start.lower()
        goal_l = request.goal.lower()
        obstacle_l = request.obstacle.lower().strip()

        start = goal = None
        obstacles = []
        for name, (u, v, sz) in fresh.items():
            label = name.lower()
            if start_l and start_l in label and start is None:
                start = (u, v)
            elif goal_l and goal_l in label and goal is None:
                goal = (u, v)
            elif obstacle_l and obstacle_l in label:
                obstacles.append({'x': u, 'y': v, 'radius': sz * 0.5})

        if start is None or goal is None:
            return self._fail(
                response, f"'{request.start}' and/or '{request.goal}' not seen in the last "
                          f'{self.detection_buffer_sec:.1f}s (have: {sorted(fresh.keys())})')

        grid = self.planner.create_grid_map(obstacles)
        path = self.planner.a_star(start, goal, grid)
        if path is None:
            return self._fail(response, 'A* found no collision-free path')
        path = self.planner.smooth_path(path)

        K_inv = np.linalg.inv(self._K)
        out = Path()
        out.header.frame_id = self.base_frame
        out.header.stamp = self.get_clock().now().to_msg()

        # The drawn line itself, at a FIXED base_link draw height (independent of the
        # camera calibration): the paper plane table_z, pressed down by contact_depth.
        draw_z = self.table_z - self.contact_depth
        draw_poses = []
        for (px, py) in path:
            base_xy = self._pixel_to_base(px, py, T, K_inv)
            if base_xy is None:
                continue
            draw_poses.append(self._pose(out.header, base_xy[0], base_xy[1], draw_z))

        if len(draw_poses) < 2:
            return self._fail(response, 'path collapsed to < 2 points after projection')

        # Wrap the drawn line with pen-up travel: approach above the start (pen up),
        # lower to draw, lift above the goal, then return to the crane/park pose.
        travel_z = self.table_z + self.lift_clearance
        sx, sy = draw_poses[0].pose.position.x, draw_poses[0].pose.position.y
        gx, gy = draw_poses[-1].pose.position.x, draw_poses[-1].pose.position.y
        out.poses.append(self._pose(out.header, sx, sy, travel_z))   # pen up over start
        out.poses.extend(draw_poses)                                 # descend + draw
        out.poses.append(self._pose(out.header, gx, gy, travel_z))   # pen up over goal
        out.poses.append(self._pose(out.header, *self.crane))        # return to crane

        self.pub.publish(out)
        self.draw_pub.publish(out)
        response.success = True
        response.message = (
            f'drew {len(draw_poses)}-pt line {request.start}->{request.goal} '
            f'(avoiding {len(obstacles)} obstacle(s)); '
            f'start=({sx:.3f},{sy:.3f}) goal=({gx:.3f},{gy:.3f}); '
            f'pen-up z={travel_z:.3f}, returned to crane {tuple(round(c, 3) for c in self.crane)}')
        self.get_logger().info(response.message)
        return response

    def _fail(self, response, msg):
        response.success = False
        response.message = msg
        self.get_logger().warn(f'connect rejected: {msg}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ConnectServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
