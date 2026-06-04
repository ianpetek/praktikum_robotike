#!/usr/bin/env python3
"""Inverse kinematics node for the PR RRR manipulator.

The arm has three actuated joints (yaw, bicep, forearm) plus a passive
pen_mount_joint that keeps the pen perpendicular to the ground at all times
(its angle is driven to bicep + forearm by pen_mount_broadcaster). Because the
pen assembly is held at a constant orientation in the world, the pen tip is a
fixed offset from the wrist *in the world frame* -- but that offset is sizable
and points forward, so the tip does not sit directly below the wrist.

Rather than hand-deriving a closed form (error-prone given the joint axis flips
and offsets in pr_robot.xacro), this node builds the exact forward kinematics
from the URDF transform chain and inverts it numerically (damped least squares
with joint-limit clamping and a few seeds). The arm is only 3-DOF, so this is
cheap and avoids sign/offset mistakes.

The reachable workspace is on the -y side of the base (the home reach direction,
because the yaw_joint carries an rpy="0 0 pi" flip).

Two interfaces are exposed, both in the base frame:
  * ``~/target_point`` (geometry_msgs/PointStamped) -- jog: solve IK for one
    pen-tip target and send a single-point trajectory.
  * ``~/draw_path`` (nav_msgs/Path) -- draw: resample the polyline at a fixed
    Cartesian spacing, solve IK per sample, time the samples by a draw speed,
    and run the whole stroke through the arm controller's FollowJointTrajectory
    action. Z is the pen height (full 3D), so this also handles pen up/down.
"""
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


def _trans(x, y, z):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def _rotx(t):
    c, s = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[1:3, 1:3] = ((c, -s), (s, c))
    return T


def _rotz(t):
    c, s = np.cos(t), np.sin(t)
    T = np.eye(4)
    T[:2, :2] = ((c, -s), (s, c))
    return T


# Constant transforms between joints, from pr_robot.xacro. The variable joint
# rotations (yaw about +z, bicep/forearm about +x, passive pen_mount about -x)
# are composed in forward_kinematics().
_T_YAW = _trans(0.0, 0.012, 0.036) @ _rotz(np.pi)          # base -> yaw_joint
_T_BICEP = _trans(-0.019, 0.0, 0.0405)                     # yaw -> bicep_joint
_T_FOREARM = _trans(0.0, 0.000172, 0.174)                  # bicep -> forearm_joint
_T_PENMOUNT = _trans(0.0143, 0.157, -3.3e-5)               # forearm -> pen_mount_joint
_T_PEN_TIP = _trans(0.0047, 0.045368, -0.018223) @ _trans(0.0, 0.000261, -0.042585)


class InverseKinematicsControl(Node):
    # Actuated joints, in the order the arm_controller expects them.
    JOINT_NAMES = ('yaw_joint', 'bicep_joint', 'forearm_joint')

    # Actuated joint limits (rad), from the URDF, aligned with JOINT_NAMES.
    # bicep and forearm are symmetric (+/-90 deg) so the arm can fully extend
    # forward past vertical.
    JOINT_LOWER = np.array([-1.570796, -1.570796, -1.570796])
    JOINT_UPPER = np.array([1.570796, 1.570796, 1.570796])

    _Z_EPS = 1.0e-5           # m, threshold for "this segment moves in z"
    _LIFT_MARGIN = 1.0e-3     # m above the lowest z that counts as "pen lifted"

    # Numerical IK settings.
    _IK_TOL = 1.0e-4          # acceptable position residual (m)
    _IK_MAX_ITERS = 100
    _IK_DAMPING = 1.0e-4      # Levenberg-Marquardt damping
    # Seeds (yaw, bicep, forearm) tried in turn to escape limit traps / branches.
    _IK_SEEDS = (
        (0.0, 0.5, -0.5),
        (0.0, 0.2, -0.2),
        (0.0, 1.0, -1.0),
        (0.0, 0.8, -0.4),
    )

    def __init__(self):
        super().__init__('inverse_kinematics_control')
        self.declare_parameter('use_sim', False)
        self._use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        # Seconds to reach a jog target (single-point trajectory duration).
        self.declare_parameter('jog_duration', 2.0)
        self._jog_duration = self.get_parameter('jog_duration').get_parameter_value().double_value
        # Command topic of the arm's JointTrajectoryController (jog).
        self.declare_parameter('arm_trajectory_topic', '/arm_controller/joint_trajectory')
        traj_topic = self.get_parameter('arm_trajectory_topic').get_parameter_value().string_value
        # FollowJointTrajectory action of the arm controller (draw).
        self.declare_parameter('arm_action', '/arm_controller/follow_joint_trajectory')
        action_name = self.get_parameter('arm_action').get_parameter_value().string_value
        # Drawing motion settings.
        self.declare_parameter('draw_speed', 0.05)        # pen-tip speed (m/s)
        self._draw_speed = self.get_parameter('draw_speed').get_parameter_value().double_value
        self.declare_parameter('sample_spacing', 0.005)   # Cartesian resample step (m)
        self._sample_spacing = self.get_parameter('sample_spacing').get_parameter_value().double_value
        # Minimum time to reach the first point. The actual approach time is
        # distance(current pose -> first point) / travel_speed, floored by this.
        self.declare_parameter('draw_lead_time', 0.5)
        self._draw_lead_time = self.get_parameter('draw_lead_time').get_parameter_value().double_value
        # Pen up/down moves are timed separately (and slowly) so raising
        # draw_speed never makes the pen slam onto the paper.
        self.declare_parameter('lift_speed', 0.03)        # vertical pen move speed (m/s)
        self._lift_speed = self.get_parameter('lift_speed').get_parameter_value().double_value
        self.declare_parameter('contact_dwell', 0.15)     # settle time at pen down/up (s)
        self._contact_dwell = self.get_parameter('contact_dwell').get_parameter_value().double_value
        # Pen-up repositioning (between strokes/letters) -- faster than drawing
        # since the pen is off the paper.
        self.declare_parameter('travel_speed', 0.12)      # pen-up move speed (m/s)
        self._travel_speed = self.get_parameter('travel_speed').get_parameter_value().double_value
        # Floor on per-segment time: never hand the controller setpoints closer
        # than this (its update period is 0.02 s). This sets the top speed:
        # max speed ~= sample_spacing / min_segment_time. Lower it for faster
        # moves, at the risk of the controller dropping points on tight curves.
        self.declare_parameter('min_segment_time', 0.02)
        self._min_segment_time = self.get_parameter('min_segment_time').get_parameter_value().double_value
        # Sharp turns within a stroke get a zero-velocity stop (and optional
        # hold) so corners stay crisp instead of being rounded off.
        self.declare_parameter('corner_angle', 35.0)     # deg; turns sharper than this stop
        self._corner_angle = self.get_parameter('corner_angle').get_parameter_value().double_value
        self.declare_parameter('corner_dwell', 0.05)     # s, hold at a sharp corner
        self._corner_dwell = self.get_parameter('corner_dwell').get_parameter_value().double_value

        # Allow the drawing parameters to be changed live, e.g.
        # `ros2 param set /inverse_kinematics_control draw_speed 0.03`.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Warm-start seed, updated with the last solution for path continuity.
        self._ik_seed = np.array(self._IK_SEEDS[0])
        self._active_goal = None  # in-flight FollowJointTrajectory goal handle
        self._current_q = None    # latest (yaw, bicep, forearm) from joint_states

        self._traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self._traj_action = ActionClient(self, FollowJointTrajectory, action_name)
        self.create_subscription(PointStamped, '~/target_point', self._jog_cb, 10)
        self.create_subscription(Path, '~/draw_path', self._path_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.get_logger().info(
            f'inverse_kinematics_control started (use_sim={self._use_sim}); '
            f'jog -> {self.resolve_topic_name("~/target_point")}, '
            f'draw -> {self.resolve_topic_name("~/draw_path")}, '
            f'arm action -> {action_name}'
        )

    def _on_set_parameters(self, params):
        """Apply live updates to the drawing parameters with validation."""
        for p in params:
            if (p.name in ('draw_speed', 'sample_spacing', 'lift_speed', 'travel_speed',
                           'min_segment_time') and p.value <= 0.0):
                return SetParametersResult(
                    successful=False, reason=f'{p.name} must be > 0'
                )
            if p.name in ('contact_dwell', 'corner_dwell') and p.value < 0.0:
                return SetParametersResult(
                    successful=False, reason=f'{p.name} must be >= 0'
                )
            if p.name == 'corner_angle' and not (0.0 < p.value < 180.0):
                return SetParametersResult(
                    successful=False, reason='corner_angle must be in (0, 180) deg'
                )
            if p.name == 'draw_speed':
                self._draw_speed = p.value
            elif p.name == 'sample_spacing':
                self._sample_spacing = p.value
            elif p.name == 'draw_lead_time':
                self._draw_lead_time = p.value
            elif p.name == 'lift_speed':
                self._lift_speed = p.value
            elif p.name == 'contact_dwell':
                self._contact_dwell = p.value
            elif p.name == 'travel_speed':
                self._travel_speed = p.value
            elif p.name == 'min_segment_time':
                self._min_segment_time = p.value
            elif p.name == 'corner_angle':
                self._corner_angle = p.value
            elif p.name == 'corner_dwell':
                self._corner_dwell = p.value
        return SetParametersResult(successful=True)

    def forward_kinematics(self, q):
        """Pen-tip position (x, y, z) in the base frame for joint vector
        q = (yaw, bicep, forearm). The passive pen_mount tracks bicep + forearm
        about its -x axis, keeping the pen vertical."""
        qy, qb, qf = q
        T = (
            _T_YAW @ _rotz(qy)
            @ _T_BICEP @ _rotx(qb)
            @ _T_FOREARM @ _rotx(qf)
            @ _T_PENMOUNT @ _rotx(-(qb + qf))
            @ _T_PEN_TIP
        )
        return T[:3, 3]

    def inverse_kinematics(self, x, y, z):
        """Compute actuated joint angles for a pen-tip target (x, y, z) in the
        base frame.

        Returns a dict {'yaw_joint', 'bicep_joint', 'forearm_joint'} of angles
        in radians. Raises ValueError if the target cannot be reached within the
        joint limits.
        """
        target = np.array([x, y, z], dtype=float)
        # Try the warm-start seed first, then the fixed seeds.
        best_q, best_res = None, float('inf')
        for seed in (self._ik_seed, *self._IK_SEEDS):
            q, res = self._solve(target, np.array(seed, dtype=float))
            if res < best_res:
                best_q, best_res = q, res
            if best_res < self._IK_TOL:
                break

        if best_res > self._IK_TOL:
            raise ValueError(
                f'target ({x:.3f}, {y:.3f}, {z:.3f}) unreachable within limits '
                f'(residual {best_res * 1000:.1f} mm)'
            )

        self._ik_seed = best_q  # warm-start the next call
        angles = {name: float(v) for name, v in zip(self.JOINT_NAMES, best_q)}
        self._check_limits(angles)
        return angles

    def _solve(self, target, q):
        """Damped least-squares iteration from a single seed. Returns the
        clamped joint vector and the final position residual (m)."""
        eps = 1.0e-6
        for _ in range(self._IK_MAX_ITERS):
            p = self.forward_kinematics(q)
            err = target - p
            if np.linalg.norm(err) < eps:
                break
            # Finite-difference Jacobian of tip position w.r.t. joints.
            jac = np.column_stack([
                (self.forward_kinematics(q + np.eye(3)[i] * eps) - p) / eps
                for i in range(3)
            ])
            dq = jac.T @ np.linalg.solve(
                jac @ jac.T + self._IK_DAMPING * np.eye(3), err
            )
            q = np.clip(q + dq, self.JOINT_LOWER, self.JOINT_UPPER)
        return q, float(np.linalg.norm(target - self.forward_kinematics(q)))

    def _check_limits(self, angles):
        for i, name in enumerate(self.JOINT_NAMES):
            lo, hi = self.JOINT_LOWER[i], self.JOINT_UPPER[i]
            if not (lo <= angles[name] <= hi):
                self.get_logger().warn(
                    f'{name}={angles[name]:.3f} rad outside limits [{lo:.3f}, {hi:.3f}]'
                )

    # --- Jog interface -----------------------------------------------------
    def _jog_cb(self, msg: PointStamped):
        p = msg.point
        try:
            angles = self.inverse_kinematics(p.x, p.y, p.z)
        except ValueError as exc:
            self.get_logger().warn(f'jog target rejected: {exc}')
            return
        self._send_trajectory(angles, self._jog_duration)
        self.get_logger().info(
            f'jog -> ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}): '
            f"yaw={angles['yaw_joint']:.3f} bicep={angles['bicep_joint']:.3f} "
            f"forearm={angles['forearm_joint']:.3f}"
        )

    def _send_trajectory(self, angles, duration):
        """Send a single-waypoint trajectory to the arm controller."""
        point = JointTrajectoryPoint()
        point.positions = [angles[j] for j in self.JOINT_NAMES]
        point.time_from_start = Duration(seconds=duration).to_msg()

        traj = JointTrajectory()
        traj.joint_names = list(self.JOINT_NAMES)
        traj.points = [point]
        self._traj_pub.publish(traj)

    def _joint_state_cb(self, msg: JointState):
        try:
            self._current_q = np.array(
                [msg.position[msg.name.index(j)] for j in self.JOINT_NAMES]
            )
        except (ValueError, IndexError):
            pass  # arm joints not in this message; keep the last known value

    # --- Draw interface (nav_msgs/Path) ------------------------------------
    def _path_cb(self, msg: Path):
        pts = [np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
               for p in msg.poses]
        if len(pts) < 2:
            self.get_logger().warn(f'draw_path needs >= 2 poses, got {len(pts)}')
            return
        if msg.header.frame_id and msg.header.frame_id != 'base_link':
            self.get_logger().warn(
                f"draw_path frame '{msg.header.frame_id}' != base_link; "
                'points are used as-is (no transform)'
            )

        samples = self._resample(pts, self._sample_spacing)
        try:
            angles = [self.inverse_kinematics(*s) for s in samples]
        except ValueError as exc:
            self.get_logger().warn(f'draw_path rejected: {exc}')
            return

        traj = self._build_trajectory(samples, angles)
        self._send_path_goal(traj)

    def _resample(self, pts, spacing):
        """Subdivide the polyline so consecutive samples are <= spacing apart,
        keeping the original waypoints. Zero-length segments are dropped."""
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            seg = b - a
            dist = float(np.linalg.norm(seg))
            if dist < 1e-9:
                continue
            steps = max(1, int(np.ceil(dist / spacing)))
            for i in range(1, steps + 1):
                out.append(a + seg * (i / steps))
        return out

    def _build_trajectory(self, samples, angles):
        """Build a JointTrajectory, timing each segment by what it does:

        * 'down'   -- vertical descent toward the paper -> lift_speed (slow, so
          the pen never slams down -- this is the only move that must be gentle);
        * 'up'     -- vertical lift away from the paper -> travel_speed (fast,
          the pen is leaving contact);
        * 'travel' -- planar move while the pen is lifted -> travel_speed (fast);
        * 'draw'   -- planar move at the paper -> draw_speed.

        Every segment is floored at min_segment_time so setpoints never arrive
        faster than the controller can execute (avoids skipping). The paper
        level is taken as the lowest z in the path.

        Line quality is improved two ways:
        * velocity feedforward -- each point carries the path-tangent joint
          velocity (central difference), so the controller tracks the line
          instead of guessing the spline shape from positions alone;
        * stops (zero velocity, plus an optional hold) are placed at pen
          contact/lift and at sharp corners, so corners stay crisp and the pen
          lands/leaves cleanly instead of overshooting or rounding off.
        """
        n = len(samples)
        z_paper = min(float(s[2]) for s in samples)
        verticals = {'up', 'down'}

        def seg_kind(a, b):
            dz = b[2] - a[2]
            dxy = float(np.hypot(b[0] - a[0], b[1] - a[1]))
            if abs(dz) > self._Z_EPS and abs(dz) >= dxy:
                return 'down' if dz < 0.0 else 'up'
            if min(a[2], b[2]) - z_paper > self._LIFT_MARGIN:
                return 'travel'
            return 'draw'

        def seg_time(a, b, kind):
            dz = abs(b[2] - a[2])
            dxy = float(np.hypot(b[0] - a[0], b[1] - a[1]))
            if kind == 'down':
                dt = dz / self._lift_speed                 # gentle approach only
            elif kind == 'up':
                dt = dz / self._travel_speed               # leaving paper -> fast
            else:
                speed = self._travel_speed if kind == 'travel' else self._draw_speed
                dt = dxy / speed
                if dz > self._Z_EPS:                       # diagonal: don't descend faster than lift_speed
                    dt = max(dt, dz / self._lift_speed)
            return max(dt, self._min_segment_time)

        def turn_angle(a, b, c):
            """Direction change (rad) of the planar path at b."""
            v1 = np.array([b[0] - a[0], b[1] - a[1]])
            v2 = np.array([c[0] - b[0], c[1] - b[1]])
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 < 1e-9 or n2 < 1e-9:
                return 0.0
            cos = float(np.dot(v1, v2) / (n1 * n2))
            return float(np.arccos(max(-1.0, min(1.0, cos))))

        qarr = [np.array([ang[j] for j in self.JOINT_NAMES]) for ang in angles]
        corner_rad = np.radians(self._corner_angle)

        # --- pass 1: build (q, time, stop) nodes ---------------------------
        # Approach to the first point: time it like a travel move, by the actual
        # distance from the current pen pose, floored by draw_lead_time.
        t = self._draw_lead_time
        if self._current_q is not None:
            approach = float(np.linalg.norm(samples[0] - self.forward_kinematics(self._current_q)))
            t = max(approach / self._travel_speed, self._draw_lead_time)

        nodes = [[qarr[0], t, True]]   # [positions, time_from_start, is_stop]
        for i in range(1, n):
            kind = seg_kind(samples[i - 1], samples[i])
            t += seg_time(samples[i - 1], samples[i], kind)

            contact = corner = False
            if 0 < i < n - 1:
                next_kind = seg_kind(samples[i], samples[i + 1])
                contact = ((kind == 'draw') != (next_kind == 'draw')
                           and (kind in verticals or next_kind in verticals))
                if kind == 'draw' and next_kind == 'draw':
                    corner = turn_angle(samples[i - 1], samples[i], samples[i + 1]) > corner_rad

            is_last = (i == n - 1)
            nodes.append([qarr[i], t, contact or corner or is_last])

            # Optional hold to guarantee a clean stop.
            if contact and self._contact_dwell > 0.0:
                t += self._contact_dwell
                nodes.append([qarr[i], t, True])
            elif corner and self._corner_dwell > 0.0:
                t += self._corner_dwell
                nodes.append([qarr[i], t, True])

        # --- pass 2: emit with velocity feedforward ------------------------
        traj = JointTrajectory()
        traj.joint_names = list(self.JOINT_NAMES)
        m = len(nodes)
        for k in range(m):
            q, tt, stop = nodes[k]
            if stop or k == 0 or k == m - 1:
                vel = np.zeros(3)
            else:
                qp, tp, _ = nodes[k - 1]
                qn, tn, _ = nodes[k + 1]
                dt = tn - tp
                vel = (qn - qp) / dt if dt > 1e-9 else np.zeros(3)
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in q]
            point.velocities = [float(x) for x in vel]
            point.time_from_start = Duration(seconds=tt).to_msg()
            traj.points.append(point)
        return traj

    def _send_path_goal(self, traj):
        if not self._traj_action.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'arm_controller FollowJointTrajectory action server unavailable'
            )
            return
        # Cancel any stroke still in flight before starting a new one.
        if self._active_goal is not None:
            self._active_goal.cancel_goal_async()
            self._active_goal = None

        last = traj.points[-1].time_from_start
        total = last.sec + last.nanosec * 1e-9
        self.get_logger().info(
            f'draw_path: {len(traj.points)} samples, ~{total:.1f}s at '
            f'{self._draw_speed:.3f} m/s'
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._traj_action.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('draw_path goal rejected by arm controller')
            return
        self._active_goal = handle
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'draw_path finished (error_code={result.error_code})')
        self._active_goal = None


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(InverseKinematicsControl())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
