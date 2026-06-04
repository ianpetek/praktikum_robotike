#!/usr/bin/env python3
"""Demo: draw simple capital text with the PR manipulator.

Builds a nav_msgs/Path of pen-tip waypoints (base frame) from a single-stroke
capital font and publishes it to the inverse_kinematics_control draw_path topic.
Pen up/down is expressed purely through Z: strokes are drawn at DRAW_Z and the
pen lifts to TRAVEL_Z to move between them.

The text is laid out on the horizontal plane in front of the robot (the -y
side): character advance runs along base +x, character height along base -y.

Tunables are the module constants below; the most useful ones (text, draw_z)
can also be overridden on the command line:

    ros2 run pr_controller draw_text_demo.py HELLO
    ros2 run pr_controller draw_text_demo.py HI --draw-z 0.045
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# --- Tunables ---------------------------------------------------------------
DRAW_Z = 0.016         # pen-down height (m) -- found to draw well empirically
TRAVEL_Z = 0.05        # pen-up height (m) for moving between strokes
CHAR_HEIGHT = 0.045    # letter height along base -y (m)
CHAR_WIDTH = 0.030     # letter width along base +x (m)
CHAR_GAP = 0.003       # gap between letters (m)
BASELINE_Y = -0.20     # base-frame y of the text baseline (closest edge)
CENTER_X = 0.0         # base-frame x the text is centred on
# Reading orientation. The default (FLIP_X=True, FLIP_Y=False) reads correctly
# from the robot's side. Viewing from the opposite side of the table is a 180 deg
# rotation, so set FLIP_X=False *and* FLIP_Y=True for that side.
FLIP_X = False          # mirror left-right
FLIP_Y = True         # mirror top-bottom
DEFAULT_TEXT = 'HELLO'
FRAME = 'base_link'
TOPIC = '/inverse_kinematics_control/draw_path'

# Soft reachability box at low z (from the workspace probe); only warns.
SAFE_X = (-0.15, 0.15)
SAFE_Y = (-0.30, -0.16)

# --- Single-stroke capital font ---------------------------------------------
# Each glyph is a list of strokes; each stroke is a polyline of (u, v) points in
# the unit cell, u in [0,1] (left->right), v in [0,1] (bottom->top).


def _ellipse(cx, cy, rx, ry, n=24, start_deg=90.0):
    """A closed elliptical polyline (n segments) for round glyphs."""
    pts = []
    for i in range(n + 1):
        a = math.radians(start_deg) + 2.0 * math.pi * i / n
        pts.append((round(cx + rx * math.cos(a), 4), round(cy + ry * math.sin(a), 4)))
    return pts


# The cell is taller than wide, so a physical circle needs a wider radius in u
# than in v. Stretch the horizontal cell-radius by this aspect ratio.
_ASPECT = CHAR_HEIGHT / CHAR_WIDTH


def _circle(ry, n=28):
    """A closed polyline that is circular in metres: vertical cell-radius ry,
    horizontal cell-radius ry*_ASPECT. Left edge at u=0 (like other glyphs)."""
    rx = ry * _ASPECT
    return _ellipse(rx, 0.5, rx, ry, n=n)


FONT = {
    ' ': [],
    'A': [[(0, 0), (0.5, 1), (1, 0)], [(0.2, 0.4), (0.8, 0.4)]],
    'B': [[(0, 0), (0, 1), (0.6, 1), (0.8, 0.85), (0.8, 0.6), (0.6, 0.5), (0, 0.5)],
          [(0, 0.5), (0.65, 0.5), (0.85, 0.35), (0.85, 0.15), (0.6, 0), (0, 0)]],
    'C': [[(0.9, 0.85), (0.6, 1), (0.3, 1), (0.05, 0.75), (0.05, 0.25), (0.3, 0), (0.6, 0), (0.9, 0.15)]],
    'D': [[(0, 0), (0, 1), (0.55, 1), (0.85, 0.7), (0.85, 0.3), (0.55, 0), (0, 0)]],
    'E': [[(0.85, 1), (0, 1), (0, 0), (0.85, 0)], [(0, 0.5), (0.6, 0.5)]],
    'F': [[(0.85, 1), (0, 1), (0, 0)], [(0, 0.5), (0.6, 0.5)]],
    'G': [[(0.9, 0.85), (0.6, 1), (0.3, 1), (0.05, 0.75), (0.05, 0.25), (0.3, 0),
           (0.65, 0), (0.9, 0.2), (0.9, 0.45), (0.6, 0.45)]],
    'H': [[(0, 0), (0, 1)], [(0.8, 0), (0.8, 1)], [(0, 0.5), (0.8, 0.5)]],
    'I': [[(0.4, 0), (0.4, 1)], [(0.15, 1), (0.65, 1)], [(0.15, 0), (0.65, 0)]],
    'J': [[(0.7, 1), (0.7, 0.25), (0.5, 0), (0.25, 0), (0.1, 0.2)]],
    'K': [[(0, 0), (0, 1)], [(0.75, 1), (0, 0.5), (0.8, 0)]],
    'L': [[(0, 1), (0, 0), (0.75, 0)]],
    'M': [[(0, 0), (0, 1), (0.4, 0.45), (0.8, 1), (0.8, 0)]],
    'N': [[(0, 0), (0, 1), (0.8, 0), (0.8, 1)]],
    'O': [_circle(0.35)],
    'P': [[(0, 0), (0, 1), (0.6, 1), (0.8, 0.82), (0.8, 0.62), (0.6, 0.5), (0, 0.5)]],
    'Q': [_circle(0.35), [(0.95, 0.28), (1.3, -0.05)]],
    'R': [[(0, 0), (0, 1), (0.6, 1), (0.8, 0.82), (0.8, 0.62), (0.6, 0.5), (0, 0.5)],
          [(0.35, 0.5), (0.8, 0)]],
    'S': [[(0.85, 0.82), (0.6, 1), (0.25, 1), (0.05, 0.82), (0.1, 0.58), (0.3, 0.5),
           (0.6, 0.45), (0.8, 0.32), (0.8, 0.15), (0.55, 0), (0.2, 0), (0.05, 0.15)]],
    'T': [[(0, 1), (0.8, 1)], [(0.4, 1), (0.4, 0)]],
    'U': [[(0, 1), (0, 0.25), (0.2, 0.05), (0.5, 0), (0.7, 0.05), (0.8, 0.25), (0.8, 1)]],
    'V': [[(0, 1), (0.4, 0), (0.8, 1)]],
    'W': [[(0, 1), (0.2, 0), (0.4, 0.55), (0.6, 0), (0.8, 1)]],
    'X': [[(0, 0), (0.8, 1)], [(0, 1), (0.8, 0)]],
    'Y': [[(0, 1), (0.4, 0.5), (0.8, 1)], [(0.4, 0.5), (0.4, 0)]],
    'Z': [[(0, 1), (0.8, 1), (0, 0), (0.8, 0)]],
    '0': [_ellipse(0.4, 0.5, 0.30, 0.48)],
    '1': [[(0.2, 0.8), (0.45, 1), (0.45, 0)], [(0.2, 0), (0.7, 0)]],
    '2': [[(0.05, 0.8), (0.3, 1), (0.6, 1), (0.8, 0.8), (0.8, 0.6), (0.05, 0), (0.8, 0)]],
    '3': [[(0.05, 0.85), (0.35, 1), (0.65, 0.88), (0.4, 0.55), (0.7, 0.45),
           (0.7, 0.15), (0.4, 0), (0.1, 0.12)]],
    '4': [[(0.6, 0), (0.6, 1), (0.05, 0.35), (0.8, 0.35)]],
    '5': [[(0.8, 1), (0.1, 1), (0.1, 0.55), (0.5, 0.6), (0.75, 0.45),
           (0.75, 0.15), (0.45, 0), (0.1, 0.12)]],
    '6': [[(0.7, 0.9), (0.4, 1), (0.15, 0.75), (0.1, 0.3), (0.35, 0),
           (0.65, 0.1), (0.7, 0.35), (0.45, 0.5), (0.15, 0.4)]],
    '7': [[(0.05, 1), (0.8, 1), (0.35, 0)]],
    '8': [[(0.4, 0.5), (0.15, 0.65), (0.2, 0.9), (0.5, 1), (0.7, 0.85), (0.6, 0.6),
           (0.4, 0.5), (0.15, 0.35), (0.15, 0.12), (0.45, 0), (0.75, 0.15), (0.65, 0.4), (0.4, 0.5)]],
    '9': [[(0.65, 0.6), (0.35, 0.5), (0.15, 0.65), (0.3, 0.9), (0.6, 1),
           (0.7, 0.6), (0.65, 0.2), (0.35, 0)]],
}


def build_waypoints(text, draw_z, travel_z):
    """Return a list of (x, y, z) pen-tip waypoints in the base frame."""
    chars = list(text.upper())
    advance = CHAR_WIDTH + CHAR_GAP
    total_w = max(0.0, len(chars) * advance - CHAR_GAP)
    x0 = CENTER_X - total_w / 2.0

    pts = []
    cursor = 0.0
    unknown = set()
    for ch in chars:
        strokes = FONT.get(ch)
        if strokes is None:
            unknown.add(ch)
            strokes = []  # treat unknown glyphs as a blank space
        for stroke in strokes:
            def to_base(uv):
                u, v = uv
                local_x = cursor + u * CHAR_WIDTH        # 0 .. total_w across the line
                if FLIP_X:
                    local_x = total_w - local_x          # mirror the whole line about its center
                if FLIP_Y:
                    v = 1.0 - v                          # mirror each glyph top-bottom
                return (x0 + local_x, BASELINE_Y - v * CHAR_HEIGHT)
            sx, sy = to_base(stroke[0])
            pts.append((sx, sy, travel_z))   # pen up, move over start
            pts.append((sx, sy, draw_z))     # pen down
            for uv in stroke[1:]:
                bx, by = to_base(uv)
                pts.append((bx, by, draw_z))
            pts.append((bx, by, travel_z))   # pen up at stroke end
        cursor += advance
    return pts, unknown


def make_path(pts):
    path = Path()
    path.header.frame_id = FRAME
    for (x, y, z) in pts:
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description='Draw capital text with the PR arm.')
    parser.add_argument('text', nargs='?', default=DEFAULT_TEXT, help='text to draw')
    parser.add_argument('--draw-z', type=float, default=DRAW_Z, help='pen-down height (m)')
    parser.add_argument('--travel-z', type=float, default=TRAVEL_Z, help='pen-up height (m)')
    parser.add_argument('--topic', default=TOPIC, help='draw_path topic')
    args = parser.parse_args(remove_ros_args(sys.argv if argv is None else argv)[1:])

    pts, unknown = build_waypoints(args.text, args.draw_z, args.travel_z)
    if not pts:
        print('Nothing to draw (empty/blank text).')
        return

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    print(f"text '{args.text}': {len(pts)} waypoints, "
          f"x in [{min(xs):+.3f}, {max(xs):+.3f}], y in [{min(ys):+.3f}, {max(ys):+.3f}], "
          f"draw_z={args.draw_z}, travel_z={args.travel_z}")
    if unknown:
        print(f'  warning: no glyph for {sorted(unknown)} -> drawn as spaces')
    if not (SAFE_X[0] <= min(xs) and max(xs) <= SAFE_X[1]
            and SAFE_Y[0] <= min(ys) and max(ys) <= SAFE_Y[1]):
        print(f'  warning: text extends past the verified-reachable box '
              f'(x{SAFE_X}, y{SAFE_Y}); the IK node may reject some points')

    rclpy.init()
    node = Node('draw_text_demo')
    pub = node.create_publisher(Path, args.topic, 10)

    # Wait for the IK node to subscribe so the one-shot publish is delivered.
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().warn(f'no subscriber on {args.topic}; publishing anyway')

    pub.publish(make_path(pts))
    node.get_logger().info(f"published '{args.text}' to {args.topic}")
    # Spin briefly to flush the message before exiting.
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
