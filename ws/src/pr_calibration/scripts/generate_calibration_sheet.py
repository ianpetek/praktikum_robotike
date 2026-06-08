#!/usr/bin/env python3
"""Generate the A4 calibration sheet: the robot base footprint at one end and an
AprilTag at the other, separated by the exact offset defined in config/layout.yaml.

Print at **100% scale** (no "fit to page"), place the robot base on the printed
outline, and the tag is then at a known pose relative to base_link. Verify the
printout with the 100 mm ruler before trusting distances.

Standalone tool (no rclpy):

    ros2 run pr_calibration generate_calibration_sheet.py
    ros2 run pr_calibration generate_calibration_sheet.py --out /tmp/sheet.pdf
"""
import argparse
import os

import cv2
import numpy as np
import yaml
from PIL import Image
from stl import mesh as stlmesh
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas
from ament_index_python.packages import get_package_share_directory

# Page-frame placement: where base_link origin sits on the A4 (mm from bottom-left),
# with +x_base -> +page_x (right) and +y_base -> +page_y (up).
BASE_ORIGIN_PAGE = (105.0, 72.0)


def base_footprint_polygon(stl_path, px_per_mm=5.0):
    """Return the base outline (list of (x_mm, y_mm) in base_link XY) as the
    top-down silhouette of base_link.stl."""
    tris = stlmesh.Mesh.from_file(stl_path).vectors  # (N, 3, 3), mm
    xy = tris[:, :, :2]
    minx, miny = float(xy[..., 0].min()), float(xy[..., 1].min())
    maxx, maxy = float(xy[..., 0].max()), float(xy[..., 1].max())
    pad = 3
    w = int((maxx - minx) * px_per_mm) + 2 * pad
    h = int((maxy - miny) * px_per_mm) + 2 * pad
    img = np.zeros((h, w), np.uint8)
    px = np.empty_like(xy, dtype=np.int32)
    px[..., 0] = ((xy[..., 0] - minx) * px_per_mm + pad).astype(np.int32)
    px[..., 1] = ((xy[..., 1] - miny) * px_per_mm + pad).astype(np.int32)
    cv2.fillPoly(img, [tri for tri in px], 255)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(contours, key=cv2.contourArea)
    return [((p[0] - pad) / px_per_mm + minx, (p[1] - pad) / px_per_mm + miny)
            for p in c[:, 0, :]]


def tag_image(family, tag_id, px=1200):
    dict_id = getattr(cv2.aruco, f'DICT_APRILTAG_{family}')
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    img = cv2.aruco.drawMarker(aruco_dict, int(tag_id), px)
    return Image.fromarray(img)


def b2p(bx, by):
    """base_link mm -> page mm."""
    return BASE_ORIGIN_PAGE[0] + bx, BASE_ORIGIN_PAGE[1] + by


def main():
    share = get_package_share_directory('pr_calibration')
    parser = argparse.ArgumentParser(description='Generate the A4 calibration sheet.')
    parser.add_argument('--layout', default=os.path.join(share, 'config', 'layout.yaml'))
    parser.add_argument('--mesh', default=os.path.join(
        get_package_share_directory('pr_description'), 'meshes', 'base_link.stl'))
    parser.add_argument('--out', default=os.path.join(share, 'calibration_sheet_a4.pdf'))
    args = parser.parse_args()

    with open(args.layout) as f:
        cfg = yaml.safe_load(f)
    tag_size = float(cfg['tag_size_mm'])
    tx = float(cfg['tag_center_in_base_mm']['x'])
    ty = float(cfg['tag_center_in_base_mm']['y'])
    tag_yaw = float(cfg.get('tag_yaw_deg', 0.0))

    poly = base_footprint_polygon(args.mesh)
    tag_im = tag_image(cfg['tag_family'], cfg['tag_id'])

    c = canvas.Canvas(args.out, pagesize=A4)

    # --- title / instructions ---
    c.setFont('Helvetica-Bold', 13)
    c.drawString(15 * mm, 285 * mm, 'PR manipulator — camera calibration sheet')
    c.setFont('Helvetica', 8)
    for i, line in enumerate([
        'Print at 100% (no scaling / "fit to page"). Verify the 100 mm ruler below.',
        'Place the robot base on the outline; align the FRONT arrow with the workspace (-y).',
        f'Tag {cfg["tag_family"]} id {cfg["tag_id"]}, {tag_size:.0f} mm, '
        f'center at base ({tx:.0f}, {ty:.0f}) mm.',
    ]):
        c.drawString(15 * mm, (280 - 4 * i) * mm, line)

    # --- base footprint outline (1:1) ---
    c.setLineWidth(0.8)
    path = c.beginPath()
    x0, y0 = b2p(*poly[0])
    path.moveTo(x0 * mm, y0 * mm)
    for (bx, by) in poly[1:]:
        px, py = b2p(bx, by)
        path.lineTo(px * mm, py * mm)
    path.close()
    c.drawPath(path, stroke=1, fill=0)

    # base_link origin crosshair
    ox, oy = b2p(0, 0)
    c.setLineWidth(0.4)
    c.line((ox - 6) * mm, oy * mm, (ox + 6) * mm, oy * mm)
    c.line(ox * mm, (oy - 6) * mm, ox * mm, (oy + 6) * mm)
    c.setFont('Helvetica', 6)
    c.drawString((ox + 7) * mm, (oy + 1) * mm, 'base_link (0,0)')

    # yaw axis marker at base (0, +12 mm)
    yx, yy = b2p(0, 12)
    c.circle(yx * mm, yy * mm, 1.5 * mm, stroke=1, fill=0)
    c.drawString((yx + 3) * mm, yy * mm, 'yaw axis')

    # orientation arrows: FRONT (-y, workspace) and TAG (+y)
    fx, fy = b2p(0, -50)
    c.line(ox * mm, oy * mm, fx * mm, fy * mm)
    c.drawString((fx + 2) * mm, fy * mm, 'FRONT  (workspace, -y)')
    c.line(ox * mm, oy * mm, *(np.array(b2p(0, 30)) * mm))
    c.drawString((b2p(0, 30)[0] + 2) * mm, b2p(0, 30)[1] * mm, '+y  -> tag')

    # --- AprilTag at the designed offset ---
    if abs(tag_yaw) > 1e-6:
        tag_im = tag_im.rotate(-tag_yaw, expand=False, fillcolor=255)
    cx, cy = b2p(tx, ty)
    c.drawImage(ImageReader(tag_im),
                (cx - tag_size / 2) * mm, (cy - tag_size / 2) * mm,
                tag_size * mm, tag_size * mm)
    c.setLineWidth(0.3)
    c.rect((cx - tag_size / 2) * mm, (cy - tag_size / 2) * mm,
           tag_size * mm, tag_size * mm, stroke=1, fill=0)
    c.setFont('Helvetica', 6)
    c.line((cx - 5) * mm, cy * mm, (cx + 5) * mm, cy * mm)
    c.line(cx * mm, (cy - 5) * mm, cx * mm, (cy + 5) * mm)
    c.drawString((cx + tag_size / 2 + 2) * mm, cy * mm,
                 f'tag center ({tx:.0f}, {ty:.0f}) mm in base_link')

    # --- 100 mm print-scale ruler (bottom) ---
    rx, ry = 55.0, 12.0
    c.setLineWidth(0.6)
    c.line(rx * mm, ry * mm, (rx + 100) * mm, ry * mm)
    for t in range(0, 101, 10):
        c.line((rx + t) * mm, ry * mm, (rx + t) * mm, (ry + (3 if t % 50 == 0 else 2)) * mm)
    c.setFont('Helvetica', 7)
    c.drawString(rx * mm, (ry + 4) * mm, '0')
    c.drawString((rx + 100 - 4) * mm, (ry + 4) * mm, '100 mm  (verify with a ruler)')

    c.showPage()
    c.save()
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
