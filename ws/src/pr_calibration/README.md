# pr_calibration

Camera↔robot calibration for the PR manipulator: **intrinsics** (checkerboard) +
**extrinsics** (AprilTag at a known offset from the base), producing a persistent
`camera → base_link` static transform the rest of the pipeline uses. No runtime
marker is needed once calibrated — the overhead camera is fixed.

## One-time calibration

1. **Intrinsics** — print `files/checkerboard_a4.pdf` (7×9 inner corners, 18.6 mm
   squares) and run:
   ```bash
   ros2 launch pr_calibration intrinsic_calibration.launch.py
   ```
   Move the board until X/Y/Size/Skew are green → **Calibrate** → **Save** → **Commit**.
   Extract `/tmp/calibrationdata.tar.gz` and copy `ost.yaml`'s matrix + distortion into
   `config/camera_calibration_params.yaml`, then `colcon build --packages-select pr_calibration`.

2. **Extrinsics** — print `scripts/generate_calibration_sheet.py`'s output
   (`ros2 run pr_calibration generate_calibration_sheet.py`), place the robot base on the
   footprint, and run:
   ```bash
   ros2 launch pr_calibration calibrate.launch.py
   ```
   This writes `config/camera_extrinsics.yaml` (`base_link → camera`). If the logged camera
   height is negative, re-run with `-p invert_tag_z:=true`.

## Using the calibration (runtime)

```bash
ros2 launch pr_calibration camera.launch.py      # /image_raw + calibrated /camera_info
ros2 launch pr_calibration static_tf.launch.py   # publishes base_link -> camera (from the yaml)
```
The full detect→connect→draw pipeline is brought up by
`ros2 launch generate_trajectory connect_objects.launch.py` (which includes both of the
above plus YOLO and the path planner).

## Config
- `config/camera_calibration_params.yaml` — camera intrinsics (camera_info).
- `config/camera_params.yaml` — usb_cam settings (`/dev/video0`, frame `camera`).
- `config/layout.yaml` — AprilTag family/size/id and its fixed offset from `base_link`.
- `config/camera_extrinsics.yaml` — **output** of the extrinsic calibration.
