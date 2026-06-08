# generate_trajectory

Connects detected objects on the table with a collision-free path that the PR arm
draws. An overhead camera + YOLO detect objects; this package converts a chosen
pair of detections into a `base_link` path (A* around an obstacle) using the
**fixed** camera↔base calibration from `pr_calibration` — no runtime marker. Detection
runs continuously; connections are requested **on demand** via a service.

```
/yolo/detections ─┐
/camera_info ─────┤  connect_objects_server  ── /trajectory_path (viz, base_link)
TF base_link←camera┘   (pixel→base via the      └─ /inverse_kinematics_control/draw_path (arm)
                        static calibration,
ConnectObjects srv ───►  then A* path)           also: /detected_classes, /detected_objects
```

## Run

Bring the pipeline up once (camera + static TF + YOLO + this server), leave running:
```bash
ros2 launch generate_trajectory connect_objects.launch.py        # model:=yolov8m.pt table_z:=0.02
```
Prerequisites: `pr_calibration` calibrated (intrinsics + `camera_extrinsics.yaml`), and the
arm running separately (`ros2 launch pr_controller hardware.launch.py` / `sim.launch.py`).

Then connect objects on demand, repeatably:
```bash
ros2 service call /connect_objects pr_interfaces/srv/ConnectObjects \
    "{start: apple, goal: cup, obstacle: bottle}"
ros2 service call /connect_objects pr_interfaces/srv/ConnectObjects \
    "{start: pen, goal: book, obstacle: ''}"        # empty obstacle = nothing to avoid
```
Labels are case-insensitive substrings of the YOLO class names. The reply
`{success, message}` reports the drawn path (or why it was rejected).

## Service
- **`/connect_objects`** (`pr_interfaces/srv/ConnectObjects`): `start`, `goal`, `obstacle`
  → `success`, `message`. Plans against the *latest* detections, projects to `base_link`,
  publishes the path, and the arm draws it.

## Topics
| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/yolo/detections` | `yolo_msgs/DetectionArray` | sub | YOLO 2D detections |
| `/camera_info` | `sensor_msgs/CameraInfo` | sub | camera intrinsics (from `pr_calibration`) |
| `base_link ← camera` (TF) | — | sub | the static calibration transform |
| `/detected_classes` | `pr_interfaces/DetectedClasses` | pub | **`string[]` of currently detected class names** |
| `/detected_objects` | `visualization_msgs/MarkerArray` | pub | each detection's label at its `base_link` table position (RViz) |
| `/trajectory_path` | `nav_msgs/Path` | pub | planned path in `base_link` (visualization) |
| `/inverse_kinematics_control/draw_path` | `nav_msgs/Path` | pub | the path sent to the arm |

See what's detectable right now (which labels you can pass to the service):
```bash
ros2 topic echo /detected_classes
```

## Parameters (`connect_objects_server`)
| Parameter | Default | Meaning |
|---|---|---|
| `table_z` | 0.02 | object/draw plane height in `base_link` (set to the pen drawing-contact height) |
| `camera_frame` / `base_frame` | `camera` / `base_link` | TF frames for the static calibration |
| `image_width` / `image_height` | 640 / 480 | A* workspace size (px) |
| `grid_resolution` | 5 | A* grid cell size (px) |
| `detections_topic` / `camera_info_topic` | `/yolo/detections` / `/camera_info` | inputs |
| `draw_topic` | `/inverse_kinematics_control/draw_path` | arm path output |
| `detected_classes_topic` / `detected_objects_topic` | `/detected_classes` / `/detected_objects` | detection outputs |
| `service_name` | `/connect_objects` | the connect service |

## Notes
- Detected objects must lie in the arm's −y workspace; out-of-reach points are rejected by the IK.
- Path planning (`path_planning.PathPlanner`) is A* on a pixel grid; the result is converted to
  `base_link` by intersecting each pixel ray with the `z = table_z` plane using the static calibration.
- Monocular only — no depth; objects are assumed to lie on the table plane.
