# PR Manipulator — Drawing Robot

Software for a **3-DOF RRR manipulator** with a passive pen wrist. A passive mechanism
keeps the pen **perpendicular to the ground at all times**, so the arm only has to
position the pen tip in space — the pen stays vertical on its own. The robot can be
driven in **Gazebo simulation** or on **real hardware** (Dynamixel XL430 servos), and
its headline use case is **drawing / plotting**: send it a path of 3-D points and it
traces them with the pen.

```
        yaw_joint (Z)         actuated, base rotation
           │
        bicep_joint (X)       actuated, "shoulder"
           │
        forearm_joint (X)     actuated, "elbow"
           │
        pen_mount_joint       PASSIVE — kept vertical by the mechanism
           │
        pen → pen_tip         the drawing point
```

---

## Repository layout

```
.
├── .devcontainer/            ROS 2 Jazzy dev container (Docker Compose)
│   ├── devcontainer.json
│   └── docker-compose.yml
└── ws/                       colcon workspace
    └── src/
        ├── pr_description/    robot model, world, controllers, sim/viz launches
        │   ├── urdf/pr_robot.xacro      links, joints, ros2_control blocks
        │   ├── config/controller.yaml   ros2_control controller definitions
        │   ├── meshes/                  STL visual/collision meshes
        │   ├── worlds/room.sdf          Gazebo world
        │   └── launch/                  gazebo / visualize / controller launches
        └── pr_controller/     the application: nodes + top-level launches
            ├── scripts/
            │   ├── inverse_kinematics_control.py   IK + jog + path drawing
            │   ├── pen_mount_broadcaster.py        keeps the passive pen vertical
            │   └── draw_text_demo.py               demo: draw capital text
            ├── launch/sim.launch.py        full simulation bring-up
            ├── launch/hardware.launch.py   full hardware bring-up
            └── config/hardware.rviz        RViz layout for hardware
```

`pr_description` is the **robot** (model + low-level building blocks); `pr_controller`
is the **application** (the IK/drawing logic and the launches you actually run).

---

## 1. Dev container

The project runs in a VS Code dev container based on **ROS 2 Jazzy**.

- Open the repository in VS Code and choose **"Reopen in Container"** (Dev Containers
  extension). The container is defined by [`.devcontainer/docker-compose.yml`](.devcontainer/docker-compose.yml)
  and built from `Dockerfile-jazzy`.
- The workspace is mounted at `/workspace`.
- **GUI** (Gazebo / RViz): the compose file forwards X11 (`/tmp/.X11-unix`, `DISPLAY`)
  and requests an NVIDIA GPU (optional — falls back to software rendering).
- **Hardware**: `/dev/ttyUSB0` is passed through and the container joins the `dialout`
  group, so the Dynamixel U2D2 adapter is available inside.

If you build the image manually:
```bash
cd .devcontainer && docker build . -t ros2-jazzy-dev:latest -f Dockerfile-jazzy
```

---

## 2. Build the workspace

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash          # ROS 2 environment
cd /workspace/ws
colcon build --symlink-install            # build both packages
source install/setup.bash                 # overlay the built workspace
```

> **`--symlink-install` matters.** Python scripts are symlinked into the install space,
> so editing a `.py` takes effect on the next run **without rebuilding**. You only need
> to `colcon build` again when you **add a new file**, change `CMakeLists.txt`/`package.xml`,
> or edit the URDF's install target.

Source both lines in **every new terminal** (or add them to `~/.zshrc`).

---

## 3. Run it

### Simulation (Gazebo)

```bash
ros2 launch pr_controller sim.launch.py
```

This brings up Gazebo with the robot, the controllers, and the application nodes.
Wait for the controllers to finish spawning (a few seconds), then send a target — see
[§5 Drawing](#5-drawing).

### Hardware (Dynamixel)

Make sure the servos are powered and the U2D2 is on `/dev/ttyUSB0`, then:

```bash
ros2 launch pr_controller hardware.launch.py
```

This starts `ros2_control` against the real servos, the controllers, the application
nodes, and RViz. The IK/drawing commands are identical to simulation.

### Just look at the model (no sim, no hardware)

```bash
ros2 launch pr_description visualize.launch.py
```

Opens RViz with a `joint_state_publisher_gui` so you can drag the joints and inspect
the URDF/TF. Useful for sanity-checking the model.

---

## 4. How it fits together

```
   target point / path
          │
          ▼
 ┌──────────────────────────────┐
 │ inverse_kinematics_control    │  solves IK, builds a timed JointTrajectory
 └──────────────────────────────┘
          │ FollowJointTrajectory (draw)  /  JointTrajectory topic (jog)
          ▼
 ┌──────────────────────────────┐
 │ arm_controller (JTC)          │  yaw / bicep / forearm
 └──────────────────────────────┘
          │ commands
          ▼
     Gazebo  or  Dynamixel hardware
          │ /joint_states
          ▼
 ┌──────────────────────────────┐
 │ pen_mount_broadcaster         │  drives the passive pen_mount = bicep + forearm
 └──────────────────────────────┘  (keeps the pen vertical)
```

### Controllers (`config/controller.yaml`)

| Controller | Type | Joints |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | publishes `/joint_states` |
| `arm_controller` | `JointTrajectoryController` | `yaw_joint`, `bicep_joint`, `forearm_joint` |
| `pen_mount_controller` | `ForwardCommandController` | `pen_mount_joint` (sim only) |

Update rate is 50 Hz. On **hardware** only the three actuated joints exist as servos
(`forearm` id 11, `yaw` id 12, `bicep` id 13); the pen wrist is mechanical.

### Nodes

**`inverse_kinematics_control.py`** — the brain. It:
- Builds the **exact forward kinematics** from the URDF transform chain and **inverts it
  numerically** (damped least-squares with joint-limit clamping and several seeds).
  This avoids the sign/offset pitfalls of a hand-derived closed form and handles the
  passive wrist exactly.
- Exposes two interfaces (both in the **base frame**, Z = pen height):
  - **Jog** — publish a `geometry_msgs/PointStamped` to `~/target_point`; it solves IK
    for that pen-tip target and sends a single-point trajectory.
  - **Draw** — publish a `nav_msgs/Path` to `~/draw_path`; it resamples the polyline,
    solves IK per sample, times the motion, and runs the whole stroke through the arm
    controller's `FollowJointTrajectory` **action** (with cancel-on-new-path).
- Tracks `/joint_states` so the opening move is timed from the arm's actual pose.
- Rejects unreachable targets (logs the offending point).

**`pen_mount_broadcaster.py`** — keeps the pen vertical by mirroring the arm:
it watches `/joint_states` and commands `pen_mount = bicep + forearm`. In **sim** it
publishes to `/pen_mount_controller/commands`; on **hardware** it republishes a
`pen_mount_joint` state (the wrist has no encoder) so TF/RViz stay correct.

**`draw_text_demo.py`** — a self-contained demo that turns a string into a `nav_msgs/Path`
of pen strokes and publishes it. See [§5](#5-drawing).

### Launch files

- `pr_controller/sim.launch.py` — includes `pr_description/gazebo.launch.py`, spawns the
  three controllers, and starts the application nodes. **Main sim entry point.**
- `pr_controller/hardware.launch.py` — `robot_state_publisher` + `ros2_control_node`
  against the Dynamixels, the controllers, the application nodes, and RViz. **Main
  hardware entry point.**
- `pr_description/gazebo.launch.py` — launches Gazebo with `worlds/room.sdf`, the
  clock bridge, `robot_state_publisher`, and spawns the robot. (Building block used by
  `sim.launch.py`.)
- `pr_description/visualize.launch.py` — `robot_state_publisher` + joint sliders +
  RViz for inspecting the model only.
- `pr_description/controller.launch.py` — a standalone controller spawner from an
  earlier setup; its controller names predate the current `controller.yaml`, so prefer
  the top-level launches above.

---

## 5. Drawing

> Reachable space is the **−y side** of the base (the arm's home reach direction). A good
> drawing plane is around `z ≈ 0.02`. Targets are in metres in the `base_link` frame.

### Jog to a single point
```bash
ros2 topic pub --once /inverse_kinematics_control/target_point geometry_msgs/PointStamped \
  '{header: {frame_id: base_link}, point: {x: 0.0, y: -0.20, z: 0.05}}'
```

### Draw a path
Publish a `nav_msgs/Path`; Z encodes pen up/down (lift Z to travel, lower to draw):
```bash
ros2 topic pub --once /inverse_kinematics_control/draw_path nav_msgs/Path \
'{header: {frame_id: base_link}, poses: [
  {pose: {position: {x: 0.0,  y: -0.18, z: 0.02}}},
  {pose: {position: {x: 0.06, y: -0.18, z: 0.02}}}
]}'
```
You can also visualize a `Path` in RViz by adding a *Path* display on
`/inverse_kinematics_control/draw_path`.

### Draw text (demo)
```bash
ros2 run pr_controller draw_text_demo.py HELLO
ros2 run pr_controller draw_text_demo.py "HI" --draw-z 0.02
```
Tunables (text size, layout, draw/travel heights, reading orientation) are constants at
the top of [`draw_text_demo.py`](ws/src/pr_controller/scripts/draw_text_demo.py); a
single-stroke capital font (A–Z, 0–9) lives in the `FONT` table.

---

## 6. Tuning the motion (live parameters)

All of these are parameters on `inverse_kinematics_control` and can be changed **live**
(they apply to the next path), e.g.:
```bash
ros2 param set /inverse_kinematics_control draw_speed 0.04
```

| Parameter | Default | Meaning |
|---|---|---|
| `draw_speed` | 0.05 m/s | pen-tip speed while drawing (on paper) |
| `travel_speed` | 0.12 m/s | pen-up moves between strokes, and upward lifts |
| `lift_speed` | 0.03 m/s | **downward** descent to the paper (kept gentle so it doesn't slam) |
| `contact_dwell` | 0.15 s | settle/hold at pen down & up |
| `corner_angle` | 35° | turns sharper than this get a stop (crisp corners); curves below it stay smooth |
| `corner_dwell` | 0.05 s | hold at a sharp corner |
| `sample_spacing` | 0.005 m | Cartesian resample step — smaller = smoother curves, more points |
| `min_segment_time` | 0.02 s | floor per trajectory segment; sets the **top speed** ≈ `sample_spacing / min_segment_time` |
| `draw_lead_time` | 0.5 s | minimum time to reach the first point |
| `jog_duration` | 2.0 s | time for a single jog move |

**Speed model in one line:** drawing uses `draw_speed`; everything in the air
(approach, travel, lift-up) uses `travel_speed`; only the descent uses `lift_speed`.
The hard ceiling on all of them is `sample_spacing / min_segment_time` (≈ 0.25 m/s at
the defaults) — lower `min_segment_time` or raise `sample_spacing` to go faster.

**Line quality:** each trajectory point carries velocity feedforward (path tangent),
and sharp corners get zero-velocity stops, so lines track straight and corners stay
crisp. Loosen `corner_angle` / raise `corner_dwell` if corners round off; raise
`corner_angle` if curves get chopped.

---

## 7. Kinematics notes

- The `yaw_joint` carries an `rpy="0 0 π"` flip, so the arm's home reach direction is
  **−y** in the base frame. Targets with `y` greater than the base are *behind* the
  robot and unreachable within the ±90° yaw limit.
- Joint limits (URDF): `yaw ∈ [−90°, 90°]`, `bicep ∈ [−90°, 90°]`,
  `forearm ∈ [−90°, 90°]`. Maximum forward reach is about **−0.36 m** (the arm's
  physical length); points beyond it are correctly rejected.
- The pen tip is a fixed offset from the wrist *in the world frame* (the pen is always
  vertical), which the numerical FK/IK accounts for exactly.

---

## 8. Troubleshooting

- **`executable '...py' not found on the libexec directory`** — the package wasn't
  (re)built after the script was added. Run `colcon build --packages-select pr_controller`
  and re-source. New scripts must also be listed in `CMakeLists.txt` *and* be executable
  (`chmod +x`).
- **`colcon build` fails with `No module named 'ament_package'`** — you didn't source
  ROS first: `source /opt/ros/jazzy/setup.bash`.
- **Commands do nothing** — make sure the target is on the reachable **−y** side, and
  that the controllers finished spawning (watch the launch log for `arm_controller`).
- **Nothing moves on hardware** — check the servos are powered and `/dev/ttyUSB0` exists
  inside the container; the pen wrist is passive and won't move on its own.
- **GUI doesn't open** — verify `DISPLAY` is set and X11 is forwarded (see the compose
  file); on WSL ensure WSLg is available.

---

## 9. Vision pipeline: detect → connect → draw

An overhead camera detects objects on the table (YOLO) and the arm draws a
collision-free line connecting two of them. The camera is calibrated **once**
(`pr_calibration`: intrinsics via checkerboard + extrinsics via AprilTag → a fixed
`camera → base_link` transform); after that there is no runtime marker. Detection
runs continuously and connections are requested on demand via a service.

Packages: `pr_calibration` (calibration + static TF), `prarob_yolo` (detection),
`generate_trajectory` (the `connect_objects` service: pixel → `base_link` via the
static calibration, A* path, publishes to the arm's `~/draw_path`),
`pr_interfaces` (the `ConnectObjects` service type).

```bash
# --- one-time calibration (per camera mount) ---
ros2 launch pr_calibration intrinsic_calibration.launch.py   # checkerboard → camera_calibration_params.yaml
ros2 launch pr_calibration calibrate.launch.py               # AprilTag      → camera_extrinsics.yaml

# --- run the system (bring up once, leave running) ---
ros2 launch pr_controller hardware.launch.py                 # the arm (or sim.launch.py)
ros2 launch generate_trajectory connect_objects.launch.py    # camera + YOLO + static TF + connect service

# --- connect objects on demand, repeatably ---
ros2 service call /connect_objects pr_interfaces/srv/ConnectObjects \
    "{start: apple, goal: cup, obstacle: bottle}"
ros2 service call /connect_objects pr_interfaces/srv/ConnectObjects \
    "{start: pen, goal: book, obstacle: ''}"                  # empty obstacle = nothing to avoid
```

See what's currently detected (so you know which labels you can pass):
```bash
ros2 topic echo /detected_classes        # pr_interfaces/DetectedClasses: string[] of current class names
```
`/detected_objects` (visualization_msgs/MarkerArray) also shows each detection's class
label at its `base_link` table position — add a *MarkerArray* display in RViz.

Dial in against your real setup:
- **`table_z`** (param on `connect_objects_server`, default 0.02 m) = the object/draw
  plane height in `base_link`; set it to the actual pen drawing-contact height.
- **Service labels** must match your YOLO model's class names (COCO by default).
- Detected objects must lie in the arm's −y workspace; out-of-reach points are rejected.

---

## Quick reference

```bash
# build + source (every new shell)
source /opt/ros/jazzy/setup.bash && source /workspace/ws/install/setup.bash

# simulation
ros2 launch pr_controller sim.launch.py
# hardware
ros2 launch pr_controller hardware.launch.py

# draw text
ros2 run pr_controller draw_text_demo.py HELLO

# slow it down for precision
ros2 param set /inverse_kinematics_control draw_speed 0.03

# vision pipeline: detect objects + connect them with the arm
ros2 launch generate_trajectory connect_objects.launch.py
ros2 service call /connect_objects pr_interfaces/srv/ConnectObjects \
    "{start: apple, goal: cup, obstacle: bottle}"
```
