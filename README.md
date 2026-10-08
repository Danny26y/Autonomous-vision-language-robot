# Autonomous Vision-Language Semantic Navigation & Inspection Rover

A ROS 2 + Gazebo mobile robot that **sees** an object through a learned vision model,
**locates** it in the world using TF2, and **autonomously drives** to it through Nav2 —
with no hardcoded coordinates anywhere in the loop.

**Author:** Daniel Yeke — mechatronics engineer (Abuja, Nigeria)
[GitHub](https://github.com/Danny26y) · [LinkedIn](https://linkedin.com/in/daniel-yeke-842b35344)

**[▶ Watch the demo video](./demo_autonomous_approach.mp4)** — robot detects a fire
hydrant, confirms the target, and drives to a 1 m stand-off point, fully autonomously.

---

## What it does

```
Gazebo (gzserver)
  ├── camera topics ──► perception_node ── 3D point (camera frame)
  │                                              │
  │                                              ▼
  │                                       target_locator ── 3D point (map frame) + marker
  │                                              │
  │                                              ▼
  │                                       mission_controller ── NavigateToPose goal
  │                                              │
  └── scan / odom / tf ──► Nav2 (AMCL + costmaps + planner + controller)
                                                   │
                                                   ▼
                                                /cmd_vel ──► Gazebo
```

1. A simulated TurtleBot3 Waffle (fitted with an added depth camera) explores a Gazebo
   world containing a fire hydrant among several pillars.
2. **`perception_node`** runs a YOLOv8n object detector on the live camera feed and
   back-projects the 2D detection into a 3D point in the camera's optical frame, using
   the pinhole camera model and the depth image.
3. **`target_locator`** transforms that point into the map frame via `tf2_ros`, at the
   image's own capture timestamp (not "now"), and publishes a target marker.
4. **`mission_controller`** waits for five consistent detections, takes their median
   position, computes a goal 1 m short of the target, and sends it to Nav2's
   `NavigateToPose` action — the robot plans and drives there on its own.

## Proof it works

Real terminal output from the autonomous run that produced the demo video.

**Perception** — the node's output as it detects the hydrant: label, confidence,
pixel position, the back-projected 3D position in the camera frame, and per-frame
latency.

![Perception log](docs/perception_log.png)

**Mission** — the target confirmed from five detections, the stand-off goal sent to
Nav2, `distance remaining` falling towards zero, and `arrived: mission complete`.
No coordinate was typed in by hand.

![Mission log](docs/mission_log.png)

## Why "vision-language"

Alongside the closed-set YOLOv8n detector that drives the live demo, the project also
implements and benchmarks **YOLO-World**, an open-vocabulary (text-prompted) detector,
as a third interchangeable `detector` option in `perception_node`. It works correctly
on real photographs but shows a measured, documented **sim-to-real domain gap** on the
simulated hydrant (see [Results](#results) below) — a genuine finding about applying
CLIP-based vision-language models to synthetic/simulated environments, not a dead end.
Two concrete next steps for closing that gap are documented in
[`PROJECT_STATUS.md`](./PROJECT_STATUS.md).

## Hardware this was built on

HP ZBook 15u G4 — 16 GB RAM, no usable GPU (2 GB low-power iGPU/dGPU, software
rendering throughout). Every design decision in this repo — resolution choices,
`imgsz:=320`, the Nav2 inflation tuning, the ONNX export — exists because of this
constraint, and is documented as such rather than hidden.

---

## Packages

| Package            | Role |
|---------------------|------|
| `rover_sim`         | Gazebo world, a custom TurtleBot3 Waffle SDF (stock Waffle is RGB-only — a simulated depth camera was added), launch files, and a tuned Nav2 params override |
| `rover_perception`  | `perception_node` — RGB + depth → detector (colour / YOLOv8n / YOLO-World) → 3D point, pinhole back-projection |
| `rover_localize`    | `target_locator` — camera-frame point → map-frame point via TF2, + RViz marker |
| `rover_mission`     | `mission_controller` — median-of-detections target confirmation → stand-off goal → Nav2 `NavigateToPose`, with retries |

---

## Quick start

```bash
# 1. Clone and build
git clone https://github.com/Danny26y/Autonomous-vision-language-robot-.git ~/rover_ws
cd ~/rover_ws
colcon build --symlink-install
source install/setup.bash

# 2. Fetch the hydrant model (not bundled — see "Assets not in this repo" below)
git clone --depth 1 --filter=blob:none --sparse https://github.com/osrf/gazebo_models.git /tmp/gm
(cd /tmp/gm && git sparse-checkout set fire_hydrant)
mkdir -p ~/.gazebo/models && cp -r /tmp/gm/fire_hydrant ~/.gazebo/models/
rm -rf /tmp/gm

# 3. Run it — five terminals, in order:

# T1 — simulation
ros2 launch rover_sim rover_world.launch.py gui:=true
# wait for: Successfully spawned entity [waffle]

# T2 — Nav2, with the tuned inflation params
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true \
  map:=$HOME/turtlebot3_map_v2.yaml \
  params_file:=$HOME/rover_ws/install/rover_sim/share/rover_sim/params/nav2_params.yaml
# as soon as AMCL warns "please set the initial pose":
~/rover_ws/set_pose.sh
# wait for: Managed nodes are active

# T3 — perception
ros2 run rover_perception perception_node --ros-args -p use_sim_time:=true \
  -p detector:=yolo -p "classes:=fire hydrant" -p imgsz:=320

# T4 — locator
ros2 run rover_localize target_locator --ros-args -p use_sim_time:=true

# T5 — mission
ros2 run rover_mission mission_controller --ros-args -p use_sim_time:=true
```

Expect, in the mission terminal: `target confirmed at map (...)` → `driving to (...)`
→ a falling `distance remaining` → `arrived: mission complete`.

### Assets not in this repo

- `~/.gazebo/models/fire_hydrant/` — fetched above from `osrf/gazebo_models`.
- `yolov8n.pt`, `yolov8s-world.pt` — auto-downloaded by `ultralytics` on first use.
- `yolov8n.onnx` — export it yourself:
  ```bash
  python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx', imgsz=320)"
  ```
  then point perception at it with `-p model:=/path/to/yolov8n.onnx`.
- `~/turtlebot3_map_v2.{yaml,pgm}` — the saved SLAM map. Rebuild it by running
  `slam_toolbox` and driving the robot around the world (see `PROJECT_STATUS.md` §3 for
  the lessons learned doing this — drive slowly, stop fully on any collision).

---

## Results

### Perception latency (YOLOv8n, CPU-only)

| Config | Mean | Median | Max |
|---|---|---|---|
| 320 px | 96 ms | **66 ms** | 387 ms |
| 640 px | 344 ms | **187 ms** | 4780 ms |
| HSV colour baseline | 6.2 ms | — | 17 ms |

Mean is skewed upward by occasional CPU-contention spikes under this hardware's
dual-core load; median is the more representative "typical frame" figure.

### ONNX export fidelity

PyTorch and ONNX-exported YOLOv8n give near-identical confidence scores on the same
input (within ~1–2%) — the export is faithful, not a source of accuracy loss. ONNX's
lighter CPU execution path was what ultimately made `gui:=true` + screen recording +
the full pipeline run reliably together for the demo video.

### YOLO-World (open-vocabulary) domain gap

| Image | Top result |
|---|---|
| Real photo (`bus.jpg`) | "fire hydrant", conf **0.055**, correct, immediate |
| Simulated hydrant, narrow vocabulary | "fire hydrant", conf **0.038** |
| Simulated hydrant, broad vocabulary | "column" 0.106, "fire hydrant" 0.038 — genuinely ambiguous |

Ruled out as causes: `set_classes()` wiring, ONNX export correctness, confidence
threshold. The model *does* register visual signal on the object (it isn't blind to
it) but the text-image match is weak and ambiguous for this rendering style — a
measured limitation of applying a real-photo-trained CLIP backbone to flat-shaded
synthetic geometry, not a bug. See `PROJECT_STATUS.md` for two credible, untried fixes
(a textured Gazebo material; a two-stage YOLOv8n-propose + CLIP-classify pipeline).

### Localisation

AMCL and raw odometry agreed to within **~3.0 cm** while the robot was stationary,
consistent across 5 samples.

### Path planning

7–20 ms wall-clock across three goals of increasing difficulty (including a 248-pose
path). **The planner was never the bottleneck** — an earlier failed run that took 19
recovery attempts and ~26 s to cover 1.5 m was a local-controller / execution problem
(navigating a tight, imperfectly-mapped gap between pillars), not a planning-speed
problem. Fixed by re-mapping more carefully and reducing Nav2's `inflation_radius`
from the 0.55 m default to 0.3 m to match this environment's pillar spacing.

### End-to-end mission

Succeeded with **zero recovery attempts**, a smooth `distance_remaining` descent
(1.42 → 0.00 m), ending in `arrived: mission complete` — with no hardcoded goal
anywhere in the pipeline.

---

## Engineering notes worth reading before you dig into the code

The full debugging history — a stalled Gazebo spawn silently splitting the TF tree,
a SLAM map corrupted by driving too fast during mapping, Nav2 inflation tuning, the
reasoning behind every parameter choice — is documented in
[`PROJECT_STATUS.md`](./PROJECT_STATUS.md). It's kept intentionally unpolished: a
transparent account of the actual engineering process, not a rewritten success story.

## Limitations

- RViz2 does not render the robot's 3D mesh under this WSL2/software-rendering setup
  (TF, LaserScan, costmaps, and markers all render correctly — this is cosmetic only).
  Gazebo's own window renders the robot fine.
- The mission controller's TF lookup timeout is believed non-functional under a
  single-threaded executor (works in practice because the needed transform is usually
  already buffered) — a known weak point, not a tested guarantee.
- Spawned objects (e.g. the hydrant) don't persist across simulation restarts unless
  baked into the world file.
- See `PROJECT_STATUS.md` for the complete, current list.

## License

Apache-2.0 (matches the ROS 2 package manifests).
