# Autonomous Vision-Language Semantic Navigation & Inspection Rover

**Owner:** Daniel Yeke — Nigerian mechatronics engineering graduate, building this as a
scholarship/internship portfolio project (AI + robotics).
**Platform:** HP ZBook 15u G4, 16GB RAM, 2GB VRAM (no usable GPU). Ubuntu 22.04 on WSL2
(a separate distro was installed after discovering the original WSL Ubuntu was 24.04).
ROS 2 Humble, Gazebo Classic.

**Working style:** the owner wants to deeply understand every concept, not just copy
commands — expects line-by-line code explanations, the underlying physics/maths, and
"teach-back" style lessons. Also wants to actually *see* the robot do autonomous things,
not just pass numeric checks — prioritise working end-to-end demos over polishing one
layer in isolation. Preference: when resuming a day's work, give ONE clean linear
walkthrough rather than a branching diagnosis trail.

---

## 1. Project architecture

```
Gazebo (gzserver) ──camera topics──► perception_node ──PointStamped (camera frame)──► target_locator ──PointStamped (map frame)──► mission_controller ──NavigateToPose──► Nav2 ──/cmd_vel──► Gazebo
                 └──scan/odom/tf───► Nav2 (AMCL + costmaps + planner + controller)
```

- **rover_sim** — Gazebo world + custom TurtleBot3 Waffle SDF (added a simulated depth
  camera; stock Waffle is RGB-only) + launch file + Nav2 params override.
- **rover_perception** — `perception_node`: subscribes to RGB + depth + camera_info,
  runs a detector (HSV colour threshold OR YOLOv8n), back-projects the detection's pixel
  centre + depth into a 3D point in the camera optical frame using the pinhole model,
  publishes `/perception/target_camera` (PointStamped) and `/perception/target_label`.
- **rover_localize** — `target_locator`: uses `tf2_ros` to transform the camera-frame
  point into the `map` frame (at the image's capture timestamp, not "now"), publishes
  `/perception/target_map` and a red sphere `Marker`.
- **rover_mission** — `mission_controller`: waits for 5 consistent detections (median
  position), computes a goal 1.0 m short of the target along the line from the robot,
  sends it to Nav2's `NavigateToPose` action, retries up to 2x on failure.

## 2. Key design decisions and why

- **Depth camera added via a copied+edited SDF** (not the stock Waffle model) — stock
  Waffle is RGB-only in sim. Resolution dropped to 640x480 @ 10Hz (from 1920x1080@30Hz)
  because the laptop's software-rendered GPU can't keep up; frame_name set to
  `camera_rgb_optical_frame` so depth images carry the correct optical-frame convention.
- **Detector is swappable via a `detector` ROS param** (`color` | `yolo`), sharing one
  output interface `(x,y,w,h,label,conf,mask_or_None)`, so the rest of the pipeline
  never changes when the detector changes. The colour detector (red-object HSV
  threshold) was deliberately built FIRST and used to validate the 2D→3D maths against
  a known ground-truth object (a red box at a measured position), independent of any ML
  model's reliability — this separated "is the geometry right" from "does the detector
  work."
- **Median depth over many pixels, not the single centre pixel** — a bounding box
  includes background pixels at the edges; a median is robust to that as long as >50%
  of the sampled pixels are the real object.
- **tf2 transform uses the image's timestamp, not "latest"** — YOLO inference takes
  1-2s+ on this CPU; the robot may have moved during that time, so the correct camera
  pose to use is the one at capture time, looked up from tf2's ~10s buffer.
- **Nav2 `inflation_radius` reduced from the 0.55m default to 0.3m** — the simulated
  world's pillars are spaced ~1.1m apart; with imperfect SLAM map quality (pillars
  rendered fatter than their true 0.3m diameter) the default inflation closed off the
  lanes between pillars entirely. Override lives in `rover_sim/params/nav2_params.yaml`
  (copied from the installed default, NOT edited in place) and is passed via
  `params_file:=...` on the Nav2 launch line.
- **Hydrant chosen as the test object** (not the original red box, not a person) —
  YOLO needs a COCO class; a person doesn't work well because the camera sits only
  ~12cm off the ground (mostly see legs); fire hydrant is short, textured, and a COCO
  class, so it detects reliably once placed ~2.4m away (too close and it's cropped out
  of frame and under-detected).

## 3. Known issues / fragile points (read before debugging blind)

- **Gazebo's model database download can hang** `spawn_entity` past its default 30s
  timeout on first run after a reboot — launch file sets `-timeout 120`. If a spawn
  still seems to hang, check for `Successfully spawned entity [waffle]` in the log
  before assuming anything downstream is broken — a stalled/incomplete spawn means
  Gazebo's diff-drive plugin never starts, which means `odom → base_footprint` is never
  published, which **splits the TF tree** (symptom: "Could not find a connection
  between 'odom' and 'base_link'... two unconnected trees"). Fix: kill everything
  (`pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -f component_container;
  pkill -9 -f spawn_entity`) and relaunch clean; don't try to patch around it.
- **RViz2 does not render the robot's mesh** (TF, LaserScan, Map, costmaps all render
  fine) — suspected Mesa/llvmpipe + Ogre Collada-mesh incompatibility under WSL2
  software rendering. Does not block any functional work. Not yet fixed. Gazebo's own
  window renders the robot fine, so use that for visual confirmation, or Foxglove
  Studio as a from-scratch alternative if ever needed for Day 7 demo recording.
- **The hydrant (and any manually-spawned object) does NOT persist** across a fresh
  `ros2 launch` — it was added at runtime via `spawn_entity.py`, not baked into the
  world file. The owner closes everything at the end of each work session, so **every
  session needs the hydrant respawned** unless the world file is permanently patched
  (see `rover_world.world`, which has a `<include><uri>model://fire_hydrant</uri>...`
  block added — confirm this is actually in use via the launch file before assuming
  it's automatic).
- **AMCL will not publish `map→odom`** until an initial pose is set. This produces
  "frame 'map' does not exist" warnings everywhere downstream (costmaps, target_locator)
  until `~/rover_ws/set_pose.sh` is run. This is normal/expected at every session start,
  not a bug — don't waste time diagnosing it, just run the script.
- **First SLAM map (`turtlebot3_map`) was poor quality**: pillars smeared up to 0.7m
  wide (true size 0.3m), likely because driving was done at normal speed with Gazebo
  GUI + RViz2 + SLAM + teleop all competing for CPU on a 2-core laptop, AND because the
  robot collided with obstacles and kept scanning while physically tilting/settling
  (LiDAR scans taken mid-collision get matched to the wrong pose). Re-mapped slowly and
  carefully (teleop slowed via `x`/`c` keys, full stop + wait after any contact) as
  `turtlebot3_map_v2` — noticeably cleaner, pillars ~0.5m, used from Day 5 onward.
  **turtlebot3_map (v1) should be considered deprecated; use turtlebot3_map_v2.**
- **YOLO inference time is highly variable under load**: ~1-2s per frame when little
  else is running, up to 6s when Gazebo GUI/RViz2 are also active. `imgsz:=320` (vs the
  default 640) cuts this roughly 4x per the pinhole/convolution-cost-scales-with-area
  argument, and was adopted as the default for all mission-node runs. This is a known,
  not-yet-fully-isolated confound for Day 6 benchmarking — vary ONE thing at a time
  (gui on/off, imgsz, detector) per benchmark run, don't change multiple variables and
  attribute the result to one of them.
- **Nav2 navigation to the hydrant stand-off goal has succeeded at least once** but
  took 19 recoveries and ~26s to cover ~1.5m — it works, but is not smooth. Worth a
  line in the Day 6/7 write-up as an honest limitation. Likely improvable with a better
  map and/or further inflation tuning, not yet attempted.
- **The mission_controller's 0.3s tf2 transform timeout is suspected non-functional**
  (single-threaded executor can't receive new TF data while blocked waiting inside its
  own callback) — works in practice because the needed transform is usually already in
  the buffer, but this is a known weak point worth mentioning as a limitation, not
  claiming as a tested, working timeout mechanism.

## 4. Status by day (against the original 7-day plan)

- **Day 1 (setup, bring-up):** DONE. ROS 2 Humble + Gazebo Classic installed in a
  dedicated Ubuntu-22.04 WSL2 distro. TurtleBot3 Waffle spawns, TF tree verified fully
  connected via `tf2_tools view_frames`.
- **Day 2 (mapping + Nav2):** DONE, map later redone (see v2 above). Nav2 bring-up,
  AMCL localisation, and at least one successful `NavigateToPose` goal all verified
  working.
- **Day 3 (perception):** DONE. Depth camera added to a custom SDF. Perception node
  built with swappable colour/YOLO detectors sharing one interface. Pinhole
  back-projection validated against a known red box (~1cm 3D error at 1.16m range) and
  cross-checked again on the hydrant via manual TF arithmetic (~5cm error, attributable
  to depth being measured on the object's front face, not its centre).
- **Day 4 (map-frame transform + marker):** DONE. `target_locator` built; transforms
  camera-frame detections into `map` via `tf2_ros`, publishes a red sphere Marker.
  Verified against the hydrant's known position (map output ~5cm off from true spawn
  position, consistent with the front-face depth offset).
- **Day 5 (mission controller, autonomous approach):** DONE. Full pipeline confirmed
  end-to-end with NO hardcoded goal anywhere: perception detected the hydrant, the
  locator placed it at map (0.54, -0.54), mission_controller computed a stand-off point
  1m away and sent it to Nav2, and the robot drove there cleanly (distance_remaining
  fell 1.42 -> 1.09 -> 0.86 -> 0.59 -> 0.36 -> 0.00m, zero recoveries this run) ending
  in "arrived: mission complete". This is the project's core autonomy claim, verified
  working. Getting here required: fixing a bad SLAM map (turtlebot3_map_v2), lowering
  Nav2's inflation_radius 0.55->0.3, and diagnosing a TF-tree split caused by a stalled
  Gazebo spawn (see §3) — all documented so this is reproducible, not a fluke.
- **Day 6 (benchmarking):** DONE. Full results table:

  | Metric | Result |
  |---|---|
  | Perception latency, 320px | median 66 ms, mean 96 ms, max 387 ms |
  | Perception latency, 640px | median 187 ms, mean 344 ms, max 4780 ms |
  | Perception latency, colour (HSV) | mean 6.2 ms, max 17 ms |
  | ONNX vs PyTorch fidelity | confidence within ~1-2% on identical inputs — faithful export |
  | YOLO-World domain gap | real photo (bus.jpg): 0.055 confident, correct. Simulated hydrant: 0.038-0.13, sometimes confused with "column" |
  | Localisation, AMCL vs odom (stationary) | ~3.0 cm offset, consistent across 5 samples |
  | Path planning time (wall-clock) | 7-20 ms across near/pillar-field/far goals. Nav2's own `planning_time` result field reported 0.0 for all three — unpopulated by this Nav2 build, not a real measurement; use the wall-clock figures |
  | Autonomous mission, full loop | succeeded, 0 recoveries, smooth descent 1.42→0.00 m |

  Key conclusion: **the planner was never the bottleneck** (7-20ms even for a
  248-pose path) — Day 5's earlier 19-recovery struggle was a local-controller /
  execution problem threading a tight, imperfectly-mapped gap, not a planning-speed
  problem. This distinction is worth stating explicitly in the write-up.

  **YOLO-World finding, in detail (for the write-up):** implemented as a third
  `detector` option in `perception_node.py` (`'color' | 'yolo' | 'yolo_world'`, same
  output interface, nothing downstream changed). Verified working correctly on a real
  photograph (bus.jpg → "fire hydrant" 0.055, correct and immediate). On the actual
  simulated hydrant, confidence stayed low (0.038-0.13) even at a near-zero threshold,
  and widening the vocabulary revealed real ambiguity between "fire hydrant" and
  "column" — both plausible labels for a cylindrical, flat-shaded synthetic object.
  This is a genuine, measured sim-to-real domain gap in CLIP's visual-text matching,
  not a configuration bug (ruled out: `set_classes()` wiring, ONNX export fidelity,
  confidence thresholding all checked and confirmed fine). YOLOv8n (closed-set) remains
  the detector actually driving the autonomous demo. Two credible, untried fixes
  documented as future work (see §8): (1) apply a photographic texture to the hydrant's
  Gazebo material to narrow the visual domain gap, (2) a two-stage pipeline — use the
  already-working YOLOv8n as a region proposer, then classify only the cropped region
  with CLIP, rather than asking YOLO-World to propose and name in one pass on a full
  synthetic scene (this mirrors real open-vocabulary detection research, e.g.
  RegionCLIP/Detic-style architectures).
- **Day 7 (docs, demo, CI/CD):** NOT STARTED. A git repo was initialised in `~/rover_ws`
  with a `.gitignore` (`build/ install/ log/ __pycache__/ *.pyc`) and one commit made
  after Day 3. Needs updating with everything since. Note for the README: the
  `fire_hydrant` model lives in `~/.gazebo/models/`, OUTSIDE the repo — installation
  instructions need a step to fetch it (see the `git sparse-checkout` command used
  originally, in chat history, to pull just that one model folder from
  `osrf/gazebo_models`).
- **Vision-language / open-vocabulary detector:** NOT STARTED, but the owner considers
  this ESSENTIAL (not optional) given the project's name and scholarship framing. Plan:
  try **YOLO-World** first (ships in `ultralytics`, open-vocabulary via text prompts,
  same output format as YOLOv8 so it drops into the existing `detect_yolo`-style
  interface with minimal change). MobileCLIP alone was ruled out (can't localise,
  only classifies crops/whole images). NanoOWL was ruled out (NVIDIA/TensorRT-only,
  this laptop has no NVIDIA GPU). OWL-ViT is a fallback if YOLO-World is too slow —
  expected to be much heavier, unverified. An OpenVINO export of whichever model is
  used would be a good Day 6 benchmark story (baseline vs optimised).

## 5. Teaching/lessons already covered (don't re-teach from scratch)

The owner has been taught, in a "teach back" style with questions and corrections,
through:
1. Pinhole camera model, intrinsics, the `X=(u-cx)Z/fx` derivation, optical-frame vs
   ROS body-frame axis conventions.
2. TF2: transform composition, the `map→odom→base_link` split and why, tf2 exceptions
   (`LookupException`, `ExtrapolationException`, `ConnectivityException`), the tf
   buffer's ~10s history.
3. ROS 2 fundamentals: nodes/topics/messages, services vs actions, QoS (reliable vs
   best-effort, why sensor data uses best-effort), single-threaded executors and
   callback queuing/dropping under load, use_sim_time vs wall-clock, message_filters'
   ApproximateTimeSynchronizer.
4. `perception_node.py` line-by-line (HSV colour thresholding mechanics, YOLO's
   single-pass + NMS, median-depth-over-mask rationale).
5. `target_locator.py` line-by-line (tf2_ros.Buffer/TransformListener, marker fields,
   the fixed-id-and-lifetime pattern for an updating marker).
6. The simulation stack: URDF vs SDF purposes, the `<plugin>` bridge from Gazebo's
   internal sim to ROS topics, launch file anatomy, colcon/package.xml/setup.py roles,
   `--symlink-install`.
7. Detector internals in more depth: HSV's hue-stability-under-lighting rationale and
   its failure modes; YOLO's single-pass design, grid-cell/NMS mechanics, why "nano"
   is still slow on a 2-core CPU, why conf is a combined objectness×class estimate
   (not a bare class probability).

**NOT yet covered** (owner explicitly deferred to "after Day 4" then it got pushed
further by debugging time): **Lesson 7 — SLAM's pose-graph/loop-closure mechanics,
AMCL's particle filter steps in detail, and costmap layer semantics in depth.** Some of
this came up ad hoc while debugging the Day 5 map/costmap issues, but the planned
structured lesson has not been delivered. `mission_controller.py` has also not yet
received its own line-by-line walkthrough (promised, not yet done).

## 6. Daily runbook (reproduce from a cold start)

See `~/rover_ws/RUNBOOK.md` on the machine for the terminal-by-terminal version. Summary:

```bash
# T1 — simulation (headless by default; this laptop struggles with gui:=true + everything else)
ros2 launch rover_sim rover_world.launch.py gui:=false
# WAIT for the literal line: Successfully spawned entity [waffle]
# If the hydrant isn't baked into the world file yet, respawn it manually:
ros2 run gazebo_ros spawn_entity.py -entity hydrant \
  -file ~/.gazebo/models/fire_hydrant/model.sdf -x 0.6 -y -0.5 -z 0

# T2 — Nav2, with the inflation override, map v2, tick-rate spam filtered
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true \
  map:=$HOME/turtlebot3_map_v2.yaml \
  params_file:=$HOME/rover_ws/install/rover_sim/share/rover_sim/params/nav2_params.yaml \
  2>&1 | grep --line-buffered -v "tick rate"
# As soon as AMCL warns "Please set the initial pose", in a new terminal:
~/rover_ws/set_pose.sh
# Wait for "Managed nodes are active" (may print twice — localisation, then navigation)

# T3 — perception (always source first; this has silently failed before)
source ~/rover_ws/install/setup.bash
ros2 run rover_perception perception_node --ros-args -p use_sim_time:=true \
  -p detector:=yolo -p "classes:=fire hydrant" -p imgsz:=320
# Allow up to 2 minutes for the first detection on this laptop — not a hang.

# T4 — locator
source ~/rover_ws/install/setup.bash
ros2 run rover_localize target_locator --ros-args -p use_sim_time:=true

# T5 — mission
source ~/rover_ws/install/setup.bash
ros2 run rover_mission mission_controller --ros-args -p use_sim_time:=true
# Expect, in order: "target confirmed at map (...)", "driving to (...)",
# falling "distance remaining", then "arrived: mission complete"
```

**Useful diagnostic one-liners** (all zero-GUI, text-only — prefer these over opening
RViz2/Gazebo windows when just checking pipeline health, since the GUIs are a
significant CPU/freeze risk on this machine):

```bash
ros2 run tf2_ros tf2_echo map odom                       # confirms AMCL is localised
ros2 topic echo /perception/target_map --once            # confirms the full chain works
ros2 run tf2_tools view_frames                           # confirms TF tree is whole (no split)
python3 ~/rover_ws/cost_query.py                          # numeric costmap cost at goal/robot/hydrant
python3 ~/rover_ws/cost_dump.py                           # ASCII-art costmap (visual, no GUI)
python3 ~/rover_ws/map_dump.py ~/turtlebot3_map_v2.yaml   # ASCII-art of the raw saved map
```

## 7. Immediate next step for whoever picks this up

Days 1-6 are all DONE. **Day 7 — docs, demo recording, repo cleanup — is next and is
the only remaining item for the core 7-day plan.**

Demo recording plan: run with `gui:=true` (the pipeline is now proven stable enough to
risk the extra render load for this one recording), and capture Gazebo's window
alongside `rqt_image_view` on `/perception/debug_image` simultaneously if possible, so
a viewer sees the detection box + confidence score at the same moment the robot starts
driving toward the target — this is the single most convincing shot the project can
produce, don't settle for the Gazebo window alone.

Day 7 checklist:
1. Record the full autonomous run (detect → confirm → drive → arrive) with GUI on.
2. Clean up the GitHub repo — it currently has `build/`/`install/` artefacts committed
   (per Grok's note) instead of just `src/`; add/fix `.gitignore`, commit only source,
   params, and world files.
3. Write the README: architecture diagram (§1 of this doc is a good starting point),
   install instructions (include fetching `fire_hydrant` into `~/.gazebo/models/` —
   NOT in the repo currently), the runbook (§6), and the Day 6 results table (§4).
4. Include PROJECT_STATUS.md itself (this file) or fold its content into the README —
   it's a more honest and complete account of the engineering process than a polished
   README alone, and reviewers who dig in will find a well-documented debugging trail
   rather than a suspiciously clean history.
5. Optional stretch (post-Day-7, not blocking): the two-stage YOLOv8n-propose +
   CLIP-classify pipeline described in §4's YOLO-World section, or a textured-material
   experiment to narrow the sim-to-real domain gap.

Still owed from the teaching track, to slot in opportunistically, not blocking Day 7:
`mission_controller.py` line-by-line walkthrough, and Lesson 7 (SLAM pose-graph/
loop-closure, AMCL particle filter steps, costmap layer semantics, in the same depth
as the earlier lessons).
