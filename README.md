# Multi-AMR Dolly Transport — Digital Twin

Three autonomous mobile robots collect wheeled dollies from parking bays in a
factory and deliver them to drop nodes, in Isaac Sim, driven over ROS 2. The
task-to-robot assignment is solved by NVIDIA cuOpt; the final docking approach
is guided by camera recognition of the dolly deck.

---

## Problem

Moving material dollies between stations is a job factories give to fleets of
AMRs, and it fails in ways that are expensive to discover on real hardware:
two robots claim the same aisle, a robot docks against a dolly that has drifted
from its recorded position, one robot ends up doing most of the work while the
others idle. Each of those is a fleet-level problem — it cannot be reproduced by
testing a single robot, and it cannot be reproduced reliably on a factory floor
because the failure depends on timing.

A digital twin is the practical place to test it. The scene is the real factory
layout, the robots are the real chassis, and the failure modes above can be
provoked on demand and measured. What this repository sets out to verify:

| Question | Measured by |
|---|---|
| Do all transports complete? | Success count over the task set |
| How accurate is docking? | Position and yaw error at dock, per mission |
| Does the fleet share work? | Tasks per robot, distance per robot, makespan |
| Do robots avoid each other? | Traffic-edge lock acquisitions and waits |
| Does vision recognise the dolly? | Detection rate per snapshot, bearing error |
| Does it survive vision failure? | Docking accuracy when recognition is refused |

### Known limits

Stated up front, because they bound what the numbers mean.

- **Dolly positions are fixed per run.** The bridge places them from the scene,
  so the same run repeats identically. Random placement is not implemented.
- **One robot uses vision.** The bridge builds a single docking camera, so only
  one robot's approach is camera-guided; the other two dock on coordinates.
- **The vision correction is clamped.** Measured bearing error has a standard
  deviation of 2.6 degrees, which is 0.14 m of lateral error at 3 m — the same
  order as the coordinate dock's own 0.11 m. An unclamped correction was
  observed making docking worse, so it is limited to 0.05 m. Vision's reliable
  contribution is confirming the dolly is there, not out-measuring odometry.
- **Single machine.** Multi-PC ROS 2 distribution is not set up here.
- **Two USD assets carry a dead absolute reference.**
  `Collected_AF2_FLAT/AF2_FLAT.usd` and `AF2_MULTI_BACKUP.usd` both reference
  `/home/skywalker/.../simulation/isaac_sim/Collected/AF2.usd`. That file does
  not exist on the development machine either, and is gitignored, so the
  reference already fails to resolve and the scene loads and runs regardless -
  a fresh clone is in exactly the same position. It is recorded here because
  it is a real absolute path inside a tracked file, and fixing it means
  rewriting a 126 MB binary USD crate, which is deliberately not done casually:
  the handover notes require that the original USD is never written and that
  all corrections happen at runtime.

---

## Requirements

| Component | Version / path | Notes |
|---|---|---|
| Isaac Sim | 5.x at `~/isaacsim` | Provides its own Python 3.11 |
| ROS 2 | Jazzy at `/opt/ros/jazzy` | |
| Python (ROS side) | system `python3` 3.12 | |
| Vision env | `.venv_vision` in this repo | `opencv-python`, `numpy`, `ultralytics`, `cv_bridge` |
| cuOpt env | `~/.venvs/cuopt` | Python 3.12 + `cudf`, see below |
| GPU | NVIDIA, CUDA 12+ | RTX 5080 Laptop used for development |

`.venv_vision` is a symlink on the development machine. To build it fresh:

```bash
python3 -m venv .venv_vision
./.venv_vision/bin/pip install opencv-python numpy ultralytics
# cv_bridge and rclpy come from the ROS 2 install; run with rosenv.sh sourced
```

cuOpt cannot share an interpreter with Isaac Sim — it needs `cudf`, and Isaac
brings its own CUDA stack. It gets its own environment:

```bash
python3 -m venv ~/.venvs/cuopt
~/.venvs/cuopt/bin/pip install --extra-index-url https://pypi.nvidia.com \
    cuopt-cu12==26.8.0
```

### Environment variables

All are optional; defaults are in `simulation/isaac_sim/standalone_factory_bridge.py`.

| Variable | Default | Meaning |
|---|---|---|
| `FLEET` | `amr1,amr2` | Which robots to spawn |
| `TASK_IDS` | `T1..T7` | Which transports to run |
| `PLAN_SOLVER` | `auto` | `auto`, `manual`, `greedy`, `exact`, `cuopt` |
| `HEADLESS` | `0` | `1` renders without a window |
| `CAMERA_WIDTH` / `CAMERA_HEIGHT` | `1280` / `720` | Docking camera resolution |
| `SHOW_WAYPOINT_GRAPH` | `0` | `1` restores the waypoint markers |
| `ROS_DOMAIN_ID` | `130` | Set by `scripts/rosenv.sh` |

`scripts/rosenv.sh` sets the ROS domain, RMW and Fast DDS profile for every
process. Source it rather than exporting by hand: a bridge once ended up on a
different domain from the controllers and the two could not see each other.

---

## Running

Every command is run from the repository root. Nothing depends on where the
repository lives.

### First time in a fresh clone

```bash
# Build both packages. `--packages-select amr_control` on its own fails:
# amr_control depends on interfaces, and colcon will not build it implicitly.
cd ros2_ws && colcon build --symlink-install && cd ..
```

```bash
# 0. Clear any leftover processes. Two bridges publishing /odom will make the
#    robots chase conflicting pose streams.
bash scripts/cleanup.sh

# 1. Solve the fleet assignment with cuOpt (writes cuopt_plan.json).
#    Needs factory_inventory.json, so run the bridge once first if this is a
#    fresh clone, or skip this step and use PLAN_SOLVER=auto.
~/.venvs/cuopt/bin/python scripts/plan_cuopt.py

# 2. Start the simulation bridge. Wait for "BRIDGE RUNNING" (about 40 s).
HEADLESS=0 CAMERA_WIDTH=640 CAMERA_HEIGHT=360 \
  FLEET=amr1,amr2,amr3 TASK_IDS=T1,T2,T3,T4,T5,T6,T7 PLAN_SOLVER=cuopt \
  bash scripts/run_bridge.sh

# 3. Vision node, plan panel, and viewers (each in its own terminal)
bash scripts/run_vision_node.sh
bash scripts/run_planner_panel.sh
bash scripts/run_rqt.sh /vision/dolly_docking/debug_image
bash scripts/run_rqt.sh /planner/comparison

# 4. Controllers, one per robot. Only amr1 uses the camera.
bash scripts/run_controller.sh amr1 --vision
bash scripts/run_controller.sh amr2
bash scripts/run_controller.sh amr3
```

### Judging success from the logs

Topic connectivity is not evidence. When vision fails, the fallback still
publishes docking commands, and a run in which nothing was ever recognised can
still look connected. Check these lines instead:

```
SNAPSHOT DOLLY seen at -1.18 deg (expected -0.74 deg, 5/5 frames)   recognised
SNAPSHOT hold done, resuming approach                               paused to look
DOCK OK | position_error=0.108 m                                    docked
```

`DOLLY CONFIRMED on n/5 frames` means the dolly was recognised but its bearing
was not precise enough to steer on; the approach continues on coordinates. That
is a designed outcome, not a failure.

---

## Layout

```
scripts/
  rosenv.sh              shared ROS 2 environment - source everywhere
  cleanup.sh             kill leftover bridges, controllers, viewers
  run_bridge.sh          Isaac Sim + ROS 2 bridge
  run_vision_node.sh     snapshot recognition node
  run_controller.sh      one AMR mission controller
  run_planner_panel.sh   fleet plan comparison image
  run_rqt.sh             image topic viewer
  plan_cuopt.py          solve the assignment with cuOpt, write JSON
  planner_panel.py       draw the plan comparison as bars
  planner_map.py         draw the plan on the factory, and
                         manual against cuOpt side by side
  measure_dolly.py       measure the dolly geometry from the USD
  classify_tasks.py      report which tasks pass corners / cross the factory
  design_tasks.py        search for a balanced task set
  scale_intrinsics.py    write camera intrinsics for a resolution / field of view
  capture_frames.py      record frames with the pose sampled at the same instant
  eval_detection.py      detection rate over recorded frames
  project_dolly.py       draw where the dolly should be, from known geometry

simulation/isaac_sim/
  standalone_factory_bridge.py   scene setup, ROS 2 bridge, fleet plan
  mission_planner.py             graph, solvers, plan scoring
  vision_docking/
    config/vision_config.py      keypoints, image size, SDG ranges
    config/camera_intrinsics*.npz  calibrated camera model
    runtime/blue_deck_detector.py  dolly recognition
    runtime/dolly_snapshot_node.py snapshot protocol + rqt overlay
    sdg/                          synthetic dataset generation
    training/train_yolo_pose.py   YOLO pose training

ros2_ws/src/amr_control/         mission controller (FSM), fleet traffic locking
docs/                            work logs and handover notes
```

---

## Design

### Control

Diagrams are in `docs/ARCHITECTURE.md`; the summary follows.

Each robot runs an independent finite state machine. There is no central
executive issuing motion commands; the fleet coordinates through a shared
traffic-claim topic, so losing one controller does not stop the others.

```
WAIT_ODOM -> APPROACH -> [VISION_STANDOFF -> SNAPSHOT_ALIGN
                          -> WAIT_SNAPSHOT -> SNAPSHOT_HOLD]
          -> GO_TO_PRE_DOCK -> FINAL_DOCK -> LIFT_UP -> ATTACH_DOLLY
          -> CARRY -> LIFT_DOWN -> DETACH_DOLLY -> UNDOCK -> next mission
```

The bracketed states run only for the robot and mission that vision is enabled
for. They are entered *during* the approach, at the moment the range to the dock
first falls below the standoff distance, so the robot reaches the observation
point going forwards. Aiming at it after arriving at the pickup node made the
robot reverse away from the dolly and drive back in.

### Vision

Recognition is a single measurement taken from a standstill, not a control loop.
An earlier version steered from the camera at frame rate; because its open-loop
fallback published on the same topic, a run in which nothing was recognised still
logged that vision was engaged. A snapshot either produces a number or it does
not, and the controller applies it as a bounded correction to a target it
already had.

The detector separates two questions with very different reliabilities:

- **Is a dolly there?** Answered on 75-100% of frames between 1 and 8 m. Gates
  the approach.
- **Where exactly?** Answered far less often. Only ever trims the target, and
  only within `snapshot_max_correction_m`.

### Planning

`mission_planner.py` holds the graph and three solvers — greedy, exhaustive and
local search — all minimising makespan. cuOpt is solved out of process by
`scripts/plan_cuopt.py` and read back from JSON.

cuOpt is not used because it is faster or finds better routes; for seven tasks
the exhaustive solver is optimal in under 2 ms. It is used because it can
express constraints the others cannot. Asked to minimise distance with no return
trip, cuOpt initially gave every task to one robot — correct, and useless as a
fleet. Requiring all vehicles to be used and capping each route's cost turns it
into the 3/2/2 split the demonstration needs, and neither constraint is
expressible in a makespan-only solver.

Every candidate plan is re-scored through the same cost model before comparison,
so the panel is not comparing one solver's arithmetic against another's.

---

## Documentation

- `docs/WORKLOG_2026-08-27.md` — development log: what was measured, what was
  tried and rejected, and why. Includes two conclusions that turned out to be
  wrong and how they were caught.
- `docs/ARCHITECTURE.md` — process and interface map, the mission controller
  state machine, the recognition-to-wheels sequence, the planning data flow,
  and the technology stack. Diagrams are Mermaid, so they render on GitHub.
- `docs/HANDOFF_vision_docking.md` — environment notes and known traps.
