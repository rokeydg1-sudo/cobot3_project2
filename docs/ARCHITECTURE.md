# Architecture

Diagrams of what runs where, how the robots are controlled, and how a
recognition result reaches the wheels. Written in Mermaid so they render on
GitHub and stay next to the code that has to match them.

---

## 1. Processes and interfaces

Four Python interpreters are involved, and they cannot be merged. Isaac Sim
ships its own 3.11; ROS 2 Jazzy uses the system 3.12; the vision node needs
OpenCV and `cv_bridge` together; cuOpt needs `cudf`, which will not sit beside
Isaac's CUDA stack. Everything therefore talks over ROS 2 topics or files.

```mermaid
flowchart LR
    subgraph isaac["Isaac Sim  (~/isaacsim/python.sh, 3.11)"]
        bridge["standalone_factory_bridge.py<br/>scene, physics, OmniGraph ROS bridge"]
    end

    subgraph ros["ROS 2 Jazzy  (system python3, 3.12)"]
        c1["amr_mission_controller<br/>amr1"]
        c2["amr_mission_controller<br/>amr2"]
        c3["amr_mission_controller<br/>amr3"]
    end

    subgraph venv[".venv_vision  (OpenCV + cv_bridge)"]
        snap["dolly_snapshot_node.py"]
        panel["planner_panel.py"]
        map["planner_map.py"]
    end

    subgraph cuopt["~/.venvs/cuopt  (cudf, run before the sim)"]
        solver["plan_cuopt.py"]
    end

    solver -- "cuopt_plan.json" --> bridge
    bridge -- "factory_inventory.json" --> c1 & c2 & c3
    bridge -- "factory_inventory.json" --> panel & map

    bridge <-- "/odom  /cmd_vel<br/>/lift_cmd  /lift_joint_state<br/>/dolly_cmd" --> c1 & c2 & c3
    bridge -- "/vision/front_camera/image_raw" --> snap
    c1 <-- "/dock/snapshot_request<br/>/dock/snapshot_result" --> snap

    c1 <-- "/traffic/claims" --> c2
    c2 <-- "/traffic/claims" --> c3

    snap -- "/vision/dolly_docking/debug_image" --> rqt1["rqt_image_view"]
    panel -- "/planner/comparison" --> rqt2["rqt_image_view"]
    map -- "/planner/map<br/>/planner/map_compare" --> rqt3["rqt_image_view"]
```

Two things worth noting because they are easy to misread.

**The bridge is not an rclpy node.** Its ROS interface is an OmniGraph built
inside Isaac Sim, which is why `ros2 topic list` can come back empty while
topics are publishing normally. Use `ros2 topic hz /odom` to check it.

**cuOpt runs before the simulation, not during it.** The assignment is solved
into a file and read at startup. That is a consequence of the interpreter
split, not a design preference.

---

## 2. Mission controller state machine

One instance per robot, each independent. There is no central executive issuing
motion commands: the fleet coordinates only through `/traffic/claims`, so
losing one controller stops one robot rather than the fleet.

```mermaid
stateDiagram-v2
    [*] --> WAIT_ODOM
    WAIT_ODOM --> APPROACH: odometry received<br/>and start delay elapsed

    APPROACH --> VISION_STANDOFF: range to dock<br/>&lt; standoff distance
    note right of VISION_STANDOFF
        Only for the robot and mission
        vision is enabled for. Entered
        during the approach, so the
        standoff is reached going
        forwards rather than by
        reversing back to it.
    end note

    VISION_STANDOFF --> SNAPSHOT_ALIGN: arrived, stopped
    SNAPSHOT_ALIGN --> WAIT_SNAPSHOT: heading within<br/>standoff_yaw_tolerance
    WAIT_SNAPSHOT --> SNAPSHOT_HOLD: result received<br/>or timeout
    SNAPSHOT_HOLD --> APPROACH: dwell elapsed<br/>(resume the route)

    APPROACH --> GO_TO_PRE_DOCK: last waypoint reached
    GO_TO_PRE_DOCK --> FINAL_DOCK: pre-dock reached
    FINAL_DOCK --> LIFT_UP: within dock tolerance
    LIFT_UP --> ATTACH_DOLLY: lift raised
    ATTACH_DOLLY --> CARRY: attach acknowledged
    CARRY --> LIFT_DOWN: goal node reached
    LIFT_DOWN --> DETACH_DOLLY: lift lowered
    DETACH_DOLLY --> UNDOCK: release acknowledged
    UNDOCK --> APPROACH: next mission
    UNDOCK --> DONE: no missions left

    APPROACH --> TIMEOUT: waypoint timeout
    GO_TO_PRE_DOCK --> TIMEOUT: waypoint timeout
    FINAL_DOCK --> TIMEOUT: waypoint timeout
    TIMEOUT --> [*]
    DONE --> [*]
```

The bracketed vision states are skipped entirely when vision is off, and the
remaining path is the coordinate-only sequence that has always worked. That is
deliberate: vision is layered on top of a working dock, never in place of it.

---

## 3. How a recognition reaches the wheels

The controller stops, asks once, and applies a bounded correction. It is not a
control loop.

```mermaid
sequenceDiagram
    participant C as mission controller
    participant S as snapshot node
    participant B as Isaac bridge

    B-->>S: /vision/front_camera/image_raw (continuous)
    S-->>S: detect + draw overlay
    S-->>C: /vision/dolly_docking/debug_image (for rqt, always)

    Note over C: range to dock < standoff<br/>stop, turn to the dock heading

    C->>S: /dock/snapshot_request {seq, range_m, deck_width_m}
    Note over S: 5 frames while stationary<br/>expected deck width from<br/>range and measured geometry
    S->>C: /dock/snapshot_result {seq, present, ok, bearing_deg}

    alt bearing usable
        C->>C: lateral = range * tan(measured - expected)
        C->>C: clamp to snapshot_max_correction_m
        C->>C: shift the dock target
    else recognised but bearing refused
        C->>C: proceed, no correction
    else nothing recognised
        C->>C: proceed, no correction
    end

    Note over C: dwell, then resume the approach
    C->>B: /cmd_vel
```

### Why the answer is split in two

`present` and `ok` are reported separately because the two questions have very
different reliabilities.

| Question | Field | Measured rate | Used for |
|---|---|---|---|
| Is a Dolly there? | `present` | 99% with a Dolly in view | Confirming the approach |
| Is the bearing steerable? | `ok` | roughly a third of snapshots | A clamped trim only |

Reporting one combined number would hide which half failed. It would also
overstate what vision contributes: bearing error has a standard deviation of
2.6 degrees, which is 0.14 m at 3 m, against a coordinate dock that already
achieves 0.11 m. Applied unclamped, corrections of −0.34 m and −0.59 m were
observed on individual snapshots; either would have missed the Dolly.

---

## 4. Planning

```mermaid
flowchart TD
    inv["factory_inventory.json<br/>nodes, corridor edges, AMR spawns"]
    tasks["TASKS in the bridge<br/>T1..T7: dolly, pickup, dropoff"]

    inv --> cuopt["plan_cuopt.py<br/>cuOpt pickup-and-delivery"]
    tasks --> cuopt
    cuopt -- "min_vehicles = 3<br/>per-route cost cap" --> plan["cuopt_plan.json"]

    plan --> load["load_cuopt_plan()"]
    load --> score["planner._build_result()<br/>re-scored on one cost model"]

    inv --> manual["plan_manual()"] --> score
    inv --> greedy["plan(solver=greedy)"] --> score

    score --> chosen["chosen plan → missions"]
    score --> cands["plan_candidates → inventory"]
    cands --> viz["planner_panel.py<br/>planner_map.py"]
```

### Why cuOpt rather than the exhaustive solver

Not for speed or route quality. For seven tasks and three robots the
exhaustive solver in `mission_planner.py` returns an optimal makespan in under
2 ms, and cuOpt cannot beat optimal.

It is used because it can express constraints the others cannot. Asked only to
minimise distance, with no return trip, cuOpt gave all seven tasks to one robot
at 314.6 m — genuinely minimal, and useless as a fleet. `set_min_vehicles` and
`set_vehicle_max_costs` turn it into the 3/2/2 split the scenario calls for,
and neither is expressible in a solver that only minimises makespan.

Every candidate is re-scored through `planner._build_result` before comparison,
so the panel is not comparing one solver's arithmetic against another's, and a
stale plan file referencing tasks this run does not have fails loudly instead
of producing a plausible number.

---

## 5. Technology stack

| Layer | Choice | Note |
|---|---|---|
| Simulation | Isaac Sim 5.x | Factory USD, PhysX, RTX camera |
| Middleware | ROS 2 Jazzy, Fast DDS | Domain 130, whitelist profile |
| Fleet control | Per-robot FSM + claim-based traffic locking | No central executive |
| Task assignment | NVIDIA cuOpt (pickup-and-delivery) | Solved offline into JSON |
| Baselines | Exhaustive, greedy, manual | Same cost model, for comparison |
| Perception | HSV deck segmentation with geometric gates | Snapshot, not a loop |
| Camera model | Calibrated from rendered frames | fx measured, not requested |
| Visualisation | ROS image topics, `rqt_image_view` | Overlay, plan bars, factory map |

### On the camera model

The intrinsics file describes the lens rather than configuring it. Two attempts
to widen the field of view from that file both failed silently — the renderer
kept the authored optics while every consumer believed otherwise, which made a
Dolly appear 2.6 times larger than predicted and put bearings out by up to 35
degrees. Measuring `fx` off rendered frames against the Dolly's known 1.242 m
deck, and writing that measurement back, reduced bearing error from a standard
deviation of about 19 degrees to 2.6.
