#!/usr/bin/env python3
"""Classify each transport task by the shape of the route it forces.

    python3 scripts/classify_tasks.py

The demonstration needs two properties that have nothing to do with efficiency:

* every AMR drives through at least one corner of the factory, so the audience
  sees each robot cover ground rather than shuffle in one aisle
* at least two of the three cross between the north and south halves, which
  makes their paths overlap and puts the traffic-edge lock to work

Efficiency solvers cannot express either. `mission_planner.solve_exact`
minimises makespan and will happily hand every corner to one robot, which is
exactly what happened: amr1 took T1, T2 and T4 while amr3 took only T6.

Before writing constraints it is worth knowing whether they can be satisfied at
all, and that is what this reports: which tasks touch a corner, which cross the
divide, and whether any assignment of six tasks to three robots can give each
robot a corner and at least two robots a crossing.
"""
import itertools
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ISAAC_DIR = REPO / "simulation" / "isaac_sim"
sys.path.insert(0, str(ISAAC_DIR))

import mission_planner as planner  # noqa: E402

# The four extremes of the waypoint graph. Corner rather than "far" because
# reaching one means turning through it, which is what reads as movement.
CORNER_NODES = {2, 3, 12, 13}

# Tasks are defined in standalone_factory_bridge.py; duplicated here rather than
# imported because importing that module starts Isaac Sim.
TASKS = [
    ("T1", 10, 12),
    ("T2", 10, 9),
    ("T3", 11, 7),
    ("T4", 11, 13),
    ("T5", 4, 8),
    ("T6", 6, 2),
]

FLEET = ["amr1", "amr2", "amr3"]


def load_graph():
    inventory = json.loads(
        (ISAAC_DIR / "factory_inventory.json").read_text(encoding="utf-8")
    )
    nodes = {
        int(n["id"]): (float(n["x"]), float(n["y"]))
        for n in inventory["nodes"]
        if n.get("id") is not None
    }
    raw = inventory.get("edges")
    if not raw:
        raise SystemExit(
            "factory_inventory.json has no 'edges'.\n"
            "Start the bridge once to regenerate it - the corridor topology is\n"
            "written by load_waypoint_graph(). Guessing the edges would make\n"
            "every route a straight line and the corner test meaningless."
        )
    edges = [(int(a), int(b), float(w)) for a, b, w in raw]
    return nodes, planner.WaypointGraph(nodes, edges)


def describe(nodes, graph, task):
    task_id, pickup, dropoff = task
    route, length = graph.shortest_path(pickup, dropoff)
    corners = sorted(set(route) & CORNER_NODES)
    py, dy = nodes[pickup][1], nodes[dropoff][1]
    crosses = (py > 0) != (dy > 0)
    return {
        "id": task_id,
        "pickup": pickup,
        "dropoff": dropoff,
        "route": route,
        "corners": corners,
        "crosses": crosses,
        "length": length,
    }


def main():
    nodes, graph = load_graph()
    facts = [describe(nodes, graph, task) for task in TASKS]

    print(f"{'task':5s} {'pickup':>7s} {'drop':>5s} {'len':>7s} "
          f"{'corner':>8s} {'crosses':>8s}  route")
    for fact in facts:
        print(
            f"{fact['id']:5s} {fact['pickup']:7d} {fact['dropoff']:5d} "
            f"{fact['length']:7.1f} "
            f"{str(fact['corners'] or '-'):>8s} "
            f"{'yes' if fact['crosses'] else 'no':>8s}  "
            f"{'->'.join(str(n) for n in fact['route'])}"
        )

    corner_tasks = {f["id"] for f in facts if f["corners"]}
    crossing_tasks = {f["id"] for f in facts if f["crosses"]}
    print(f"\ntasks touching a corner : {sorted(corner_tasks) or 'none'}")
    print(f"tasks crossing the divide: {sorted(crossing_tasks) or 'none'}")

    # Can the constraints be met at all? Enumerate every way of dealing six
    # tasks to three robots and count the feasible ones. Six tasks is small
    # enough that exhaustive is both exact and instant.
    feasible = []
    for labels in itertools.product(range(len(FLEET)), repeat=len(facts)):
        buckets = [[] for _ in FLEET]
        for index, owner in enumerate(labels):
            buckets[owner].append(facts[index])
        if any(not bucket for bucket in buckets):
            continue
        if not all(any(f["corners"] for f in bucket) for bucket in buckets):
            continue
        crossing_robots = sum(
            1 for bucket in buckets if any(f["crosses"] for f in bucket)
        )
        if crossing_robots < 2:
            continue
        feasible.append(
            (
                max(sum(f["length"] for f in bucket) for bucket in buckets),
                [[f["id"] for f in bucket] for bucket in buckets],
                crossing_robots,
            )
        )

    print(f"\nassignments satisfying both constraints: {len(feasible)}")
    if not feasible:
        print("  NOT SATISFIABLE with the current six tasks")
        print("  -> the task set itself has to change, not the solver")
        return

    feasible.sort()
    print(f"\n{'makespan':>9s} {'crossing':>9s}  assignment")
    for makespan, assignment, crossing in feasible[:5]:
        pretty = "  ".join(
            f"{name}[{','.join(ids)}]" for name, ids in zip(FLEET, assignment)
        )
        print(f"{makespan:9.1f} {crossing:9d}  {pretty}")
    print(f"\nbest makespan under constraints: {feasible[0][0]:.1f} m")


main()
