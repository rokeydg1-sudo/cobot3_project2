#!/usr/bin/env python3
"""Solve the fleet assignment with NVIDIA cuOpt and write it out as JSON.

    ~/.venvs/cuopt/bin/python scripts/plan_cuopt.py

Run separately from the bridge, not inside it. cuOpt needs cudf and lives in
~/.venvs/cuopt on Python 3.12, while the bridge runs on Isaac Sim's own Python
3.11 interpreter. Importing one from the other is not possible, and installing
cuOpt into Isaac's interpreter would drag cudf in alongside Isaac's own CUDA
stack. Solving ahead of time and handing over a file keeps both environments
untouched, and has the side benefit that the plan is inspectable and
reproducible rather than recomputed invisibly on every start.

The bridge picks the file up when PLAN_SOLVER=cuopt.

Requires factory_inventory.json to carry the edge list, which the bridge writes
on startup. Run the bridge once first if it is missing.
"""
import argparse
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ISAAC_DIR = REPO / "simulation" / "isaac_sim"
sys.path.insert(0, str(ISAAC_DIR))

import mission_planner as planner  # noqa: E402

OUTPUT = ISAAC_DIR / "cuopt_plan.json"

# Kept in step with TASKS in standalone_factory_bridge.py. Duplicated rather
# than imported because importing that module boots Isaac Sim.
TASKS = [
    ("T1", "/World/dolly_physics_01", 10, 12),
    ("T2", "/World/dolly_physics", 10, 9),
    ("T3", "/World/dolly_physics_03", 11, 7),
    ("T4", "/World/dolly_physics_02", 11, 13),
    ("T5", "/World/dolly_physics_04", 4, 8),
    ("T6", "/World/dolly_physics_07", 6, 2),
    ("T7", "/World/dolly_physics_06", 6, 5),
]


def load_graph_and_vehicles():
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
            "factory_inventory.json has no 'edges'. Start the bridge once so "
            "load_waypoint_graph() can write the corridor topology."
        )
    graph = planner.WaypointGraph(
        nodes, [(int(a), int(b), float(w)) for a, b, w in raw]
    )

    vehicles = []
    for robot in inventory.get("amrs", []):
        start = robot["amr_start"]
        vehicles.append(
            planner.Vehicle(
                robot["name"],
                planner.nearest_node(
                    graph, float(start["x"]), float(start["y"])
                ),
            )
        )
    if not vehicles:
        raise SystemExit("no AMRs in the inventory")
    return graph, vehicles


class CuOptSolver:
    """Pickup-and-delivery model over the waypoint cost matrix.

    One capacity unit per vehicle with +1 on pickup and -1 on delivery is what
    forces a strict pickup-then-delivery order: an AMR physically cannot carry
    two Dollies, and without the capacity dimension cuOpt is free to interleave
    them into a route the robot could not execute.
    """

    def __init__(self, graph):
        import cudf
        import numpy as np
        from cuopt import routing

        self.np, self.cudf, self.routing = np, cudf, routing
        self.ids = sorted(graph.nodes)
        self.index = {n: i for i, n in enumerate(self.ids)}
        self.matrix = np.array(
            [[graph.cost(a, b) for b in self.ids] for a in self.ids],
            dtype=np.float32,
        )

    def solve(self, tasks, vehicles, seconds, max_route_factor=1.6):
        np, cudf, routing = self.np, self.cudf, self.routing
        model = routing.DataModel(len(self.ids), len(vehicles), len(tasks) * 2)
        model.add_cost_matrix(cudf.DataFrame(self.matrix))

        order_locations, pickups, deliveries, demand = [], [], [], []
        for k, task in enumerate(tasks):
            order_locations += [self.index[task.pickup], self.index[task.dropoff]]
            pickups.append(2 * k)
            deliveries.append(2 * k + 1)
            demand += [1, -1]

        model.set_order_locations(
            cudf.Series(np.array(order_locations, dtype=np.int32))
        )
        model.set_pickup_delivery_pairs(
            cudf.Series(np.array(pickups, dtype=np.int32)),
            cudf.Series(np.array(deliveries, dtype=np.int32)),
        )
        homes = cudf.Series(
            np.array([self.index[v.start_node] for v in vehicles], dtype=np.int32)
        )
        model.set_vehicle_locations(homes, homes)
        # The AMRs stay where they finish; there is no depot to return to.
        model.set_drop_return_trips(cudf.Series([True] * len(vehicles)))
        model.add_capacity_dimension(
            "dolly",
            cudf.Series(np.array(demand, dtype=np.int32)),
            cudf.Series(np.array([1] * len(vehicles), dtype=np.int32)),
        )

        # Without these cuOpt hands every task to one AMR.
        #
        # There is no return trip, so total distance - which is what the solver
        # minimises by default - is genuinely lowest when a single vehicle
        # chains all seven pickups. The first run did exactly that: amr1 got
        # T1..T7 at 314.6 m while amr2 and amr3 stayed parked. Minimal, and
        # useless as a fleet demonstration.
        #
        # Two constraints turn it into a fleet problem. Requiring every vehicle
        # to be used stops the degenerate single-route answer, and capping each
        # route's cost forces the work to spread rather than piling onto
        # whichever vehicle happens to start nearest. The cap is set from the
        # ideal even share with headroom, so it constrains without dictating a
        # particular split.
        model.set_min_vehicles(len(vehicles))

        total_direct = sum(
            float(self.matrix[self.index[t.pickup]][self.index[t.dropoff]])
            for t in tasks
        )
        cap = total_direct / len(vehicles) * max_route_factor
        model.set_vehicle_max_costs(
            cudf.Series(np.array([cap] * len(vehicles), dtype=np.float32))
        )
        print(
            f"[cuopt] min_vehicles={len(vehicles)}, "
            f"per-route cost cap={cap:.1f} m",
            flush=True,
        )

        settings = routing.SolverSettings()
        settings.set_time_limit(seconds)
        started = time.monotonic()
        solution = routing.Solve(model, settings)
        wall = time.monotonic() - started
        if solution.get_status() != 0:
            return None, wall

        routes = solution.get_route().to_pandas()
        routes = routes[routes["type"] == "Pickup"]
        name_of = {i: v.name for i, v in enumerate(vehicles)}
        task_of = {2 * k: tasks[k] for k in range(len(tasks))}
        buckets = {v.name: [] for v in vehicles}
        for truck, group in routes.groupby("truck_id"):
            for order in group["route"].tolist():
                buckets[name_of[truck]].append(task_of[order])
        return buckets, wall


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--max-route-factor", type=float, default=1.6)
    parser.add_argument("--tasks", default="", help="comma separated task ids")
    parser.add_argument("--out", default=str(OUTPUT))
    args = parser.parse_args()

    wanted = (
        {t.strip() for t in args.tasks.split(",") if t.strip()}
        if args.tasks else {t[0] for t in TASKS}
    )
    graph, vehicles = load_graph_and_vehicles()
    tasks = [
        planner.Task(tid, dolly, pickup, dropoff)
        for tid, dolly, pickup, dropoff in TASKS
        if tid in wanted
    ]
    print(f"[cuopt] {len(tasks)} tasks, {len(vehicles)} vehicles", flush=True)

    solver = CuOptSolver(graph)
    buckets, wall = solver.solve(
        tasks, vehicles, args.seconds, args.max_route_factor
    )
    if buckets is None:
        raise SystemExit(f"cuOpt found no feasible solution in {wall:.2f}s")

    # Score it with the same cost model the other solvers are scored with, so
    # the comparison panel is apples to apples.
    result = planner._build_result(
        graph, vehicles, buckets, "cuopt", wall
    )

    payload = {
        "solver": "cuopt",
        "solve_seconds": wall,
        "makespan_m": result.makespan,
        "total_m": result.total_distance,
        "assignment": {
            name: [task.task_id for task in group]
            for name, group in buckets.items()
        },
    }
    Path(args.out).write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )

    print(f"[cuopt] solved in {wall * 1000:.1f} ms")
    print(f"[cuopt] makespan {result.makespan:.1f} m, "
          f"total {result.total_distance:.1f} m")
    for name, ids in payload["assignment"].items():
        print(f"  {name}: {ids}")
    print(f"[cuopt] wrote {args.out}")


main()
