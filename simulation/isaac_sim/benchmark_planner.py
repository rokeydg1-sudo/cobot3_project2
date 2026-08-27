#!/usr/bin/env python3
"""Reproducible benchmark: our planner vs NVIDIA cuOpt on the factory graph.

Answers one question honestly: at what problem size does a GPU VRP solver start
to pay off for this demo?

Run without cuOpt (any Python 3):

    python3 simulation/isaac_sim/benchmark_planner.py

Run with cuOpt (separate venv, keeps Isaac Sim's Python untouched):

    ~/.venvs/cuopt/bin/python simulation/isaac_sim/benchmark_planner.py

cuOpt install used for the recorded results:

    python3 -m venv ~/.venvs/cuopt
    ~/.venvs/cuopt/bin/pip install --extra-index-url https://pypi.nvidia.com \
        cuopt-cu12==26.8.0
"""

import argparse
import json
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mission_planner import (  # noqa: E402
    Task,
    Vehicle,
    WaypointGraph,
    plan,
    sequence_cost,
    solve_local_search,
)

DEFAULT_INVENTORY = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "factory_inventory.json"
)

# The authored waypoint graph of the factory USD.
EDGES = [
    (0, 3), (10, 8), (11, 13), (12, 10), (13, 1), (1, 12), (2, 0), (2, 4),
    (5, 4), (5, 7), (5, 8), (6, 3), (6, 7), (7, 9), (8, 9), (9, 11),
]

# tasks, vehicles
SIZES = [(6, 3), (10, 3), (15, 4), (25, 4), (40, 5), (60, 5)]

VEHICLE_HOMES = [("amr1", 10), ("amr2", 11), ("amr3", 5), ("amr4", 0), ("amr5", 2)]


def load_graph(inventory_path):
    with open(inventory_path, encoding="utf-8") as stream:
        inventory = json.load(stream)
    nodes = {
        int(n["id"]): (n["x"], n["y"])
        for n in inventory["nodes"]
        if n.get("id") is not None
    }
    edges = [(a, b, math.dist(nodes[a], nodes[b])) for a, b in EDGES]
    return WaypointGraph(nodes, edges)


def random_tasks(graph, count, seed):
    rng = random.Random(seed)
    node_ids = sorted(graph.nodes)
    tasks = []
    for i in range(count):
        pickup, dropoff = rng.sample(node_ids, 2)
        tasks.append(Task(f"T{i + 1}", f"dolly{i}", pickup, dropoff))
    return tasks


def total_cost(graph, buckets, vehicles):
    return sum(sequence_cost(graph, v, buckets[v.name]) for v in vehicles)


# ----------------------------------------------------------------- cuOpt


class CuOptSolver:
    """Thin wrapper so the benchmark still runs when cuOpt is absent."""

    def __init__(self, graph):
        import cudf  # noqa: F401  (import here so absence is detectable)
        import numpy as np
        from cuopt import routing

        self.np = np
        self.cudf = cudf
        self.routing = routing
        self.ids = sorted(graph.nodes)
        self.index = {n: i for i, n in enumerate(self.ids)}
        self.matrix = np.array(
            [[graph.cost(a, b) for b in self.ids] for a in self.ids],
            dtype=np.float32,
        )

    def solve(self, tasks, vehicles, seconds):
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
        # Our AMRs never return to a depot after the last drop.
        model.set_drop_return_trips(cudf.Series([True] * len(vehicles)))
        # One Dolly at a time, which forces a strict pickup -> delivery order.
        model.add_capacity_dimension(
            "dolly",
            cudf.Series(np.array(demand, dtype=np.int32)),
            cudf.Series(np.array([1] * len(vehicles), dtype=np.int32)),
        )

        settings = routing.SolverSettings()
        settings.set_time_limit(seconds)
        started = time.monotonic()
        solution = routing.Solve(model, settings)
        wall = time.monotonic() - started
        if solution.get_status() != 0:
            return None, wall

        routes = solution.get_route().to_pandas()
        # Depot rows reuse route index 0, so filter on the type column.
        routes = routes[routes["type"] == "Pickup"]
        name_of = {i: v.name for i, v in enumerate(vehicles)}
        task_of = {2 * k: tasks[k] for k in range(len(tasks))}
        buckets = {v.name: [] for v in vehicles}
        for truck, group in routes.groupby("truck_id"):
            for order in group["route"].tolist():
                buckets[name_of[truck]].append(task_of[order])
        return buckets, wall


# ----------------------------------------------------------------- runner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", default=DEFAULT_INVENTORY)
    parser.add_argument("--cuopt-seconds", type=float, default=3.0)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    graph = load_graph(args.inventory)

    cuopt = None
    try:
        cuopt = CuOptSolver(graph)
        print("cuOpt backend: available")
    except Exception as exc:  # noqa: BLE001 - any import/CUDA failure is fine
        print(f"cuOpt backend: unavailable ({type(exc).__name__}: {exc})")

    header = f"{'tasks':>6}{'veh':>5} | {'exact':>19} | {'local_search':>19}"
    if cuopt:
        header += f" | {'cuOpt(' + str(args.cuopt_seconds) + 's)':>19}"
    print()
    print(header)
    print(
        f"{'':>6}{'':>5} | {'total':>10}{'ms':>9} | {'total':>10}{'ms':>9}"
        + (f" | {'total':>10}{'ms':>9}" if cuopt else "")
    )

    for count, fleet_size in SIZES:
        tasks = random_tasks(graph, count, args.seed + count)
        vehicles = [Vehicle(n, s) for n, s in VEHICLE_HOMES[:fleet_size]]

        result = plan(graph, tasks, vehicles, solver="auto", objective="total")
        if result.solver == "exact":
            exact_cell = (
                f"{total_cost(graph, {a.vehicle: a.tasks for a in result.assignments}, vehicles):10.1f}"
                f"{result.solve_seconds * 1000:9.1f}"
            )
        else:
            exact_cell = f"{'n/a':>10}{'':>9}"

        started = time.monotonic()
        buckets = solve_local_search(graph, tasks, vehicles, "total")
        local_ms = (time.monotonic() - started) * 1000.0
        local_cell = f"{total_cost(graph, buckets, vehicles):10.1f}{local_ms:9.1f}"

        row = f"{count:>6}{fleet_size:>5} | {exact_cell} | {local_cell}"
        if cuopt:
            cu_buckets, wall = cuopt.solve(tasks, vehicles, args.cuopt_seconds)
            if cu_buckets is None:
                row += f" | {'FAIL':>10}{wall * 1000:9.1f}"
            else:
                row += (
                    f" | {total_cost(graph, cu_buckets, vehicles):10.1f}"
                    f"{wall * 1000:9.1f}"
                )
        print(row)

    print()
    print("total = summed travel distance in metres (lower is better)")
    print("exact = branch and bound, 'n/a' when the search space is too large")


if __name__ == "__main__":
    main()
