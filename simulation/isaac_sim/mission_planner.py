#!/usr/bin/env python3
"""Fleet task assignment for the Dolly transport demo.

Pure standard library so both PC1 (Isaac Sim) and PC2 (ROS 2) can import it.

Responsibilities split:

    WaypointGraph  shortest paths / travel-cost matrix   (Dijkstra, ours)
    solve(...)     which AMR takes which task, in what order
    caller         turns the assignment into drivable waypoint routes

The solver is pluggable so the same interface can be backed by an exact search,
a heuristic, OR-Tools or cuOpt without touching the callers.
"""

from dataclasses import dataclass, field
from heapq import heappop, heappush
from itertools import permutations
import math
import time


# ---------------------------------------------------------------- data model


@dataclass(frozen=True)
class Task:
    """Move one Dolly from its pickup node to a drop-off node."""

    task_id: str
    dolly: str
    pickup: int
    dropoff: int


@dataclass(frozen=True)
class Vehicle:
    name: str
    start_node: int


@dataclass
class Assignment:
    vehicle: str
    tasks: list = field(default_factory=list)   # ordered list[Task]
    cost: float = 0.0                            # metres travelled


@dataclass
class PlanResult:
    assignments: list                            # list[Assignment]
    solver: str
    solve_seconds: float
    makespan: float                              # longest single-vehicle cost
    total_distance: float

    def by_vehicle(self):
        return {a.vehicle: a for a in self.assignments}

    def summary(self):
        parts = [
            f"solver={self.solver}",
            f"solve={self.solve_seconds * 1000.0:.1f} ms",
            f"makespan={self.makespan:.1f} m",
            f"total={self.total_distance:.1f} m",
        ]
        for assignment in self.assignments:
            names = ",".join(t.task_id for t in assignment.tasks) or "-"
            parts.append(f"{assignment.vehicle}[{names}]={assignment.cost:.1f}m")
        return " | ".join(parts)


# ---------------------------------------------------------------- graph


class WaypointGraph:
    """Undirected waypoint graph with Dijkstra shortest paths."""

    def __init__(self, nodes, edges):
        # nodes: {node_id: (x, y)}   edges: [(a, b, weight)]
        self.nodes = dict(nodes)
        self.adjacency = {}
        for a, b, weight in edges:
            self.adjacency.setdefault(a, []).append((b, float(weight)))
            self.adjacency.setdefault(b, []).append((a, float(weight)))
        self._cache = {}

    def shortest_path(self, start, goal, reserved_edges=None, penalty=25.0):
        """Return (path, cost). Reserved edges are discouraged, not forbidden."""
        if start == goal:
            return [start], 0.0

        key = (start, goal, id(reserved_edges) if reserved_edges else 0)
        if reserved_edges is None and key in self._cache:
            return self._cache[key]

        distances = {start: 0.0}
        previous = {}
        queue = [(0.0, start)]
        visited = set()
        while queue:
            distance, current = heappop(queue)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                break
            for neighbour, weight in self.adjacency.get(current, []):
                if reserved_edges and frozenset((current, neighbour)) in reserved_edges:
                    weight *= penalty
                candidate = distance + weight
                if candidate < distances.get(neighbour, math.inf):
                    distances[neighbour] = candidate
                    previous[neighbour] = current
                    heappush(queue, (candidate, neighbour))

        if goal not in distances:
            raise ValueError(f"no route from {start} to {goal}")

        path = [goal]
        while path[-1] != start:
            path.append(previous[path[-1]])
        path.reverse()
        result = (path, distances[goal])
        if reserved_edges is None:
            self._cache[key] = result
        return result

    def cost(self, start, goal):
        return self.shortest_path(start, goal)[1]


def route_edges(route):
    return {frozenset((route[i], route[i + 1])) for i in range(len(route) - 1)}


def edge_key(edge):
    a, b = sorted(edge)
    return f"{a}-{b}"


# ---------------------------------------------------------------- costing


def sequence_cost(graph, vehicle, tasks):
    """Distance a vehicle drives to serve `tasks` in the given order."""
    total = 0.0
    position = vehicle.start_node
    for task in tasks:
        total += graph.cost(position, task.pickup)
        total += graph.cost(task.pickup, task.dropoff)
        position = task.dropoff
    return total


def _build_result(graph, vehicles, buckets, solver, seconds):
    assignments = []
    for vehicle in vehicles:
        tasks = buckets.get(vehicle.name, [])
        assignments.append(
            Assignment(vehicle.name, list(tasks), sequence_cost(graph, vehicle, tasks))
        )
    costs = [a.cost for a in assignments] or [0.0]
    return PlanResult(
        assignments=assignments,
        solver=solver,
        solve_seconds=seconds,
        makespan=max(costs),
        total_distance=sum(costs),
    )


# ---------------------------------------------------------------- solvers


def solve_greedy(graph, tasks, vehicles):
    """Baseline: hand each task to whichever vehicle finishes it soonest."""
    buckets = {v.name: [] for v in vehicles}
    position = {v.name: v.start_node for v in vehicles}
    elapsed = {v.name: 0.0 for v in vehicles}

    for task in tasks:
        best = None
        for vehicle in vehicles:
            finish = (
                elapsed[vehicle.name]
                + graph.cost(position[vehicle.name], task.pickup)
                + graph.cost(task.pickup, task.dropoff)
            )
            if best is None or finish < best[0]:
                best = (finish, vehicle.name)
        finish, name = best
        buckets[name].append(task)
        elapsed[name] = finish
        position[name] = task.dropoff
    return buckets


def solve_exact(graph, tasks, vehicles, objective="makespan", time_budget=10.0):
    """Branch and bound over every assignment *and* every ordering.

    Each task is tried at every insertion position of every vehicle, so the
    search really does cover all orderings; appending only would fix the service
    order to the input order and can miss the optimum. Returns None if the time
    budget runs out, letting the caller fall back to local search.
    """
    best = {"value": math.inf, "buckets": None}
    deadline = time.monotonic() + time_budget
    order = {v.name: [] for v in vehicles}
    by_name = {v.name: v for v in vehicles}
    cost = {v.name: 0.0 for v in vehicles}

    def value_of():
        costs = list(cost.values())
        return max(costs) if objective == "makespan" else sum(costs)

    def recurse(index):
        if time.monotonic() > deadline:
            raise TimeoutError
        # Costs only grow as tasks are added, so this is a valid lower bound.
        if value_of() >= best["value"]:
            return
        if index == len(tasks):
            best["value"] = value_of()
            best["buckets"] = {k: list(v) for k, v in order.items()}
            return
        task = tasks[index]
        for vehicle in vehicles:
            name = vehicle.name
            sequence = order[name]
            previous_cost = cost[name]
            for position in range(len(sequence) + 1):
                sequence.insert(position, task)
                cost[name] = sequence_cost(graph, by_name[name], sequence)
                recurse(index + 1)
                sequence.pop(position)
            cost[name] = previous_cost

    try:
        recurse(0)
    except TimeoutError:
        return None
    return best["buckets"]


def solve_local_search(graph, tasks, vehicles, objective="makespan", rounds=200):
    """Greedy start, then relocate/swap moves until no improvement."""

    def value(buckets):
        costs = [
            sequence_cost(graph, v, buckets[v.name]) for v in vehicles
        ]
        return max(costs) if objective == "makespan" else sum(costs)

    buckets = solve_greedy(graph, tasks, vehicles)
    best_value = value(buckets)

    for _ in range(rounds):
        improved = False
        names = [v.name for v in vehicles]

        # Relocate one task to another vehicle / another slot.
        for source in names:
            for i in range(len(buckets[source])):
                task = buckets[source][i]
                for target in names:
                    limit = len(buckets[target]) + (0 if target == source else 1)
                    for j in range(limit):
                        if target == source and j == i:
                            continue
                        trial = {k: list(v) for k, v in buckets.items()}
                        trial[source].pop(i)
                        trial[target].insert(j, task)
                        trial_value = value(trial)
                        if trial_value < best_value - 1e-9:
                            buckets, best_value, improved = trial, trial_value, True
                            break
                    if improved:
                        break
                if improved:
                    break
            if improved:
                break
        if improved:
            continue

        # Swap two tasks between vehicles.
        for a_index, a_name in enumerate(names):
            for b_name in names[a_index:]:
                for i in range(len(buckets[a_name])):
                    for j in range(len(buckets[b_name])):
                        if a_name == b_name and i >= j:
                            continue
                        trial = {k: list(v) for k, v in buckets.items()}
                        trial[a_name][i], trial[b_name][j] = (
                            trial[b_name][j],
                            trial[a_name][i],
                        )
                        trial_value = value(trial)
                        if trial_value < best_value - 1e-9:
                            buckets, best_value, improved = trial, trial_value, True
                            break
                    if improved:
                        break
                if improved:
                    break
            if improved:
                break
        if not improved:
            break
    return buckets


def solve_manual(graph, tasks, vehicles, mapping):
    """Honour a hand-written assignment, used as the comparison baseline."""
    by_id = {t.task_id: t for t in tasks}
    buckets = {v.name: [] for v in vehicles}
    for name, task_ids in mapping.items():
        buckets[name] = [by_id[t] for t in task_ids if t in by_id]
    return buckets


SOLVERS = {
    "greedy": solve_greedy,
    "exact": solve_exact,
    "local_search": solve_local_search,
}


def plan(graph, tasks, vehicles, solver="auto", objective="makespan"):
    """Assign tasks to vehicles. `solver='auto'` picks exact while it is cheap."""
    started = time.monotonic()

    chosen = solver
    buckets = None
    if solver in ("auto", "exact"):
        # Ways to arrange n distinct tasks into k ordered lists = (n+k-1)!/(k-1)!
        arrangements = math.factorial(len(tasks) + len(vehicles) - 1) // math.factorial(
            max(1, len(vehicles) - 1)
        )
        if arrangements <= 2_000_000:
            buckets = solve_exact(graph, tasks, vehicles, objective)
            chosen = "exact"
        if buckets is None and solver == "auto":
            buckets = solve_local_search(graph, tasks, vehicles, objective)
            chosen = "local_search"
    elif solver in SOLVERS:
        buckets = SOLVERS[solver](graph, tasks, vehicles)
        chosen = solver
    else:
        raise ValueError(f"unknown solver: {solver}")

    if buckets is None:
        buckets = solve_local_search(graph, tasks, vehicles, objective)
        chosen = "local_search"

    return _build_result(
        graph, vehicles, buckets, chosen, time.monotonic() - started
    )


def plan_manual(graph, tasks, vehicles, mapping):
    started = time.monotonic()
    buckets = solve_manual(graph, tasks, vehicles, mapping)
    return _build_result(
        graph, vehicles, buckets, "manual", time.monotonic() - started
    )


def nearest_node(graph, x, y):
    """Graph node closest to a world position, used for vehicle start points."""
    best, best_distance = None, math.inf
    for node_id, (nx, ny) in graph.nodes.items():
        distance = math.hypot(nx - x, ny - y)
        if distance < best_distance:
            best, best_distance = node_id, distance
    return best
