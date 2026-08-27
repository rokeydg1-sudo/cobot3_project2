#!/usr/bin/env python3
"""Search for a six-task set that a 2/2/2 fleet can share evenly.

    python3 scripts/design_tasks.py

The existing tasks cannot meet the demonstration constraints. T6 alone is
56.3 m where an even third of the total is 61.7 m, and because T2 and T6 are the
only crossing tasks they have to go to different robots, which forces T6 to pair
with a long task and leaves one robot at 90 m against another at 45 m.

So the task set is the thing to change, not the assignment. Requirements, in the
order they were asked for:

* exactly two tasks per robot
* every robot passes through at least one corner of the factory
* at least two robots cross between the north and south halves
* the three robots travel similar distances

Pickups are constrained by where the Dollies actually are: eight Dollies sit in
pairs near nodes 10, 11, 4 and 6, so at most two tasks may start at any of them.
Dropoffs are free.

The search is exhaustive over dropoff choices for a fixed pickup multiset, which
keeps it honest - the result is the best available set rather than the first one
that looked plausible.
"""
import itertools
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ISAAC_DIR = REPO / "simulation" / "isaac_sim"
sys.path.insert(0, str(ISAAC_DIR))

import mission_planner as planner  # noqa: E402

CORNER_NODES = {2, 3, 12, 13}

# Two Dollies are parked at each of these, so each may host at most two tasks.
PICKUP_CAPACITY = {10: 2, 11: 2, 4: 2, 6: 2}

# One task per robot pair, six in total, two per robot.
TASKS_PER_ROBOT = 2
FLEET_SIZE = 3


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
            "factory_inventory.json has no 'edges'; start the bridge once."
        )
    edges = [(int(a), int(b), float(w)) for a, b, w in raw]
    return nodes, planner.WaypointGraph(nodes, edges)


def task_facts(nodes, graph, pickup, dropoff):
    route, length = graph.shortest_path(pickup, dropoff)
    return {
        "pickup": pickup,
        "dropoff": dropoff,
        "route": route,
        "length": length,
        "corner": bool(set(route) & CORNER_NODES),
        "crosses": (nodes[pickup][1] > 0) != (nodes[dropoff][1] > 0),
    }


def best_assignment(facts):
    """Best 2/2/2 split of six tasks meeting both shape constraints.

    Returns (spread, makespan, buckets) or None. Spread first because even
    distances are the stated goal; makespan only breaks ties.
    """
    best = None
    indices = range(len(facts))
    for first in itertools.combinations(indices, TASKS_PER_ROBOT):
        rest = [i for i in indices if i not in first]
        for second in itertools.combinations(rest, TASKS_PER_ROBOT):
            third = tuple(i for i in rest if i not in second)
            groups = (first, second, third)
            if not all(
                any(facts[i]["corner"] for i in group) for group in groups
            ):
                continue
            crossing = sum(
                1 for group in groups if any(facts[i]["crosses"] for i in group)
            )
            if crossing < 2:
                continue
            loads = [sum(facts[i]["length"] for i in group) for group in groups]
            candidate = (max(loads) - min(loads), max(loads), groups)
            if best is None or candidate[:2] < best[:2]:
                best = candidate
    return best


def main():
    nodes, graph = load_graph()
    node_ids = sorted(nodes)

    # Six pickups drawn from the Dolly parks, respecting two per park.
    pickup_pool = []
    for node, capacity in PICKUP_CAPACITY.items():
        pickup_pool.extend([node] * capacity)

    # Prune before combining, not after.
    #
    # The unfiltered space is seven pickup multisets times 13^6 dropoff choices,
    # about 34 million sets each needing a 90-way assignment search - it does not
    # finish. Almost all of it is unusable anyway: an even 2/2/2 split of six
    # tasks means each robot carries about a third of the total, so a task much
    # shorter or longer than a sixth of that cannot be balanced by anything it
    # is paired with. Restricting task length to a sensible band first cuts the
    # space by orders of magnitude without discarding any balanced solution.
    LENGTH_MIN, LENGTH_MAX = 18.0, 46.0

    results = []
    seen = set()
    for pickups in itertools.combinations(sorted(pickup_pool), 6):
        if pickups in seen:
            continue
        seen.add(pickups)
        # Dropoff options per pickup: anywhere that is not the pickup itself.
        options = []
        for p in pickups:
            candidates = [
                f for f in (
                    task_facts(nodes, graph, p, d) for d in node_ids if d != p
                )
                if LENGTH_MIN <= f["length"] <= LENGTH_MAX
            ]
            options.append(candidates)
        if any(not c for c in options):
            continue
        for combo in itertools.product(*options):
            if len({(f["pickup"], f["dropoff"]) for f in combo}) < 6:
                continue
            assignment = best_assignment(list(combo))
            if assignment is None:
                continue
            spread, makespan, groups = assignment
            results.append((spread, makespan, list(combo), groups))

    if not results:
        print("no task set satisfies the constraints")
        return

    results.sort(key=lambda r: (r[0], r[1]))
    spread, makespan, facts, groups = results[0]

    print(f"searched {len(results)} feasible task sets\n")
    print(f"{'task':5s} {'pickup':>7s} {'drop':>5s} {'len':>7s} "
          f"{'corner':>7s} {'cross':>6s}  route")
    labels = {}
    for index, fact in enumerate(facts):
        labels[index] = f"T{index + 1}"
        print(
            f"{labels[index]:5s} {fact['pickup']:7d} {fact['dropoff']:5d} "
            f"{fact['length']:7.1f} "
            f"{'yes' if fact['corner'] else '-':>7s} "
            f"{'yes' if fact['crosses'] else '-':>6s}  "
            f"{'->'.join(str(n) for n in fact['route'])}"
        )

    print("\nassignment")
    for name, group in zip(("amr1", "amr2", "amr3"), groups):
        load = sum(facts[i]["length"] for i in group)
        corner = any(facts[i]["corner"] for i in group)
        crosses = any(facts[i]["crosses"] for i in group)
        print(
            f"  {name}  {[labels[i] for i in group]}  {load:6.1f} m  "
            f"corner={'yes' if corner else 'NO'}  "
            f"cross={'yes' if crosses else '-'}"
        )
    print(f"\ndistance spread {spread:.1f} m, makespan {makespan:.1f} m")

    print("\nTASKS block for standalone_factory_bridge.py:")
    for index, fact in enumerate(facts):
        print(
            f'    {{"id": "T{index + 1}", "dolly": "<assign>", '
            f'"pickup": {fact["pickup"]}, "dropoff": {fact["dropoff"]}}},'
        )


main()
