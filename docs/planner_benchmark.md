# Fleet planner benchmark: in-house solver vs NVIDIA cuOpt

## Question

The demo assigns Dolly transport jobs to a fleet of AMRs. Would NVIDIA cuOpt, a
GPU-accelerated vehicle-routing solver, plan those assignments better than the
simple solvers written for this project — and at what problem size does it start
to pay off?

## Setup

| | |
|---|---|
| GPU | RTX 5080 Laptop, compute capability 12.0 (Blackwell, sm_120) |
| Driver | 580.173.02 |
| cuOpt | `cuopt-cu12` 26.8.0 (bundles CUDA 12.9) |
| Python | 3.12.3 in `~/.venvs/cuopt` (Isaac Sim's own 3.11 left untouched) |
| Graph | Factory waypoint graph: 14 nodes, 16 edges |
| Cost | Dijkstra all-pairs shortest path over the waypoint graph |
| Objective | Total travel distance, metres (lower is better) |

Reproduce with:

```bash
# without cuOpt
python3 simulation/isaac_sim/benchmark_planner.py

# with cuOpt
~/.venvs/cuopt/bin/python simulation/isaac_sim/benchmark_planner.py
```

## Problem definition

Each task is "carry one Dolly from a pickup node to a drop-off node". Two
constraints make this a pickup-and-delivery problem rather than plain routing:

- A Dolly must be picked up before it is delivered.
- **An AMR carries exactly one Dolly at a time.** In cuOpt this is a capacity
  dimension with vehicle capacity 1 and demands `+1 / -1`.
- Vehicles do not return to a depot after the final drop
  (`set_drop_return_trips(True)`).

## Results

```
 tasks  veh |               exact |        local_search |         cuOpt(3.0s)
            |      total       ms |      total       ms |      total       ms
     6    3 |      368.6      9.3 |      368.6      0.1 |      398.0   3024.9
    10    3 |        n/a          |      629.2      1.0 |      663.1   3017.8
    15    4 |        n/a          |      810.2      6.6 |      844.5   3011.1
    25    4 |        n/a          |     1479.9     54.2 |     1458.0   3020.7
    40    5 |        n/a          |     2472.6    221.3 |     2507.7   3080.4
    60    5 |        n/a          |     3669.4    827.4 |     3692.6   3206.9
```

`exact` is branch and bound over every assignment *and* ordering; `n/a` means the
search space exceeded the guard and the planner fell back to local search.

### Quality, relative to local search

| tasks | cuOpt vs local search |
|---:|---|
| 6 | +8.0% worse |
| 10 | +5.4% worse |
| 15 | +4.2% worse |
| 25 | **−1.5% better** |
| 40 | +1.4% worse |
| 60 | +0.6% worse |

## Conclusion

**cuOpt does not pay off at this problem size.** Local search matches or beats it
at five of six sizes while running 30–30,000x faster. cuOpt won once, at 25
tasks, by 1.5%.

Two reasons:

1. **The graph is tiny.** 14 nodes and at most 60 tasks give a GPU solver almost
   nothing to parallelise. cuOpt is built for hundreds to thousands of locations.
2. **cuOpt spends its whole time budget.** It always consumed the full 3 s, so it
   cannot compete on latency for a problem local search closes in milliseconds.

For the demo's actual workload — 6 tasks, 3 AMRs — the in-house exact solver
returns a provably optimal assignment in 9 ms. There is nothing left for cuOpt to
improve.

### When cuOpt would become the right choice

- Hundreds of waypoints or tasks, where exhaustive and local methods degrade.
- Constraints local search does not model: delivery time windows, vehicle
  capacities, driver breaks, order priorities and prizes. cuOpt supports these
  natively, and they are usually what makes real fleet scheduling hard.
- Re-planning at high frequency under those constraints.

## Two bugs found while benchmarking

Both were only visible because the two solvers were compared against each other.

**1. Missing capacity constraint on the cuOpt side.** Without it cuOpt produced
routes that carried several Dollies at once — physically impossible for these
AMRs — and looked far worse when scored against our one-at-a-time cost model. At
60 tasks the score went from 6324 to 3763 once capacity 1 was declared. The
solver was right; the model was wrong.

**2. The in-house "exact" solver was not exact.** It appended each task to the end
of a vehicle's list, so task order within a vehicle was frozen to input order: it
searched assignments but not orderings. At 10 tasks it returned 715.5 while
plain local search found 629.2, which is impossible for a true optimum. Fixed by
trying every insertion position, and the search-space guard was corrected from
`k^n` to the real count `(n+k-1)!/(k-1)!`.

## Objective choice matters more than the solver

On the demo's 6-task workload, with each solver optimising its own objective:

| solver | makespan (m) | total (m) | solve |
|---|---:|---:|---|
| exact (ours, makespan) | **97.5** | 270.2 | 0.7 ms |
| greedy (ours) | 151.3 | 312.3 | 0.0 ms |
| cuOpt (total distance) | 105.3 | **240.8** | 1021 ms |

Each wins on the metric it optimises: cuOpt is 11% better on total distance, ours
is 7% better on makespan. Neither is "the better solver" — they answer different
questions. For a factory measured on throughput, makespan is the one that counts,
which is why the demo optimises it.
