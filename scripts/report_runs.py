#!/usr/bin/env python3
"""Measure a run against criteria fixed before the run, from the logs it wrote.

    python3 scripts/report_runs.py                 # read logs/, print a table
    python3 scripts/report_runs.py --write         # also write the report files

Reads the controller logs rather than instrumenting the controllers. The logs
are what the run actually produced, so a report built from them cannot claim
something the run did not do - and a run recorded weeks ago can still be
re-scored when the criteria change.

Success criteria, stated here so they are fixed rather than chosen after seeing
the numbers:

    a transport succeeds  when the controller logs MISSION n COMPLETE
    a dock succeeds       when position error <= DOCK_POSITION_TOLERANCE_M
                          and |yaw error| <= DOCK_YAW_TOLERANCE_DEG
    a snapshot succeeds   when the Dolly was recognised, whether or not the
                          bearing was precise enough to steer on

The last one is deliberate. Recognition and measurement are separate questions
with very different reliabilities, and scoring them together would hide which
one failed. A refused bearing is a designed outcome: the dock continues on
coordinates and the accuracy figure above still has to be met.
"""
import argparse
import json
import re
import statistics
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
LOG_DIR = REPO / "logs"

# Fixed acceptance thresholds.
DOCK_POSITION_TOLERANCE_M = 0.15
DOCK_YAW_TOLERANCE_DEG = 6.0

DOCK = re.compile(
    r"DOCK OK \| target=\(([-\d.]+), ([-\d.]+), ([-\d.]+) deg\) "
    r"actual=\(([-\d.]+), ([-\d.]+), ([-\d.]+) deg\) "
    r"position_error=([\d.]+) m yaw_error=([-\d.]+) deg"
)
MISSION = re.compile(
    r"MISSION (\d+) COMPLETE \((\w+)\) \| ([\d.]+) s \| ([\d.]+) m"
)
METRICS = re.compile(
    r"METRICS (\w+) \| total ([\d.]+) s \| travelled ([\d.]+) m \| (.*)"
)
SNAP_MEASURED = re.compile(
    r"SNAPSHOT DOLLY seen at ([-+\d.]+) deg \(expected ([-+\d.]+) deg, "
    r"area ([\d.]+)%, (\d+)/(\d+) frames\) -> lateral ([-+\d.]+) m, "
    r"applying ([-+\d.]+) m"
)
SNAP_CONFIRMED = re.compile(
    r"SNAPSHOT DOLLY CONFIRMED on (\d+)/(\d+) frames.*?\((.*?)\)"
)
SNAP_NONE = re.compile(r"SNAPSHOT found no Dolly \((.*?)\)")
SNAP_TIMEOUT = re.compile(r"SNAPSHOT timed out")
TRAFFIC_WAIT = re.compile(r"waiting for edge|traffic wait")


def parse(path):
    text = path.read_text(errors="replace")
    docks = [
        {
            "position_error_m": float(m.group(7)),
            "yaw_error_deg": float(m.group(8)),
        }
        for m in DOCK.finditer(text)
    ]
    missions = [
        {
            "index": int(m.group(1)),
            "task": m.group(2),
            "seconds": float(m.group(3)),
            "metres": float(m.group(4)),
        }
        for m in MISSION.finditer(text)
    ]
    metrics = None
    for m in METRICS.finditer(text):
        metrics = {
            "amr": m.group(1),
            "total_seconds": float(m.group(2)),
            "travelled_m": float(m.group(3)),
        }

    snapshots = []
    for m in SNAP_MEASURED.finditer(text):
        snapshots.append(
            {
                "outcome": "measured",
                "bearing_deg": float(m.group(1)),
                "expected_deg": float(m.group(2)),
                "area_pct": float(m.group(3)),
                "frames": f"{m.group(4)}/{m.group(5)}",
                "lateral_m": float(m.group(6)),
                "applied_m": float(m.group(7)),
            }
        )
    for m in SNAP_CONFIRMED.finditer(text):
        snapshots.append(
            {
                "outcome": "recognised",
                "frames": f"{m.group(1)}/{m.group(2)}",
                "reason": m.group(3),
            }
        )
    for m in SNAP_NONE.finditer(text):
        snapshots.append({"outcome": "not recognised", "reason": m.group(1)})
    for _ in SNAP_TIMEOUT.finditer(text):
        snapshots.append({"outcome": "timeout"})

    return {
        "log": path.name,
        "amr": metrics["amr"] if metrics else path.stem,
        "docks": docks,
        "missions": missions,
        "metrics": metrics,
        "snapshots": snapshots,
        "traffic_waits": len(TRAFFIC_WAIT.findall(text)),
        "completed": bool(re.search(r"ALL MISSIONS DONE", text)),
    }


def collect(log_dir):
    logs = sorted(log_dir.glob("amr*.log"))
    if not logs:
        raise SystemExit(
            f"no amr*.log in {log_dir}. Run the fleet first; the launchers in "
            "scripts/ write there."
        )
    return [parse(path) for path in logs]


def render(runs):
    lines = []

    def out(text=""):
        lines.append(text)

    out("=" * 74)
    out("RUN REPORT")
    out("=" * 74)
    out(
        f"criteria: dock position <= {DOCK_POSITION_TOLERANCE_M} m, "
        f"|yaw| <= {DOCK_YAW_TOLERANCE_DEG} deg"
    )
    out()

    out(f"{'amr':6s} {'missions':>9s} {'docks':>6s} {'dock ok':>8s} "
        f"{'time':>8s} {'distance':>9s} {'finished':>9s}")
    total_missions = total_docks = passed_docks = 0
    for run in runs:
        ok = sum(
            1 for d in run["docks"]
            if d["position_error_m"] <= DOCK_POSITION_TOLERANCE_M
            and abs(d["yaw_error_deg"]) <= DOCK_YAW_TOLERANCE_DEG
        )
        total_missions += len(run["missions"])
        total_docks += len(run["docks"])
        passed_docks += ok
        metrics = run["metrics"] or {}
        out(
            f"{run['amr']:6s} {len(run['missions']):9d} {len(run['docks']):6d} "
            f"{ok:4d}/{len(run['docks']):<3d} "
            f"{metrics.get('total_seconds', 0.0):7.1f}s "
            f"{metrics.get('travelled_m', 0.0):8.1f}m "
            f"{'yes' if run['completed'] else 'NO':>9s}"
        )

    out()
    errors = [
        d["position_error_m"] for run in runs for d in run["docks"]
    ]
    yaws = [
        abs(d["yaw_error_deg"]) for run in runs for d in run["docks"]
    ]
    out(f"transports completed : {total_missions}")
    out(
        f"docks within tolerance: {passed_docks}/{total_docks}"
        + (f"  ({100.0 * passed_docks / total_docks:.0f}%)" if total_docks else "")
    )
    if errors:
        out(
            f"dock position error  : min {min(errors):.3f}  "
            f"median {statistics.median(errors):.3f}  max {max(errors):.3f} m"
        )
        out(
            f"dock yaw error       : min {min(yaws):.2f}  "
            f"median {statistics.median(yaws):.2f}  max {max(yaws):.2f} deg"
        )

    # Vision, reported as two separate rates on purpose.
    snapshots = [s for run in runs for s in run["snapshots"]]
    if snapshots:
        recognised = sum(
            1 for s in snapshots if s["outcome"] in ("measured", "recognised")
        )
        measured = sum(1 for s in snapshots if s["outcome"] == "measured")
        out()
        out(f"snapshots taken      : {len(snapshots)}")
        out(
            f"  dolly recognised   : {recognised}/{len(snapshots)}"
            f"  ({100.0 * recognised / len(snapshots):.0f}%)"
        )
        out(
            f"  bearing usable     : {measured}/{len(snapshots)}"
            f"  ({100.0 * measured / len(snapshots):.0f}%)"
        )
        refusals = {}
        for s in snapshots:
            if s["outcome"] != "measured" and s.get("reason"):
                key = s["reason"].split("(")[0].strip()
                refusals[key] = refusals.get(key, 0) + 1
        for reason, count in sorted(refusals.items(), key=lambda kv: -kv[1]):
            out(f"    {count:3d}  {reason}")
        for s in snapshots:
            if s["outcome"] == "measured":
                out(
                    f"  measured {s['bearing_deg']:+.2f} deg vs expected "
                    f"{s['expected_deg']:+.2f}, applied {s['applied_m']:+.3f} m "
                    f"of {s['lateral_m']:+.3f} m"
                )

    waits = sum(run["traffic_waits"] for run in runs)
    out()
    out(f"traffic waits        : {waits}")

    plan = REPO / "simulation" / "isaac_sim" / "factory_inventory.json"
    if plan.exists():
        data = json.loads(plan.read_text(errors="replace"))
        fleet = data.get("fleet_plan") or {}
        if fleet:
            out(
                f"plan                 : solver={fleet.get('solver')} "
                f"makespan={fleet.get('makespan_m', 0.0):.1f} m "
                f"solve={fleet.get('solve_seconds', 0.0) * 1000.0:.1f} ms"
            )

    out("=" * 74)
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--logs", default=str(LOG_DIR))
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    log_dir = Path(args.logs)
    runs = collect(log_dir)
    report = render(runs)
    print(report)

    if args.write:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        text_path = log_dir / f"report_{stamp}.txt"
        json_path = log_dir / f"report_{stamp}.json"
        text_path.write_text(report + "\n", encoding="utf-8")
        json_path.write_text(
            json.dumps(
                {
                    "generated": stamp,
                    "criteria": {
                        "dock_position_tolerance_m": DOCK_POSITION_TOLERANCE_M,
                        "dock_yaw_tolerance_deg": DOCK_YAW_TOLERANCE_DEG,
                    },
                    "runs": runs,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"\nwrote {text_path}")
        print(f"wrote {json_path}")


main()
