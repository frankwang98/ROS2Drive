#!/usr/bin/env python3
"""Compare ROS2Drive benchmark JSON runs; no ROS installation required."""
import argparse
import json
import math
from pathlib import Path

FIELDS = ("loop_ms_mean", "loop_ms_p95", "loop_ms_max", "samples_with_faults", "distance_traveled_m")

def load(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError(f"{path}: unsupported benchmark schema")
    if not data.get("metrics_samples"):
        raise ValueError(f"{path}: no runtime metrics samples")
    return data

def compare(baseline, candidate):
    rows = []
    for key in FIELDS:
        old, new = baseline.get(key), candidate.get(key)
        if not isinstance(old, (int, float)) or not isinstance(new, (int, float)):
            rows.append((key, old, new, None))
            continue
        if not (math.isfinite(old) and math.isfinite(new)):
            raise ValueError(f"Non-finite metric: {key}")
        pct = (new - old) / abs(old) * 100 if old != 0 else None
        rows.append((key, old, new, pct))
    return rows

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument("--max-p95-regression-pct", type=float, default=None,
                        help="Fail when candidate p95 exceeds baseline by this percent")
    args = parser.parse_args()
    a, b = load(args.baseline), load(args.candidate)
    print("| Metric | Baseline | Candidate | Change |")
    print("|---|---:|---:|---:|")
    for name, old, new, pct in compare(a, b):
        print(f"| {name} | {old} | {new} | {pct:+.2f}% |" if pct is not None
              else f"| {name} | {old} | {new} | n/a |")
    if args.max_p95_regression_pct is not None:
        limit = args.max_p95_regression_pct
        if not math.isfinite(limit) or limit < 0:
            parser.error("--max-p95-regression-pct must be finite and nonnegative")
        old, new = a.get("loop_ms_p95"), b.get("loop_ms_p95")
        if old is None or new is None or old <= 0:
            parser.error("Need positive baseline p95 and valid candidate p95")
        if (new - old) / old * 100 > limit:
            raise SystemExit(1)

if __name__ == "__main__":
    main()
