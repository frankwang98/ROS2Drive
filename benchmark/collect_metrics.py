#!/usr/bin/env python3
"""Collect ROS2Drive runtime metrics and odometry into a reproducible JSON report.

Requires ROS 2 Python packages (rclpy, nav_msgs) and a sourced workspace.
Uses ROS time only for message stamps; wall time controls capture duration.
"""
import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from self_driving_car_demo.msg import RuntimeMetrics


class Collector(Node):
    def __init__(self, namespace):
        super().__init__("sdc_benchmark_collector")
        prefix = "/" + namespace.strip("/") if namespace.strip("/") else ""
        self.metrics = []
        self.odometry = []
        self.create_subscription(RuntimeMetrics, prefix + "/runtime/metrics", self.on_metrics, 10)
        self.create_subscription(Odometry, prefix + "/sdc/odometry", self.on_odom, 10)

    def on_metrics(self, msg):
        self.metrics.append({
            "sequence": int(msg.sequence),
            "loop_duration_ms": float(msg.loop_duration_ms),
            "trajectory_points": int(msg.trajectory_points),
            "active_faults": int(msg.active_faults),
            "runtime_state": int(msg.runtime_state),
        })

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odometry.append((float(p.x), float(p.y)))


def summarize(samples, odom, duration, namespace):
    times = sorted(x["loop_duration_ms"] for x in samples if math.isfinite(x["loop_duration_ms"]))
    distances = [
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(odom, odom[1:])
    ]
    p95 = times[math.ceil(0.95 * len(times)) - 1] if times else None
    total_distance = sum(distances)
    stationary = sum(d < 0.001 for d in distances)
    return {
        "schema_version": 1,
        "namespace": namespace,
        "capture_wall_seconds": duration,
        "metrics_samples": len(samples),
        "odometry_samples": len(odom),
        "loop_ms_mean": statistics.mean(times) if times else None,
        "loop_ms_p95": p95,
        "loop_ms_max": max(times) if times else None,
        "samples_with_faults": sum(x["active_faults"] > 0 for x in samples),
        "distance_traveled_m": total_distance,
        "stationary_odometry_steps": stationary,
        "fault_sample_ratio": (sum(x["active_faults"] > 0 for x in samples) / len(samples)) if samples else None,
        "trajectory_points_mean": statistics.mean(x["trajectory_points"] for x in samples) if samples else None,
        "limitations": [
            "No path tracking error or collision success metric is inferred from these topics.",
            "Distance is accumulated odometry displacement, not mission completion.",
            "Compare runs only with the same scenario, duration, machine and configuration.",
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0, help="Capture seconds (wall clock)")
    parser.add_argument("--namespace", default="", help="ROS namespace, e.g. car01")
    parser.add_argument("--output", default="benchmark-result.json")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be finite and positive")
    rclpy.init()
    node = Collector(args.namespace)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - started
        report = summarize(node.metrics, node.odometry, elapsed, args.namespace)
        node.destroy_node()
        rclpy.shutdown()
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report["metrics_samples"]:
        print("WARNING: no /runtime/metrics samples received; check namespace and topic names.")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
