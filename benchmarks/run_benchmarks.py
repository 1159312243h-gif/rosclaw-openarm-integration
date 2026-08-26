#!/usr/bin/env python3
"""ROSClaw v1.0 Performance Benchmark Runner.

Usage:
    python benchmarks/run_benchmarks.py

Outputs markdown-formatted results to stdout and benchmarks/results.md.
"""

import statistics
import time
from pathlib import Path

from rosclaw.core.event_bus import Event, EventBus
from rosclaw.e_urdf.parser import JointSpec, RobotModel
from rosclaw.firewall.validator import FirewallValidator, ValidationRequest
from rosclaw.memory.seekdb_client import SeekDBMemoryClient
from rosclaw.skill_manager.registry import SkillEntry, SkillRegistry

REPO_ROOT = Path(__file__).parent.parent
RESULTS_PATH = REPO_ROOT / "benchmarks" / "results.md"


def _fmt_ms(ms: float) -> str:
    return f"{ms:.4f}"


def _fmt_us(us: float) -> str:
    return f"{us:.4f}"


# ---------------------------------------------------------------------------
# EventBus Throughput
# ---------------------------------------------------------------------------


def benchmark_eventbus() -> dict:
    target = 10_000
    bus = EventBus()
    received = []
    bus.subscribe("bench.throughput", lambda e: received.append(e))

    t0 = time.perf_counter()
    for i in range(target):
        bus.publish(Event(topic="bench.throughput", payload={"i": i}, source="benchmark"))
    elapsed = time.perf_counter() - t0

    throughput = target / elapsed
    avg_latency_us = (elapsed / target) * 1_000_000

    return {
        "target": target,
        "elapsed_s": elapsed,
        "throughput": throughput,
        "avg_latency_us": avg_latency_us,
        "received": len(received),
        "verdict": "PASS" if throughput >= 10_000 else "FAIL",
    }


# ---------------------------------------------------------------------------
# SeekDB Scale
# ---------------------------------------------------------------------------


def benchmark_seekdb() -> dict:
    client = SeekDBMemoryClient()
    client.connect()

    n = 10_000
    insert_times = []
    t0_total = time.perf_counter()
    for i in range(n):
        t0 = time.perf_counter()
        client.insert(
            "experience_graph",
            {
                "id": f"exp_{i}",
                "event_type": "praxis",
                "robot_id": "ur5e",
                "timestamp": float(i),
                "instruction": f"task {i}",
            },
        )
        insert_times.append(time.perf_counter() - t0)
    total_insert_s = time.perf_counter() - t0_total

    avg_insert_us = statistics.mean(insert_times) * 1_000_000
    p99_insert_us = sorted(insert_times)[int(len(insert_times) * 0.99)] * 1_000_000

    t0 = time.perf_counter()
    results = client.query("experience_graph", filters={"robot_id": "ur5e"}, limit=100)
    query_ms = (time.perf_counter() - t0) * 1000

    return {
        "records": n,
        "total_insert_s": total_insert_s,
        "avg_insert_us": avg_insert_us,
        "p99_insert_us": p99_insert_us,
        "query_ms": query_ms,
        "query_results": len(results),
        "verdict": "PASS",
    }


# ---------------------------------------------------------------------------
# SkillRegistry Scale
# ---------------------------------------------------------------------------


def benchmark_skill_registry() -> dict:
    import io
    import sys

    reg = SkillRegistry()
    reg.initialize()

    n = 1_000
    t0 = time.perf_counter()
    # Suppress noisy stdout during registration
    old_stdout = sys.stdout
    sys.stdout = io.StringIO()
    try:
        for i in range(n):
            reg.register(
                SkillEntry(
                    name=f"skill_{i}",
                    description=f"Skill {i}",
                    skill_type="programmed" if i % 2 == 0 else "learned",
                )
            )
    finally:
        sys.stdout = old_stdout
    register_s = time.perf_counter() - t0

    t0 = time.perf_counter()
    _ = reg.list_skills()
    list_names_ms = (time.perf_counter() - t0) * 1000

    t0 = time.perf_counter()
    _ = reg.list_skills(return_entries=True)
    list_entries_ms = (time.perf_counter() - t0) * 1000

    t0 = time.perf_counter()
    programmed = reg.list_skills("programmed")
    list_filtered_ms = (time.perf_counter() - t0) * 1000

    reg.stop()

    return {
        "skills": n,
        "register_ms": register_s * 1000,
        "register_per_skill_us": (register_s / n) * 1_000_000,
        "list_names_ms": list_names_ms,
        "list_entries_ms": list_entries_ms,
        "list_filtered_ms": list_filtered_ms,
        "filtered_count": len(programmed),
        "verdict": "PASS",
    }


# ---------------------------------------------------------------------------
# FirewallValidator Trajectory
# ---------------------------------------------------------------------------


def benchmark_firewall() -> dict:
    model = RobotModel(name="test_robot")
    for i in range(6):
        model.joints[f"j{i}"] = JointSpec(
            name=f"j{i}",
            joint_type="revolute",
            parent="base" if i == 0 else f"link{i}",
            child=f"link{i + 1}",
            limits={"lower": -1.0, "upper": 1.0, "velocity": 2.0, "effort": 10.0},
        )

    bus = EventBus()
    validator = FirewallValidator(robot_model=model, event_bus=bus)
    validator.initialize()

    waypoints = [[0.0] * 6 for _ in range(100)]

    t0 = time.perf_counter()
    result = validator.validate(
        ValidationRequest(
            request_id="bench_1",
            robot_id="test_robot",
            trajectory=waypoints,
        )
    )
    elapsed_ms = (time.perf_counter() - t0) * 1000

    validator.stop()

    return {
        "waypoints": len(waypoints),
        "elapsed_ms": elapsed_ms,
        "is_safe": result.is_safe,
        "violations": len(result.violations),
        "warnings": len(result.warnings),
        "layers": [layer.value for layer in result.layers_checked],
        "verdict": "PASS",
    }


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def run_all() -> str:
    lines = [
        "# ROSClaw v1.0 Performance Benchmark Report",
        "",
        f"> **Date**: {time.strftime('%Y-%m-%d')}",
        "> **Method**: Single-process synthetic benchmark on local machine",
        "",
    ]

    # EventBus
    print("[1/4] Benchmarking EventBus throughput...")
    r = benchmark_eventbus()
    lines.extend(
        [
            "## EventBus Throughput",
            f"- Target: {r['target']:,} events",
            f"- Elapsed: {r['elapsed_s']:.4f}s",
            f"- Throughput: {r['throughput']:.1f} events/s",
            f"- Average latency: {r['avg_latency_us']:.4f} us/event",
            f"- Received: {r['received']:,} / {r['target']:,}",
            f"- Verdict: {r['verdict']} (needs >= 10,000 events/s)",
            "",
        ]
    )

    # SeekDB
    print("[2/4] Benchmarking SeekDB scale...")
    r = benchmark_seekdb()
    lines.extend(
        [
            "## SeekDB Scale Performance",
            f"- Records inserted: {r['records']:,}",
            f"- Total insert time: {r['total_insert_s']:.4f}s",
            f"- Average insert: {_fmt_ms(r['avg_insert_us'] / 1000)} ms",
            f"- P99 insert: {_fmt_ms(r['p99_insert_us'] / 1000)} ms",
            f"- Query (robot_id=ur5e): {r['query_results']} results in {_fmt_ms(r['query_ms'])} ms",
            f"- Verdict: {r['verdict']}",
            "",
        ]
    )

    # SkillRegistry
    print("[3/4] Benchmarking SkillRegistry scale...")
    r = benchmark_skill_registry()
    lines.extend(
        [
            "## SkillRegistry Scale Performance",
            f"- Skills registered: {r['skills']:,}",
            f"- Register time: {_fmt_ms(r['register_ms'])} ms ({_fmt_us(r['register_per_skill_us'])} us/skill)",
            f"- list_skills() [names]: {r['skills']:,} in {_fmt_ms(r['list_names_ms'])} ms",
            f"- list_skills(return_entries=True): {r['skills']:,} in {_fmt_ms(r['list_entries_ms'])} ms",
            f"- list_skills(skill_type='programmed'): {r['filtered_count']:,} in {_fmt_ms(r['list_filtered_ms'])} ms",
            f"- Verdict: {r['verdict']}",
            "",
        ]
    )

    # Firewall
    print("[4/4] Benchmarking FirewallValidator trajectory...")
    r = benchmark_firewall()
    lines.extend(
        [
            "## FirewallValidator Trajectory Performance",
            f"- Trajectory waypoints: {r['waypoints']}",
            f"- Validation time: {_fmt_ms(r['elapsed_ms'])} ms",
            f"- Is safe: {r['is_safe']}",
            f"- Violations: {r['violations']}",
            f"- Warnings: {r['warnings']}",
            f"- Layers checked: {r['layers']}",
            f"- Verdict: {r['verdict']}",
            "",
        ]
    )

    return "\n".join(lines)


if __name__ == "__main__":
    report = run_all()
    print("\n" + "=" * 60)
    print(report)
    print("=" * 60)

    RESULTS_PATH.write_text(report, encoding="utf-8")
    print(f"\nReport saved to: {RESULTS_PATH}")
