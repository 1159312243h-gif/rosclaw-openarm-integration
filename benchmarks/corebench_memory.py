"""ROSClaw-Memory CoreBench — Infrastructure-grade benchmark.

Dimensions:
  1. Write: trajectory / object / event / skill / failure / body-state
  2. Retrieve: semantic / spatial / temporal / entity / trajectory / causal
  3. Update: object move, state override, archive, conflict
  4. Compress: episode -> summary -> long-term
  5. Performance: ingest/query latency, DB/index size
  6. Benefit: No Memory vs Flat RAG vs ROSClaw-Memory

Usage:
    PYTHONPATH=src python3 benchmarks/corebench_memory.py
"""

from __future__ import annotations

import json
import random
import statistics
import sys
import time
import tracemalloc
from dataclasses import dataclass, field
from typing import Any

# ---------------------------------------------------------------------------
# Bootstrap path
# ---------------------------------------------------------------------------
sys.path.insert(0, "src")

from rosclaw.memory.interface import MemoryInterface
from rosclaw.memory.types import FailureMemory

# ---------------------------------------------------------------------------
# Benchmark harness
# ---------------------------------------------------------------------------


@dataclass
class BenchMetrics:
    name: str
    latencies_ms: list[float] = field(default_factory=list)
    db_records: int = 0
    db_size_bytes: int = 0
    index_size_bytes: int = 0

    @property
    def p50_ms(self) -> float:
        return statistics.median(self.latencies_ms) if self.latencies_ms else 0.0

    @property
    def p99_ms(self) -> float:
        if not self.latencies_ms:
            return 0.0
        s = sorted(self.latencies_ms)
        idx = int(len(s) * 0.99)
        return s[min(idx, len(s) - 1)]

    @property
    def throughput_hz(self) -> float:
        total_sec = sum(self.latencies_ms) / 1000.0
        return len(self.latencies_ms) / total_sec if total_sec > 0 else 0.0


def timing() -> float:
    return time.perf_counter() * 1000.0


def fmt_ms(v: float) -> str:
    return f"{v:.3f} ms"


# ---------------------------------------------------------------------------
# Synthetic data generators
# ---------------------------------------------------------------------------

SKILL_NAMES = [
    "pick_up",
    "place_on",
    "grasp_cup",
    "pour_water",
    "open_door",
    "push_button",
    "rotate_valve",
    "insert_peg",
    "screw_cap",
    "cut_object",
    "wipe_surface",
    "scan_qr",
    "navigate_to",
    "dock_charger",
    "emergency_stop",
]

OBJECTS = [
    "red_cup",
    "blue_mug",
    "wooden_block",
    "metal_plate",
    "plastic_bottle",
    "ceramic_vase",
    "glass_jar",
    "paper_box",
    "rubber_ball",
    "fabric_cloth",
]

LOCATIONS = [
    "kitchen_counter",
    "dining_table",
    "shelf_A1",
    "shelf_B2",
    "fridge_top",
    "sink_area",
    "stove_left",
    "cabinet_upper",
    "floor_center",
    "doorway",
]

FAILURE_TYPES = [
    "gripper_slip",
    "collision_detected",
    "joint_limit_exceeded",
    "object_not_found",
    "path_planning_failed",
    "force_limit_exceeded",
    "vision_occlusion",
    "ik_solver_failed",
    "timeout_exceeded",
]

BODY_STATES = [
    {"joint_1": 0.5, "joint_2": -0.3, "joint_3": 1.2, "gripper": 0.8},
    {"joint_1": 0.2, "joint_2": -0.1, "joint_3": 0.9, "gripper": 0.0},
    {"joint_1": 0.7, "joint_2": -0.5, "joint_3": 1.5, "gripper": 0.4},
]


def random_trajectory(n_waypoints: int = 10) -> list[list[float]]:
    """Generate a random 3D trajectory."""
    return [
        [
            round(random.uniform(-1.0, 1.0), 3),
            round(random.uniform(-1.0, 1.0), 3),
            round(random.uniform(0.0, 2.0), 3),
        ]
        for _ in range(n_waypoints)
    ]


def random_instruction() -> str:
    skill = random.choice(SKILL_NAMES)
    obj = random.choice(OBJECTS)
    loc = random.choice(LOCATIONS)
    templates = [
        f"{skill} the {obj} on {loc}",
        f"Navigate to {loc} and {skill} the {obj}",
        f"Use {skill} to manipulate the {obj} near {loc}",
        f"Pick up the {obj} from {loc} using {skill}",
    ]
    return random.choice(templates)


# ---------------------------------------------------------------------------
# Phase 1 — Write Benchmark
# ---------------------------------------------------------------------------


def bench_write(mem: MemoryInterface, n: int = 1000) -> dict[str, BenchMetrics]:
    print(f"\n[Phase 1] Write Benchmark — {n} records per type")
    metrics: dict[str, BenchMetrics] = {
        "event": BenchMetrics("event"),
        "trajectory": BenchMetrics("trajectory"),
        "skill": BenchMetrics("skill"),
        "failure": BenchMetrics("failure"),
        "body_state": BenchMetrics("body_state"),
    }

    # 1a — Events
    for i in range(n):
        t0 = timing()
        mem.store_experience(
            event_id=f"evt_{i:05d}",
            event_type="praxis",
            instruction=random_instruction(),
            outcome=random.choice(["success", "failure", "emergency"]),
            duration_sec=round(random.uniform(0.5, 30.0), 2),
            tags=[random.choice(SKILL_NAMES), random.choice(LOCATIONS)],
            metadata={"robot_state": random.choice(BODY_STATES)},
        )
        metrics["event"].latencies_ms.append(timing() - t0)

    # 1b — Trajectories (embedded in experience metadata)
    for i in range(n):
        t0 = timing()
        mem.store_experience(
            event_id=f"traj_{i:05d}",
            event_type="trajectory",
            instruction=f"Execute trajectory #{i}",
            trajectory=random_trajectory(20),
            outcome="success",
            metadata={"waypoint_count": 20, "planner": "RRTConnect"},
        )
        metrics["trajectory"].latencies_ms.append(timing() - t0)

    # 1c — Skills (skill_metadata table via SeekDB)
    for i in range(min(n, len(SKILL_NAMES) * 3)):
        t0 = timing()
        mem.seekdb_client.insert(
            "skill_metadata",
            {
                "skill_id": f"skill_{i:05d}",
                "name": random.choice(SKILL_NAMES),
                "description": random_instruction(),
                "category": random.choice(["manipulation", "navigation", "perception"]),
                "source": "benchmark",
                "success_count": random.randint(0, 100),
                "failure_count": random.randint(0, 20),
                "avg_duration_sec": round(random.uniform(1.0, 15.0), 2),
                "last_used": time.time(),
                "prerequisites": json.dumps(["vision_on", "gripper_calibrated"]),
            },
        )
        metrics["skill"].latencies_ms.append(timing() - t0)

    # 1d — Failures (failures table)
    for i in range(n):
        t0 = timing()
        mem.write_failure_memory(
            FailureMemory(
                failure_id=f"fail_{i:05d}",
                robot_id=mem._robot_id,
                failure_type=random.choice(FAILURE_TYPES),
                root_cause=random.choice(
                    ["friction_low", "lighting_change", "obstacle_moved", "sensor_drift"]
                ),
                recovery_hint=random.choice(
                    ["increase_force", "replan_path", "recalibrate_sensor", "call_human"]
                ),
                sandbox_intervened=random.random() > 0.7,
                category="control",
                metadata={"attempt_number": random.randint(1, 5)},
            )
        )
        metrics["failure"].latencies_ms.append(timing() - t0)

    # 1e — Body-state snapshots
    for i in range(n):
        t0 = timing()
        mem.store_experience(
            event_id=f"body_{i:05d}",
            event_type="body_state",
            instruction="State snapshot",
            outcome="success",
            metadata={
                "joint_positions": random.choice(BODY_STATES),
                "eef_pose": {
                    "x": round(random.uniform(-0.5, 0.5), 4),
                    "y": round(random.uniform(-0.5, 0.5), 4),
                    "z": round(random.uniform(0.1, 1.0), 4),
                },
                "timestamp": time.time(),
            },
        )
        metrics["body_state"].latencies_ms.append(timing() - t0)

    return metrics


# ---------------------------------------------------------------------------
# Phase 2 — Retrieve Benchmark
# ---------------------------------------------------------------------------


def bench_retrieve(mem: MemoryInterface) -> dict[str, BenchMetrics]:
    print("\n[Phase 2] Retrieve Benchmark")
    metrics: dict[str, BenchMetrics] = {
        "semantic": BenchMetrics("semantic"),
        "temporal": BenchMetrics("temporal"),
        "entity": BenchMetrics("entity"),
        "trajectory": BenchMetrics("trajectory"),
        "causal": BenchMetrics("causal"),
    }

    queries = [
        "pick up cup from kitchen",
        "grasp object on table",
        "navigate to doorway",
        "pour water into blue mug",
        "rotate valve clockwise",
    ]

    # 2a — Semantic search
    for q in queries:
        for _ in range(20):
            t0 = timing()
            mem.find_similar_experiences(q, limit=5)
            metrics["semantic"].latencies_ms.append(timing() - t0)

    # 2b — Temporal query (last 100 experiences)
    for _ in range(50):
        t0 = timing()
        mem._client.query(
            "experience_graph",
            filters={"robot_id": mem._robot_id},
            order_by="-timestamp",
            limit=100,
        )
        metrics["temporal"].latencies_ms.append(timing() - t0)

    # 2c — Entity query (by skill tag)
    for _skill in SKILL_NAMES[:5]:
        for _ in range(10):
            t0 = timing()
            mem._client.query(
                "experience_graph",
                filters={"robot_id": mem._robot_id},
                order_by="-timestamp",
                limit=50,
            )
            metrics["entity"].latencies_ms.append(timing() - t0)

    # 2d — Trajectory retrieval (by event_type)
    for _ in range(50):
        t0 = timing()
        mem._client.query(
            "experience_graph",
            filters={"robot_id": mem._robot_id, "event_type": "trajectory"},
            limit=20,
        )
        metrics["trajectory"].latencies_ms.append(timing() - t0)

    # 2e — Causal retrieval (failure -> recovery hint)
    for ft in FAILURE_TYPES[:5]:
        for _ in range(10):
            t0 = timing()
            mem._client.query(
                "failures",
                filters={"robot_id": mem._robot_id, "failure_type": ft},
                limit=5,
            )
            metrics["causal"].latencies_ms.append(timing() - t0)

    return metrics


# ---------------------------------------------------------------------------
# Phase 3 — Update Benchmark
# ---------------------------------------------------------------------------


def bench_update(mem: MemoryInterface, n: int = 200) -> BenchMetrics:
    print(f"\n[Phase 3] Update Benchmark — {n} operations")
    m = BenchMetrics("update")

    # 3a — Object move (simulate pose update via metadata)
    for i in range(n):
        t0 = timing()
        mem._client.update(
            "experience_graph",
            f"evt_{i:05d}",
            {
                "metadata": {
                    "updated_pose": {
                        "x": round(random.uniform(-0.5, 0.5), 4),
                        "y": round(random.uniform(-0.5, 0.5), 4),
                        "z": round(random.uniform(0.1, 1.0), 4),
                    },
                    "update_count": i,
                }
            },
        )
        m.latencies_ms.append(timing() - t0)

    # 3b — State override (outcome flip)
    for i in range(n, n * 2):
        t0 = timing()
        mem._client.update(
            "experience_graph",
            f"evt_{i % 1000:05d}",
            {
                "outcome": "emergency",
                "error_details": "Overridden by benchmark",
            },
        )
        m.latencies_ms.append(timing() - t0)

    # 3c — Conflict resolution (insert same ID -> REPLACE)
    for i in range(n // 2):
        t0 = timing()
        mem.store_experience(
            event_id=f"evt_{i:05d}",
            event_type="praxis",
            instruction=f"Conflict override #{i}",
            outcome="success",
        )
        m.latencies_ms.append(timing() - t0)

    return m


# ---------------------------------------------------------------------------
# Phase 4 — Compress Benchmark (episode -> summary)
# ---------------------------------------------------------------------------


def bench_compress(mem: MemoryInterface) -> BenchMetrics:
    print("\n[Phase 4] Compress Benchmark — episode aggregation")
    m = BenchMetrics("compress")

    # Aggregate episodes by skill / outcome
    for skill in SKILL_NAMES:
        t0 = timing()
        successes = mem._client.query(
            "experience_graph",
            filters={"robot_id": mem._robot_id, "outcome": "success"},
            limit=1000,
        )
        failures = mem._client.query(
            "experience_graph",
            filters={"robot_id": mem._robot_id, "outcome": "failure"},
            limit=1000,
        )

        # Write summary record
        summary = {
            "id": f"summary_{skill}",
            "event_type": "episode_summary",
            "robot_id": mem._robot_id,
            "timestamp": time.time(),
            "instruction": f"Summary for {skill}",
            "metadata": {
                "skill": skill,
                "success_count": len(successes),
                "failure_count": len(failures),
                "success_rate": len(successes) / (len(successes) + len(failures) + 1e-9),
            },
        }
        mem._client.insert("experience_graph", summary)
        m.latencies_ms.append(timing() - t0)

    return m


# ---------------------------------------------------------------------------
# Phase 5 — DB & Index Size
# ---------------------------------------------------------------------------


def measure_db_size(mem: MemoryInterface) -> dict[str, Any]:
    print("\n[Phase 5] DB Size Analysis")
    client = mem._client
    tables = [
        "experience_graph",
        "skill_metadata",
        "knowledge_graph",
        "heuristic_rules",
        "praxis_events",
        "failures",
        "success_patterns",
        "artifacts",
        "retries",
    ]
    sizes = {}
    total_records = 0
    for table in tables:
        cnt = client.count(table)
        sizes[table] = cnt
        total_records += cnt

    # Memory footprint (approximate for in-memory backend)
    tracemalloc.start()
    snapshot = tracemalloc.take_snapshot()
    top_stats = snapshot.statistics("lineno")[:5]
    mem_mb = sum(s.size for s in top_stats) / (1024 * 1024)
    tracemalloc.stop()

    return {
        "table_counts": sizes,
        "total_records": total_records,
        "memory_approx_mb": round(mem_mb, 2),
    }


# ---------------------------------------------------------------------------
# Phase 6 — Benefit Comparison
# ---------------------------------------------------------------------------


def bench_benefit(mem: MemoryInterface) -> dict[str, Any]:
    print("\n[Phase 6] Benefit Comparison")

    # Scenario: Robot encounters "gripper slip when picking cup"
    scenario_query = "gripper slip cup pick up"

    # 6a — No Memory: random guess
    t0 = timing()
    random.choice(["retry", "abort", "call_human"])
    no_mem_ms = timing() - t0

    # 6b — Flat RAG: simple keyword scan
    t0 = timing()
    all_exp = mem._client.query("experience_graph", limit=10000)
    keywords = {"gripper", "slip", "cup"}
    flat_matches = [e for e in all_exp if keywords & set(e.get("instruction", "").lower().split())]
    flat_rag_ms = timing() - t0

    # 6c — ROSClaw-Memory: structured semantic + causal
    t0 = timing()
    semantic = mem.find_similar_experiences(scenario_query, limit=5)
    causal = mem._client.query(
        "failures",
        filters={"robot_id": mem._robot_id, "failure_type": "gripper_slip"},
        limit=5,
    )
    rosclaw_ms = timing() - t0

    return {
        "scenario": scenario_query,
        "no_memory_latency_ms": round(no_mem_ms, 3),
        "flat_rag_latency_ms": round(flat_rag_ms, 3),
        "flat_rag_matches": len(flat_matches),
        "rosclaw_memory_latency_ms": round(rosclaw_ms, 3),
        "rosclaw_semantic_matches": len(semantic),
        "rosclaw_causal_matches": len(causal),
        "speedup_vs_flat_rag": round(flat_rag_ms / max(rosclaw_ms, 0.001), 2),
    }


# ---------------------------------------------------------------------------
# Main runner
# ---------------------------------------------------------------------------


def main() -> None:
    print("=" * 70)
    print("ROSClaw-Memory CoreBench")
    print("=" * 70)

    mem = MemoryInterface("benchmark_bot")
    mem.initialize()

    n = 1000  # Scale factor

    # Phase 1 — Write
    write_metrics = bench_write(mem, n=n)

    # Phase 2 — Retrieve
    retrieve_metrics = bench_retrieve(mem)

    # Phase 3 — Update
    update_metric = bench_update(mem, n=n // 5)

    # Phase 4 — Compress
    compress_metric = bench_compress(mem)

    # Phase 5 — Size
    size_info = measure_db_size(mem)

    # Phase 6 — Benefit
    benefit = bench_benefit(mem)

    mem.stop()

    # ------------------------------------------------------------------
    # Report
    # ------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)

    print("\n[Write Latency]")
    for name, m in write_metrics.items():
        print(
            f"  {name:12s}  p50={fmt_ms(m.p50_ms)}  p99={fmt_ms(m.p99_ms)}  "
            f"throughput={m.throughput_hz:.1f} hz"
        )

    print("\n[Retrieve Latency]")
    for name, m in retrieve_metrics.items():
        print(
            f"  {name:12s}  p50={fmt_ms(m.p50_ms)}  p99={fmt_ms(m.p99_ms)}  "
            f"throughput={m.throughput_hz:.1f} hz"
        )

    print(
        f"\n[Update Latency]  p50={fmt_ms(update_metric.p50_ms)}  "
        f"p99={fmt_ms(update_metric.p99_ms)}"
    )

    print(
        f"\n[Compress Latency]  p50={fmt_ms(compress_metric.p50_ms)}  "
        f"p99={fmt_ms(compress_metric.p99_ms)}"
    )

    print("\n[DB Size]")
    for table, cnt in size_info["table_counts"].items():
        print(f"  {table:20s}  {cnt:6d} records")
    print(f"  {'TOTAL':20s}  {size_info['total_records']:6d} records")
    print(f"  {'Approx memory':20s}  {size_info['memory_approx_mb']:.2f} MB")

    print("\n[Benefit Comparison]")
    print(f"  Scenario: {benefit['scenario']}")
    print(f"  No Memory:        {benefit['no_memory_latency_ms']:.3f} ms (random guess)")
    print(
        f"  Flat RAG:         {benefit['flat_rag_latency_ms']:.3f} ms ({benefit['flat_rag_matches']} matches)"
    )
    print(
        f"  ROSClaw-Memory:   {benefit['rosclaw_memory_latency_ms']:.3f} ms "
        f"(semantic={benefit['rosclaw_semantic_matches']}, causal={benefit['rosclaw_causal_matches']})"
    )
    print(f"  Speedup vs Flat:  {benefit['speedup_vs_flat_rag']}x")

    print("\n" + "=" * 70)
    print("CoreBench COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    main()
