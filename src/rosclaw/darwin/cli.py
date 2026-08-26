"""Darwin benchmark CLI dispatch."""

from __future__ import annotations

import argparse
import json

from rosclaw.darwin.engine import DarwinEngine
from rosclaw.darwin.scenarios import list_scenarios


def cmd_darwin(args: argparse.Namespace) -> int:
    """Dispatch Darwin benchmark subcommands."""
    sub = getattr(args, "darwin_command", None)

    if sub == "run":
        engine = DarwinEngine(default_seeds=args.seeds, default_episodes=args.episodes)
        event = engine.run_benchmark(
            task_id=args.task_id,
            skill_id=args.skill_id,
            candidate_skill_id=getattr(args, "candidate_skill_id", None) or None,
            scenario_id=getattr(args, "scenario_id", None) or None,
            seeds=args.seeds,
            episodes=args.episodes,
        )
        if args.json:
            print(json.dumps(event.to_dict(), indent=2, default=str))
        else:
            print(f"Benchmark {event.benchmark_id}: regression={event.regression_detected}")
            baseline = event.baseline_metrics.get("success_rate", 0.0)
            candidate = event.metrics.get("success_rate", 0.0)
            print(f"  baseline success_rate={baseline:.2f}")
            print(f"  candidate success_rate={candidate:.2f}")
        return 0

    if sub == "list-scenarios":
        scenarios = list_scenarios(task_family=args.task_family)
        if args.json:
            print(json.dumps([s.to_dict() for s in scenarios], indent=2))
        else:
            print("Available Darwin stress scenarios:")
            for scenario in scenarios:
                print(f"  {scenario.scenario_id:<25} {scenario.name} ({scenario.difficulty})")
        return 0

    if sub == "history":
        engine = DarwinEngine(default_seeds=args.seeds, default_episodes=args.episodes)
        history = engine.list_history(task_id=args.task_id)
        if args.json:
            print(json.dumps([e.to_dict() for e in history], indent=2, default=str))
        else:
            print(f"Darwin benchmark history ({len(history)})")
            for event in history:
                print(
                    f"  {event.benchmark_id} {event.task_id}/{event.skill_id} "
                    f"regression={event.regression_detected}"
                )
        return 0

    print("Usage: rosclaw darwin {run,list-scenarios,history} ...")
    return 1
