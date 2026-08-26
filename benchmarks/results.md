# ROSClaw v1.0 Performance Benchmark Report

> **Date**: 2026-05-28
> **Method**: Single-process synthetic benchmark on local machine

## EventBus Throughput
- Target: 10,000 events
- Elapsed: 0.0462s
- Throughput: 216582.6 events/s
- Average latency: 4.6172 us/event
- Received: 10,000 / 10,000
- Verdict: PASS (needs >= 10,000 events/s)

## SeekDB Scale Performance
- Records inserted: 10,000
- Total insert time: 0.0238s
- Average insert: 0.0023 ms
- P99 insert: 0.0034 ms
- Query (robot_id=ur5e): 100 results in 4.2184 ms
- Verdict: PASS

## SkillRegistry Scale Performance
- Skills registered: 1,000
- Register time: 1.7475 ms (1.7475 us/skill)
- list_skills() [names]: 1,000 in 0.0336 ms
- list_skills(return_entries=True): 1,000 in 0.0068 ms
- list_skills(skill_type='programmed'): 500 in 0.0319 ms
- Verdict: PASS

## FirewallValidator Trajectory Performance
- Trajectory waypoints: 100
- Validation time: 0.0796 ms
- Is safe: True
- Violations: 0
- Warnings: 0
- Layers checked: ['eurdf_soft_limits', 'semantic_safety']
- Verdict: PASS
