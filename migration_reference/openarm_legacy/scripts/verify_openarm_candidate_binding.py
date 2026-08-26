#!/usr/bin/env python3
"""Fail closed unless ROSClaw, MoveIt and MuJoCo all reference one candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(
    candidate: Path,
    runtime: Path,
    active_mjcf: Path,
    active_urdf: Path,
) -> dict[str, object]:
    candidate = candidate.resolve()
    runtime = runtime.resolve()
    profile = yaml.safe_load(runtime.read_text(encoding="ascii"))
    body_id = profile["robot"]["id"]
    runtime_body = Path(profile["robot"]["zoo_path"]).resolve() / body_id
    expected_urdf = candidate / "robot.urdf"
    expected_mjcf = candidate / "robot.mjcf.xml"
    checks = {
        "candidate_urdf_matches_runtime": sha256(expected_urdf)
        == sha256(runtime_body / "robot.urdf"),
        "candidate_mjcf_matches_runtime": sha256(expected_mjcf)
        == sha256(runtime_body / "robot.mjcf.xml"),
        "candidate_urdf_is_active": expected_urdf.resolve() == active_urdf.resolve(),
        "candidate_mjcf_is_active": expected_mjcf.resolve() == active_mjcf.resolve(),
    }
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "candidate_mjcf_sha256": sha256(expected_mjcf),
        "runtime_mjcf_sha256": sha256(runtime_body / "robot.mjcf.xml"),
        "runtime_profile": str(runtime),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--active-mjcf", type=Path, required=True)
    parser.add_argument("--active-urdf", type=Path, required=True)
    args = parser.parse_args()

    result = verify(
        args.candidate,
        args.runtime,
        args.active_mjcf,
        args.active_urdf,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
