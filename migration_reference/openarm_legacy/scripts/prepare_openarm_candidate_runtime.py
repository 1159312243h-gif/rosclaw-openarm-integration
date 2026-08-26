#!/usr/bin/env python3
"""Prepare an isolated ROSClaw model zoo and profile for an OpenArm candidate."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import yaml

if __package__:
    from scripts.sync_openarm_candidate_metadata import synchronize
else:
    from sync_openarm_candidate_metadata import synchronize


BODY_ID = "openarm_dual_mujoco_01"
METADATA_FILES = (
    "benchmark.yaml",
    "capabilities.yaml",
    "robot.eurdf.yaml",
    "safety.yaml",
    "semantic.yaml",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(project: Path, candidate: Path, output: Path) -> dict[str, object]:
    project = project.resolve()
    candidate = candidate.resolve()
    output = output.resolve()
    source_body = project / "models" / BODY_ID
    source_urdf = candidate / "robot.urdf"
    source_mjcf = candidate / "robot.mjcf.xml"

    for path in (source_urdf, source_mjcf):
        if not path.is_file():
            raise FileNotFoundError(f"Candidate model file is missing: {path}")
    for name in METADATA_FILES:
        if not (source_body / name).is_file():
            raise FileNotFoundError(f"Formal Body metadata is missing: {source_body / name}")

    body_dir = output / "models" / BODY_ID
    body_dir.mkdir(parents=True, exist_ok=True)
    for name in METADATA_FILES:
        shutil.copy2(source_body / name, body_dir / name)
    shutil.copy2(source_urdf, body_dir / "robot.urdf")
    shutil.copy2(source_mjcf, body_dir / "robot.mjcf.xml")

    metadata_result = synchronize(
        body_dir / "robot.urdf",
        body_dir / "robot.eurdf.yaml",
        body_dir / "safety.yaml",
    )

    hashes = {
        "robot.urdf": sha256(body_dir / "robot.urdf"),
        "robot.mjcf.xml": sha256(body_dir / "robot.mjcf.xml"),
    }
    metadata_hashes = {
        name: sha256(body_dir / name) for name in ("robot.eurdf.yaml", "safety.yaml")
    }
    (body_dir / "model.sha256").write_text(
        "".join(f"{digest}  {name}\n" for name, digest in hashes.items()),
        encoding="ascii",
    )

    profile = {
        "schema_version": "rosclaw.runtime.profile.v1",
        "robot": {"id": BODY_ID, "zoo_path": str(output / "models")},
        "workspace": {"home": str(output / "home")},
        "sandbox": {
            "engine": "mujoco",
            "world_id": "empty",
            "artifact_root": str(output / "artifacts" / "sandbox"),
            "moveit": {
                "required": True,
                "worker_command": str(project / "scripts" / "run_openarm_moveit_worker.sh"),
                "action_name": "/rosclaw/arm_motion",
                "server_timeout_sec": 5.0,
                "result_timeout_sec": 30.0,
                "cancel_timeout_sec": 3.0,
            },
        },
        "skills": {"paths": [str(project / "skills" / "openarm-move-relative")]},
        "sense": {"collector": "mock", "update_hz": 1.0},
        "mcp": {"transport": "stdio"},
    }
    profile_path = output / "runtime.yaml"
    profile_path.write_text(
        yaml.safe_dump(profile, sort_keys=False, allow_unicode=False),
        encoding="ascii",
    )

    provenance = {
        "schema_version": "openarm.candidate.runtime.v1",
        "body_id": BODY_ID,
        "candidate_source": str(candidate),
        "candidate_urdf": str(source_urdf),
        "candidate_mjcf": str(source_mjcf),
        "candidate_hashes": hashes,
        "runtime_metadata_hashes": metadata_hashes,
        "metadata_synchronization": metadata_result,
        "runtime_profile": str(profile_path),
        "runtime_model": str(body_dir / "robot.mjcf.xml"),
        "formal_model_hash": sha256(source_body / "robot.mjcf.xml"),
    }
    (output / "provenance.json").write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n",
        encoding="ascii",
    )
    return provenance


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.project, args.candidate, args.output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
