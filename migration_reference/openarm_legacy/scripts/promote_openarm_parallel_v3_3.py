#!/usr/bin/env python3
"""Fail-closed promotion of the accepted OpenArm parallel-gripper model."""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import yaml

if __package__:
    from scripts.sync_openarm_candidate_metadata import validate_metadata
else:
    from sync_openarm_candidate_metadata import validate_metadata


BODY_ID = "openarm_dual_mujoco_01"
EXPECTED_FORMAL_HASHES = {
    "robot.urdf": "1ec020a66635cb5687ec57a51c87b8ed3c868d8b447370462e31c3c84e7d22a7",
    "robot.mjcf.xml": "788efe1683b6970ce61f2b68a44c223386014c73bb7e929cad4e14fea0e722c1",
}
EXPECTED_CANDIDATE_HASHES = {
    "robot.urdf": "17ad3c2fb58f34d132f91a1a077d351f12153b6cef1964de3ef87e5fe68c7b12",
    "robot.mjcf.xml": "ae35f356ad09bd68a30bb69e4419d540e6b9c1c011e061e04b1a448af3d4265e",
    "robot.eurdf.yaml": "0d4895fbe9bb25ea5baf8646e90eb1c9faf767adf8d5a2b43209f5cde7177e55",
    "safety.yaml": "c1738cb5398f7e3fc1bdff2c5ba7ee07ab0e434f0ce807585788b44079ca1db6",
}
EXPECTED_RECEIPTS = {
    "right_arm": {
        "action_id": "action_8868a11d-e101-41b1-a41e-94f6fddb8063",
        "sha256": "572d4af2b0acafaa435b0c0a7f2bac38f812cbc75c1b95bd079912dc1dbeda9a",
    },
    "left_arm": {
        "action_id": "action_918a0192-1a13-46ad-a419-32af2d41fa12",
        "sha256": "c05fe5f95143e26466f4b022f23c041b2678ff6f3da329c131597642086e2b0d",
    },
}
UNCHANGED_FILES = ("benchmark.yaml", "capabilities.yaml", "semantic.yaml")
PROMOTED_FILES = (
    "robot.urdf",
    "robot.mjcf.xml",
    "robot.eurdf.yaml",
    "safety.yaml",
    "model.sha256",
)
PROCESS_SIGNALS = (
    "mujoco_moveit_bridge",
    "move_group",
    "relative_motion_server",
    "openarm_moveit_gripper_candidate.launch.py",
    "openarm_sim.launch.py",
    "rosclaw.entrypoint mcp serve",
    "openclaw/dist/index.js gateway",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_file(path: Path) -> None:
    if not path.is_file():
        raise ValueError(f"Required file is missing: {path}")


def require_hash(path: Path, expected: str) -> None:
    require_file(path)
    actual = sha256(path)
    if actual != expected:
        raise ValueError(f"SHA256 mismatch for {path}: {actual}, expected {expected}")


def parse_hash_file(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="ascii").splitlines():
        parts = line.split()
        if len(parts) != 2:
            raise ValueError(f"Invalid checksum line in {path}: {line!r}")
        result[parts[1]] = parts[0]
    return result


def validate_model_hash_file(body: Path, expected: dict[str, str]) -> None:
    checksums = parse_hash_file(body / "model.sha256")
    if checksums != {name: expected[name] for name in ("robot.urdf", "robot.mjcf.xml")}:
        raise ValueError(f"Unexpected model.sha256 content in {body}")
    for name, digest in checksums.items():
        require_hash(body / name, digest)


def validate_receipt(directory: Path, label: str) -> dict[str, Any]:
    expected = EXPECTED_RECEIPTS[label]
    receipt_path = directory / label / "receipt.json"
    digest_path = directory / label / "receipt.sha256"
    worker_path = directory / label / "moveit_worker_result.json"
    require_hash(receipt_path, expected["sha256"])
    require_file(digest_path)
    require_file(worker_path)

    digest_parts = digest_path.read_text(encoding="ascii").strip().split()
    if digest_parts != [expected["sha256"], "receipt.json"]:
        raise ValueError(f"Invalid receipt.sha256 for {label}")

    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    required_values = {
        "action_id": expected["action_id"],
        "body_id": BODY_ID,
        "capability_id": "openarm.arm.move_relative",
        "final_state": "COMPLETED",
        "evidence_level": "TASK_VERIFIED",
        "acknowledgement_stage": "TASK_VERIFIED",
    }
    for key, value in required_values.items():
        if receipt.get(key) != value:
            raise ValueError(f"Receipt {label} {key} is {receipt.get(key)!r}, expected {value!r}")
    if receipt.get("mode", receipt.get("execution_mode")) != "SIMULATION":
        raise ValueError(f"Receipt {label} is not SIMULATION")
    if receipt.get("verification_result", {}).get("passed") is not True:
        raise ValueError(f"Receipt {label} verification did not pass")
    simulation = receipt.get("simulation_result", {})
    required_simulation = {
        "backend": "mujoco_moveit",
        "planning_backend": "moveit2_get_cartesian_path",
        "execution_backend": "moveit2_execute_trajectory",
        "controller_backend": "follow_joint_trajectory",
        "physics_executed": True,
        "direct_mujoco_fallback": False,
    }
    for key, value in required_simulation.items():
        if simulation.get(key) != value:
            raise ValueError(
                f"Receipt {label} simulation_result.{key} is {simulation.get(key)!r}"
            )
    if float(receipt.get("final_error_m", 1.0)) > 0.0005:
        raise ValueError(f"Receipt {label} final error exceeds 0.5 mm")
    worker_digest = simulation.get("worker_artifact_sha256")
    if not isinstance(worker_digest, str) or sha256(worker_path) != worker_digest:
        raise ValueError(f"Receipt {label} worker artifact hash mismatch")
    return receipt


def live_processes(project: Path, formal: Path) -> list[dict[str, Any]]:
    matches: list[dict[str, Any]] = []
    proc = Path("/proc")
    if not proc.is_dir():
        raise RuntimeError("Process guard requires Linux /proc")
    own_pid = os.getpid()
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if not command:
            continue
        if str(formal) in command or any(signal in command for signal in PROCESS_SIGNALS):
            matches.append({"pid": int(entry.name), "command": command.strip()})
    return sorted(matches, key=lambda item: item["pid"])


def rename_exchange(left: Path, right: Path) -> None:
    if left.parent.resolve() != right.parent.resolve():
        raise ValueError("Atomic exchange paths must share one parent directory")
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise RuntimeError("Linux renameat2 is unavailable; refusing non-atomic promotion")
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    at_fdcwd = -100
    rename_exchange_flag = 2
    result = renameat2(
        at_fdcwd,
        os.fsencode(left),
        at_fdcwd,
        os.fsencode(right),
        rename_exchange_flag,
    )
    if result != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), f"{left} <-> {right}")


def validate_candidate(runtime_body: Path) -> None:
    for name, digest in EXPECTED_CANDIDATE_HASHES.items():
        require_hash(runtime_body / name, digest)
    validate_model_hash_file(runtime_body, EXPECTED_CANDIDATE_HASHES)
    validate_metadata(
        runtime_body / "robot.urdf",
        runtime_body / "robot.eurdf.yaml",
        runtime_body / "safety.yaml",
    )


def validate_effective_body(body: Path, *, expected_type: str) -> None:
    effective_path = body / "refs" / "effective_body.json"
    require_file(effective_path)
    effective = json.loads(effective_path.read_text(encoding="utf-8"))
    if effective.get("body_instance_id") != BODY_ID:
        raise ValueError(f"Effective Body ID mismatch in {effective_path}")
    joints = effective.get("joints", {})
    for side in ("left", "right"):
        for number, axis in ((1, [0.0, 1.0, 0.0]), (2, [0.0, -1.0, 0.0])):
            name = f"openarm_{side}_finger_joint{number}"
            joint = joints.get(name, {})
            if joint.get("type") != expected_type:
                raise ValueError(
                    f"Effective Body {name} type is {joint.get('type')}, expected {expected_type}"
                )
            if expected_type == "prismatic":
                if joint.get("axis") != axis:
                    raise ValueError(f"Effective Body {name} axis mismatch")
                limits = joint.get("limits", {})
                if limits.get("lower") != 0.0 or limits.get("upper") != 0.044:
                    raise ValueError(f"Effective Body {name} limits mismatch")
                expected_mimic = (
                    None
                    if number == 1
                    else {
                        "joint": f"openarm_{side}_finger_joint1",
                        "multiplier": 1.0,
                        "offset": 0.0,
                    }
                )
                if joint.get("mimic") != expected_mimic:
                    raise ValueError(f"Effective Body {name} mimic mismatch")

    if expected_type == "prismatic":
        safety = effective.get("safety", {}).get("safety_limits", {})
        positions = safety.get("joint_limits", {})
        velocities = safety.get("velocity_limits", {})
        for name in EXPECTED_FINGER_JOINTS:
            if positions.get(name) != {"lower": 0.0, "upper": 0.044}:
                raise ValueError(f"Effective Body safety position limits mismatch for {name}")
            if float(velocities.get(name, 0.0)) <= 0.0:
                raise ValueError(f"Effective Body safety velocity limit missing for {name}")


EXPECTED_FINGER_JOINTS = {
    f"openarm_{side}_finger_joint{number}"
    for side in ("left", "right")
    for number in (1, 2)
}


def validate_body_profile_matches_runtime(candidate_body: Path, runtime_body: Path) -> None:
    profile_path = candidate_body / "refs" / "eurdf.profile.yaml"
    require_file(profile_path)
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
    runtime_eurdf = yaml.safe_load(
        (runtime_body / "robot.eurdf.yaml").read_text(encoding="ascii")
    ) or {}
    if profile.get("joints") != runtime_eurdf.get("joints"):
        raise ValueError("Candidate Effective Body joints do not match runtime e-URDF")
    effective = json.loads(
        (candidate_body / "refs" / "effective_body.json").read_text(encoding="utf-8")
    )
    effective_safety = effective.get("safety", {}).get("safety_limits", {})
    runtime_safety = runtime_eurdf.get("safety_limits", {})
    for key in ("joint_limits", "velocity_limits"):
        if effective_safety.get(key) != runtime_safety.get(key):
            raise ValueError(
                f"Candidate Effective Body {key} do not match runtime e-URDF"
            )


def prepare_body_staging(
    candidate_body: Path, staging: Path, formal_model_parent: Path
) -> None:
    if staging.exists():
        raise FileExistsError(f"Refusing to reuse Body staging path: {staging}")
    shutil.copytree(candidate_body, staging, symlinks=True)
    lock_path = staging / "refs" / "eurdf.lock"
    lock = yaml.safe_load(lock_path.read_text(encoding="utf-8")) or {}
    lock["zoo_path"] = str(formal_model_parent)
    lock_path.write_text(
        yaml.safe_dump(lock, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    validate_effective_body(staging, expected_type="prismatic")


def unchanged_snapshot(body: Path) -> dict[str, str]:
    snapshot: dict[str, str] = {}
    for path in sorted(body.rglob("*")):
        relative = path.relative_to(body).as_posix()
        if relative in PROMOTED_FILES:
            continue
        if path.is_symlink():
            snapshot[relative] = f"symlink:{os.readlink(path)}"
        elif path.is_file():
            snapshot[relative] = f"sha256:{sha256(path)}"
        elif path.is_dir():
            snapshot[relative] = "directory"
    return snapshot


def validate_formal_before(formal: Path) -> dict[str, str]:
    for name, digest in EXPECTED_FORMAL_HASHES.items():
        require_hash(formal / name, digest)
    validate_model_hash_file(formal, EXPECTED_FORMAL_HASHES)
    for name in (*UNCHANGED_FILES, "assets"):
        if not (formal / name).exists():
            raise ValueError(f"Formal Body content is missing: {formal / name}")
    return unchanged_snapshot(formal)


def validate_promoted(formal: Path, unchanged: dict[str, str]) -> None:
    validate_candidate(formal)
    actual = unchanged_snapshot(formal)
    if actual != unchanged:
        missing = sorted(set(unchanged) - set(actual))
        added = sorted(set(actual) - set(unchanged))
        changed = sorted(
            name for name in set(actual) & set(unchanged) if actual[name] != unchanged[name]
        )
        raise ValueError(
            f"Unchanged Body content differs: missing={missing}, added={added}, changed={changed}"
        )


def build_staging(formal: Path, runtime_body: Path, staging: Path) -> None:
    if staging.exists():
        raise FileExistsError(f"Refusing to reuse staging path: {staging}")
    shutil.copytree(formal, staging, symlinks=True)
    for name in PROMOTED_FILES:
        shutil.copy2(runtime_body / name, staging / name)


def write_record(project: Path, payload: dict[str, Any]) -> Path:
    record_dir = project / ".rosclaw" / "promotions"
    record_dir.mkdir(parents=True, exist_ok=True)
    record = record_dir / "openarm_parallel_v3_3_latest.json"
    temporary = record.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="ascii")
    os.replace(temporary, record)
    return record


def promote(args: argparse.Namespace) -> dict[str, Any]:
    project = args.project.resolve()
    formal = project / "models" / BODY_ID
    formal_body = project / ".rosclaw" / "body"
    runtime = args.runtime.resolve()
    evidence = args.evidence.resolve()
    profile = json.loads((runtime / "provenance.json").read_text(encoding="ascii"))
    if profile.get("body_id") != BODY_ID:
        raise ValueError("Candidate provenance Body ID mismatch")
    if profile.get("candidate_hashes") != {
        name: EXPECTED_CANDIDATE_HASHES[name]
        for name in ("robot.urdf", "robot.mjcf.xml")
    }:
        raise ValueError("Candidate provenance model hashes mismatch")
    if profile.get("runtime_metadata_hashes") != {
        name: EXPECTED_CANDIDATE_HASHES[name]
        for name in ("robot.eurdf.yaml", "safety.yaml")
    }:
        raise ValueError("Candidate provenance metadata hashes mismatch")
    if profile.get("metadata_synchronization", {}).get("status") != "PASS":
        raise ValueError("Candidate provenance metadata synchronization did not pass")
    runtime_body = runtime / "models" / BODY_ID
    candidate_body = runtime / "home" / "body"

    unchanged = validate_formal_before(formal)
    validate_candidate(runtime_body)
    validate_effective_body(candidate_body, expected_type="prismatic")
    validate_body_profile_matches_runtime(candidate_body, runtime_body)
    validate_effective_body(formal_body, expected_type="revolute")
    receipts = {
        label: validate_receipt(evidence, label) for label in EXPECTED_RECEIPTS
    }
    blockers = live_processes(project, formal)
    result: dict[str, Any] = {
        "status": "PASS" if not blockers else "BLOCKED",
        "mode": "check-only" if not args.apply else "apply",
        "formal": str(formal),
        "formal_body": str(formal_body),
        "runtime": str(runtime),
        "evidence": str(evidence),
        "formal_hashes_before": EXPECTED_FORMAL_HASHES,
        "candidate_hashes": EXPECTED_CANDIDATE_HASHES,
        "receipt_action_ids": {
            label: receipt["action_id"] for label, receipt in receipts.items()
        },
        "live_processes": blockers,
        "simulation_only": True,
        "hardware_readiness_claimed": False,
    }
    if blockers:
        if args.apply:
            raise RuntimeError(f"Promotion blocked by live processes: {blockers}")
        return result
    if not args.apply:
        return result

    stamp = time.strftime("%Y%m%d_%H%M%S")
    staging = formal.parent / f".{BODY_ID}.promotion_staging_{stamp}"
    backup = formal.parent / f"{BODY_ID}.before_parallel_v3_3_{stamp}"
    body_staging = formal_body.parent / f".body.promotion_staging_{stamp}"
    body_backup = formal_body.parent / f"body.before_parallel_v3_3_{stamp}"
    build_staging(formal, runtime_body, staging)
    validate_promoted(staging, unchanged)
    prepare_body_staging(candidate_body, body_staging, formal.parent)

    model_exchanged = False
    body_exchanged = False
    model_backup_created = False
    body_backup_created = False
    try:
        rename_exchange(formal, staging)
        model_exchanged = True
        rename_exchange(formal_body, body_staging)
        body_exchanged = True
        validate_promoted(formal, unchanged)
        validate_effective_body(formal_body, expected_type="prismatic")
        os.rename(staging, backup)
        model_backup_created = True
        os.rename(body_staging, body_backup)
        body_backup_created = True

        record_payload = {
            **result,
            "status": "PASS",
            "backup": str(backup),
            "body_backup": str(body_backup),
            "promoted_at": stamp,
            "formal_hashes_after": EXPECTED_CANDIDATE_HASHES,
        }
        record = write_record(project, record_payload)
    except Exception:
        if body_backup_created:
            rename_exchange(formal_body, body_backup)
        elif body_exchanged:
            rename_exchange(formal_body, body_staging)
        if model_backup_created:
            rename_exchange(formal, backup)
        elif model_exchanged:
            rename_exchange(formal, staging)
        raise
    record_payload["record"] = str(record)
    return record_payload


def rollback(args: argparse.Namespace) -> dict[str, Any]:
    project = args.project.resolve()
    formal = project / "models" / BODY_ID
    formal_body = project / ".rosclaw" / "body"
    record = args.record.resolve()
    data = json.loads(record.read_text(encoding="ascii"))
    backup = Path(data["backup"]).resolve()
    body_backup = Path(data["body_backup"]).resolve()
    if backup.parent != formal.parent:
        raise ValueError("Rollback backup is outside the formal model parent")
    if body_backup.parent != formal_body.parent:
        raise ValueError("Rollback Body backup is outside the formal workspace")
    validate_candidate(formal)
    validate_model_hash_file(backup, EXPECTED_FORMAL_HASHES)
    validate_effective_body(formal_body, expected_type="prismatic")
    validate_effective_body(body_backup, expected_type="revolute")
    blockers = live_processes(project, formal)
    if blockers:
        raise RuntimeError(f"Rollback blocked by live processes: {blockers}")
    if not args.apply:
        return {
            "status": "PASS",
            "mode": "rollback-check-only",
            "formal": str(formal),
            "backup": str(backup),
            "body_backup": str(body_backup),
            "live_processes": [],
        }
    model_exchanged = False
    body_exchanged = False
    try:
        rename_exchange(formal, backup)
        model_exchanged = True
        rename_exchange(formal_body, body_backup)
        body_exchanged = True
        validate_model_hash_file(formal, EXPECTED_FORMAL_HASHES)
        validate_effective_body(formal_body, expected_type="revolute")
    except Exception:
        if body_exchanged:
            rename_exchange(formal_body, body_backup)
        if model_exchanged:
            rename_exchange(formal, backup)
        raise
    return {
        "status": "PASS",
        "mode": "rollback-applied",
        "formal": str(formal),
        "promoted_copy": str(backup),
        "promoted_body_copy": str(body_backup),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--runtime", type=Path)
    parser.add_argument("--evidence", type=Path)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--rollback", action="store_true")
    args = parser.parse_args()
    try:
        if args.rollback:
            if args.record is None:
                parser.error("--rollback requires --record")
            result = rollback(args)
        else:
            if args.runtime is None or args.evidence is None:
                parser.error("promotion requires --runtime and --evidence")
            result = promote(args)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"] == "PASS" else 2
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}, indent=2), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
