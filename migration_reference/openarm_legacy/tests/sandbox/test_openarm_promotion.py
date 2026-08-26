"""Fail-closed checks for the OpenArm parallel-gripper formal promotion."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from scripts import promote_openarm_parallel_v3_3 as promotion


def _write(path: Path, data: bytes) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return hashlib.sha256(data).hexdigest()


def test_parse_hash_file_rejects_malformed_lines(tmp_path: Path) -> None:
    path = tmp_path / "model.sha256"
    path.write_text("not-a-checksum-line\n", encoding="ascii")

    with pytest.raises(ValueError, match="Invalid checksum line"):
        promotion.parse_hash_file(path)


def test_unchanged_snapshot_excludes_only_promoted_files(tmp_path: Path) -> None:
    body = tmp_path / "body"
    _write(body / "robot.urdf", b"new model")
    _write(body / "semantic.yaml", b"semantics")
    _write(body / "assets/mesh.stl", b"mesh")

    snapshot = promotion.unchanged_snapshot(body)

    assert "robot.urdf" not in snapshot
    assert snapshot["semantic.yaml"].startswith("sha256:")
    assert snapshot["assets"] == "directory"
    assert snapshot["assets/mesh.stl"].startswith("sha256:")


def test_validate_receipt_requires_frozen_hash_before_trusting_json(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    directory = tmp_path / "evidence"
    receipt = directory / "right_arm/receipt.json"
    _write(receipt, json.dumps({"action_id": "wrong"}).encode())
    (receipt.parent / "receipt.sha256").write_text(
        f"{promotion.sha256(receipt)}  receipt.json\n", encoding="ascii"
    )
    _write(receipt.parent / "moveit_worker_result.json", b"{}")
    monkeypatch.setitem(
        promotion.EXPECTED_RECEIPTS,
        "right_arm",
        {"action_id": "expected", "sha256": "0" * 64},
    )

    with pytest.raises(ValueError, match="SHA256 mismatch"):
        promotion.validate_receipt(directory, "right_arm")


def test_promotion_defaults_to_check_only() -> None:
    source = Path(promotion.__file__).read_text(encoding="utf-8")

    assert 'parser.add_argument("--apply", action="store_true")' in source
    assert '"mode": "check-only" if not args.apply else "apply"' in source
    assert "if not args.apply:" in source


def test_promotion_requires_atomic_directory_exchange_and_auto_restore() -> None:
    source = Path(promotion.__file__).read_text(encoding="utf-8")

    assert "renameat2" in source
    assert "rename_exchange(formal, staging)" in source
    assert "rename_exchange(formal_body, body_staging)" in source
    assert "rename_exchange(formal, backup)" in source
    assert "rename_exchange(formal_body, body_backup)" in source
    assert "refusing non-atomic promotion" in source


def test_promotion_is_bound_to_old_new_and_receipt_hashes() -> None:
    assert promotion.EXPECTED_FORMAL_HASHES == {
        "robot.urdf": "1ec020a66635cb5687ec57a51c87b8ed3c868d8b447370462e31c3c84e7d22a7",
        "robot.mjcf.xml": "788efe1683b6970ce61f2b68a44c223386014c73bb7e929cad4e14fea0e722c1",
    }
    assert promotion.EXPECTED_CANDIDATE_HASHES["robot.urdf"] == (
        "17ad3c2fb58f34d132f91a1a077d351f12153b6cef1964de3ef87e5fe68c7b12"
    )
    assert promotion.EXPECTED_CANDIDATE_HASHES["robot.mjcf.xml"] == (
        "ae35f356ad09bd68a30bb69e4419d540e6b9c1c011e061e04b1a448af3d4265e"
    )
    assert set(promotion.EXPECTED_RECEIPTS) == {"left_arm", "right_arm"}


def test_process_guard_covers_ros_moveit_mujoco_and_mcp() -> None:
    signals = "\n".join(promotion.PROCESS_SIGNALS)

    assert "mujoco_moveit_bridge" in signals
    assert "move_group" in signals
    assert "relative_motion_server" in signals
    assert "rosclaw.entrypoint mcp serve" in signals
    assert "openclaw/dist/index.js gateway" in signals


def test_promotion_validates_effective_body_and_records_both_backups() -> None:
    source = Path(promotion.__file__).read_text(encoding="utf-8")

    assert 'validate_effective_body(candidate_body, expected_type="prismatic")' in source
    assert 'validate_effective_body(formal_body, expected_type="revolute")' in source
    assert '"body_backup": str(body_backup)' in source


def test_body_profile_comparison_allows_derived_safety_fields(tmp_path: Path) -> None:
    body = tmp_path / "home/body"
    runtime_body = tmp_path / "models/openarm_dual_mujoco_01"
    joints = [{"name": "joint", "type": "fixed"}]
    joint_limits = {"finger": {"lower": 0.0, "upper": 0.044}}
    velocity_limits = {"finger": 20.0}
    profile = {"joints": joints}
    eurdf = {
        "joints": joints,
        "safety_limits": {
            "joint_limits": joint_limits,
            "velocity_limits": velocity_limits,
            "workspace": {"planning_frame": "world"},
        },
    }
    effective = {
        "safety": {
            "safety_limits": {
                "joint_limits": joint_limits,
                "velocity_limits": velocity_limits,
                "capability_limits": {"openarm.arm.move_relative": {}},
            }
        }
    }
    (body / "refs").mkdir(parents=True)
    runtime_body.mkdir(parents=True)
    (body / "refs/eurdf.profile.yaml").write_text(
        yaml.safe_dump(profile), encoding="utf-8"
    )
    (body / "refs/effective_body.json").write_text(
        json.dumps(effective), encoding="utf-8"
    )
    (runtime_body / "robot.eurdf.yaml").write_text(
        yaml.safe_dump(eurdf), encoding="ascii"
    )

    promotion.validate_body_profile_matches_runtime(body, runtime_body)


def test_body_profile_comparison_rejects_joint_limit_drift(tmp_path: Path) -> None:
    body = tmp_path / "home/body"
    runtime_body = tmp_path / "models/openarm_dual_mujoco_01"
    joints = [{"name": "joint", "type": "fixed"}]
    profile = {"joints": joints}
    eurdf = {
        "joints": joints,
        "safety_limits": {
            "joint_limits": {"finger": {"lower": 0.0, "upper": 0.044}},
            "velocity_limits": {"finger": 20.0},
        },
    }
    effective = {
        "safety": {
            "safety_limits": {
                "joint_limits": {"finger": {"lower": 0.0, "upper": 0.04}},
                "velocity_limits": {"finger": 20.0},
            }
        }
    }
    (body / "refs").mkdir(parents=True)
    runtime_body.mkdir(parents=True)
    (body / "refs/eurdf.profile.yaml").write_text(
        yaml.safe_dump(profile), encoding="utf-8"
    )
    (body / "refs/effective_body.json").write_text(
        json.dumps(effective), encoding="utf-8"
    )
    (runtime_body / "robot.eurdf.yaml").write_text(
        yaml.safe_dump(eurdf), encoding="ascii"
    )

    with pytest.raises(ValueError, match="joint_limits"):
        promotion.validate_body_profile_matches_runtime(body, runtime_body)
