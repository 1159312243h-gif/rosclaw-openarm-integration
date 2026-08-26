"""Verify practice validate and show semantics for RealSense episodes."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from rosclaw.cli import cmd_practice_run, cmd_practice_show, cmd_practice_validate
from rosclaw.firstboot.workspace import init_workspace


def _fake_execute(self, skill_name: str, parameters: dict | None = None):
    """SkillExecutor.execute replacement that writes a fake color frame."""
    output_dir = Path(parameters.get("output_dir", "/tmp"))
    output_dir.mkdir(parents=True, exist_ok=True)
    color_path = output_dir / "color.png"
    color_path.write_bytes(b"\x89PNG\r\n\x1a\n" + b"\x00" * 20)
    return {
        "status": "success",
        "handler_result": {
            "artifacts": {"color": str(color_path)},
            "metrics": {"latency_ms": 12.3},
        },
    }


@pytest.fixture
def practice_home(tmp_path, monkeypatch):
    home = tmp_path / "rosclaw_home"
    monkeypatch.setenv("ROSCLAW_HOME", str(home))
    init_workspace(home)
    from rosclaw.body.service import BodyInstanceService

    service = BodyInstanceService(workspace=home)
    service.create_or_init(
        robot="realsense-d405",
        name="d405_lab_01",
        nickname="d405_lab_01",
        mode="single",
        update_registry=True,
        switch_active=True,
        render_agent_view=True,
        force=True,
    )
    return home


def _run_episode(practice_home: Path) -> Path:
    args = SimpleNamespace(
        robot="d405_lab_01",
        robot_type=None,
        task=None,
        skill="realsense_capture_rgbd",
        provider=None,
        capability="vlm.risk_assessment",
        output_root=str(practice_home / "practice_output"),
        data_root=None,
        workspace=str(practice_home),
        json=False,
    )
    assert cmd_practice_run(args) == 0
    return next((practice_home / "practice_output" / "sessions").iterdir())


def test_practice_validate_real_realsense_episode(practice_home, monkeypatch):
    monkeypatch.setattr(
        "rosclaw.skill_manager.executor.SkillExecutor.execute",
        _fake_execute,
    )
    session_dir = _run_episode(practice_home)

    validate_args = SimpleNamespace(
        episode_id=session_dir.name,
        data_root=str(practice_home / "practice_output"),
        strict=False,
        json=False,
    )
    assert cmd_practice_validate(validate_args) == 0


def test_practice_validate_strict_requires_all_sources(practice_home, monkeypatch):
    monkeypatch.setattr(
        "rosclaw.skill_manager.executor.SkillExecutor.execute",
        _fake_execute,
    )
    session_dir = _run_episode(practice_home)

    validate_args = SimpleNamespace(
        episode_id=session_dir.name,
        data_root=str(practice_home / "practice_output"),
        strict=True,
        json=False,
    )
    # No provider was requested, so strict validation should fail.
    assert cmd_practice_validate(validate_args) == 1


def test_practice_show_realsense_episode_summary(practice_home, monkeypatch, capsys):
    monkeypatch.setattr(
        "rosclaw.skill_manager.executor.SkillExecutor.execute",
        _fake_execute,
    )
    session_dir = _run_episode(practice_home)

    show_args = SimpleNamespace(
        episode_id=session_dir.name,
        data_root=str(practice_home / "practice_output"),
        json=False,
    )
    assert cmd_practice_show(show_args) == 0

    captured = capsys.readouterr().out
    assert "Episode:" in captured
    assert "Camera frames:" in captured
    assert "color:" in captured


def test_practice_show_json_outputs_episode(practice_home, monkeypatch, capsys):
    monkeypatch.setattr(
        "rosclaw.skill_manager.executor.SkillExecutor.execute",
        _fake_execute,
    )
    session_dir = _run_episode(practice_home)

    # Flush stdout from the run command so the JSON output is isolated.
    capsys.readouterr()

    show_args = SimpleNamespace(
        episode_id=session_dir.name,
        data_root=str(practice_home / "practice_output"),
        json=True,
    )
    assert cmd_practice_show(show_args) == 0

    report = json.loads(capsys.readouterr().out)
    assert report["practice_id"] == session_dir.name
    assert report["outcome"] == "SUCCESS"
