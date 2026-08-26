"""Tests for PracticeRecorder v2 session/episode/artifact integration."""

from __future__ import annotations

import tempfile
from pathlib import Path

from rosclaw.practice.config import PracticeConfig, SourceConfig
from rosclaw.practice.coordinator import PracticeCoordinator
from rosclaw.practice.recorder import PracticeRecorder
from rosclaw.practice.storage.catalog import PracticeCatalog
from rosclaw.runtime.bus import RuntimeBus


def test_coordinator_generates_session_and_episode_ids():
    with tempfile.TemporaryDirectory() as tmp:
        cfg = PracticeConfig(
            robot_id="test_bot",
            task_name="pick cup",
            data_root=tmp,
            sources=SourceConfig(agent=True, runtime=True),
            mock=True,
            publish_to_event_bus=False,
        )
        coord = PracticeCoordinator(cfg)
        coord.initialize()
        coord.start()

        assert coord.session is not None
        assert coord.session.session_id is not None
        assert coord.session.session_id.startswith("sess_")
        assert coord.session.episode_id is not None
        assert coord.session.episode_id.startswith("ep_")

        coord.stop()


def test_recorder_writes_v2_episode_summary_on_stop():
    with tempfile.TemporaryDirectory() as tmp:
        bus = RuntimeBus()
        recorder = PracticeRecorder(bus, data_root=tmp, publish_to_event_bus=False)
        recorder.initialize()
        recorder.start()

        cfg = PracticeConfig(
            robot_id="test_bot",
            task_name="ok_contact",
            data_root=tmp,
            sources=SourceConfig(agent=True, runtime=True),
            mock=True,
            publish_to_event_bus=False,
        )
        coord = PracticeCoordinator(cfg, runtime_bus=bus, recorder=recorder)
        coord.initialize()
        coord.start()
        coord.stop()

        recorder.stop()

        summary = coord.summary
        assert summary is not None
        practice_id = summary.practice_id

        catalog = PracticeCatalog(Path(tmp) / "indexes" / "practice_catalog.sqlite")
        practice = catalog.get_practice(practice_id)
        assert practice["session_id"] is not None
        assert practice["episode_id"] is not None

        session = catalog.get_session(practice["session_id"])
        assert session is not None
        assert session["practice_id"] == practice_id
        assert session["status"] == "closed"

        episode = catalog.get_episode(practice["episode_id"])
        assert episode is not None
        assert episode["session_id"] == practice["session_id"]
        # Catalog v2 stores the canonical lowercase outcome; the coordinator summary
        # keeps the original uppercase token for backward compatibility.
        assert episode["outcome"] == summary.outcome.lower()

        # Summary YAML artifact exists under session_id directory
        summary_path = (
            Path(tmp)
            / "sessions"
            / practice["session_id"]
            / "episodes"
            / practice["episode_id"]
            / "artifacts"
            / "summary"
            / f"summary_{practice['episode_id']}.yaml"
        )
        assert summary_path.exists()

        catalog.close()
