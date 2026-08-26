"""Integration tests for MCAP output produced by PracticeRecorder."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import pytest

from rosclaw.practice.config import RecorderConfig
from rosclaw.practice.recorder import PracticeRecorder
from rosclaw.practice.storage.layout import PracticeLayout
from rosclaw.practice.writers.mcap_writer import McapWriter
from rosclaw.runtime import RuntimeBus
from rosclaw.runtime.event import RuntimeEvent

pytestmark = pytest.mark.skipif(
    not McapWriter.is_available(), reason="mcap package is not installed"
)


def _read_message_count(mcap_path: Path) -> int:
    from mcap.reader import make_reader

    with open(mcap_path, "rb") as f:
        return len(list(make_reader(f).iter_messages()))


def test_recorder_writes_mcap_when_enabled():
    with tempfile.TemporaryDirectory() as tmp:
        bus = RuntimeBus()
        config = RecorderConfig(
            mcap_enabled=True,
            mcap_compression="zstd",
            mcap_chunk_size_bytes=1024 * 1024,
        )
        recorder = PracticeRecorder(
            bus,
            data_root=tmp,
            config=config,
            publish_to_event_bus=False,
        )
        recorder.initialize()
        recorder.start()

        bus.publish(
            RuntimeEvent(
                type="practice.start",
                source="test",
                payload={
                    "practice_id": "prac_mcap_001",
                    "robot_id": "r1",
                    "task_id": "t1",
                },
            )
        )
        for i in range(3):
            bus.publish(
                RuntimeEvent(
                    type="agent.action",
                    source="agent",
                    payload={"iteration": i},
                )
            )
        bus.publish(
            RuntimeEvent(
                type="practice.stop",
                source="test",
                payload={"outcome": "SUCCESS", "duration_ms": 100.0},
            )
        )

        summary = recorder.summary
        recorder.stop()

        layout = PracticeLayout(tmp)
        jsonl_path = layout.events_jsonl_path("prac_mcap_001")
        mcap_path = layout.mcap_path("prac_mcap_001")

        assert jsonl_path.exists()
        assert mcap_path.exists()

        lines = jsonl_path.read_text(encoding="utf-8").strip().splitlines()
        assert summary is not None
        assert summary.event_count == len(lines)
        assert _read_message_count(mcap_path) == len(lines)

        # The episode summary should point at the MCAP file.
        episode = json.loads(layout.episode_json_path("prac_mcap_001").read_text())
        assert episode["artifacts"]["mcap"] == str(mcap_path)

        # The manifest should also report the MCAP artifact.
        manifest = layout.read_manifest("prac_mcap_001")
        assert manifest is not None
        assert Path(manifest["artifacts"]["mcap"]).resolve() == mcap_path.resolve()


def test_recorder_skips_mcap_when_disabled():
    with tempfile.TemporaryDirectory() as tmp:
        bus = RuntimeBus()
        config = RecorderConfig(mcap_enabled=False)
        recorder = PracticeRecorder(
            bus,
            data_root=tmp,
            config=config,
            publish_to_event_bus=False,
        )
        recorder.initialize()
        recorder.start()

        bus.publish(
            RuntimeEvent(
                type="practice.start",
                source="test",
                payload={
                    "practice_id": "prac_no_mcap",
                    "robot_id": "r1",
                },
            )
        )
        bus.publish(RuntimeEvent(type="agent.action", source="agent"))
        bus.publish(
            RuntimeEvent(
                type="practice.stop",
                source="test",
                payload={"outcome": "SUCCESS"},
            )
        )

        recorder.stop()

        layout = PracticeLayout(tmp)
        assert layout.events_jsonl_path("prac_no_mcap").exists()
        assert not layout.mcap_path("prac_no_mcap").exists()


def test_recorder_mcap_matches_jsonl_event_count():
    with tempfile.TemporaryDirectory() as tmp:
        bus = RuntimeBus()
        config = RecorderConfig(mcap_enabled=True)
        recorder = PracticeRecorder(
            bus,
            data_root=tmp,
            config=config,
            publish_to_event_bus=False,
        )
        recorder.initialize()
        recorder.start()

        bus.publish(
            RuntimeEvent(
                type="practice.start",
                source="test",
                payload={"practice_id": "prac_count", "robot_id": "r1"},
            )
        )
        for i in range(5):
            bus.publish(
                RuntimeEvent(
                    type="rps.round.resolved",
                    source="runtime",
                    payload={"round": i},
                )
            )
        bus.publish(
            RuntimeEvent(
                type="practice.stop",
                source="test",
                payload={"outcome": "SUCCESS"},
            )
        )

        recorder.stop()

        layout = PracticeLayout(tmp)
        jsonl_path = layout.events_jsonl_path("prac_count")
        mcap_path = layout.mcap_path("prac_count")
        jsonl_count = len(jsonl_path.read_text(encoding="utf-8").strip().splitlines())
        mcap_count = _read_message_count(mcap_path)
        assert jsonl_count == mcap_count == 5
