"""Tests for the MCAP practice-event writer."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import pytest

from rosclaw.practice.writers.mcap_writer import McapWriter

pytestmark = pytest.mark.skipif(
    not McapWriter.is_available(), reason="mcap package is not installed"
)


def _read_records(path: Path) -> list[tuple[str, str | None, dict]]:
    """Return (topic, schema_name, payload) for every message in *path*."""
    from mcap.reader import make_reader

    with open(path, "rb") as f:
        reader = make_reader(f)
        return [
            (
                channel.topic,
                schema.name if schema is not None else None,
                json.loads(message.data),
            )
            for schema, channel, message in reader.iter_messages()
        ]


def test_write_and_read_roundtrip():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.mcap"
        writer = McapWriter(path)
        for i in range(3):
            writer.write(
                {
                    "practice_id": "prac_001",
                    "event_type": f"event.{i}",
                    "timestamp_ns": 1_000_000_000 + i,
                    "payload": {"index": i},
                }
            )
        writer.close()

        assert path.exists()
        records = _read_records(path)
        assert len(records) == 3
        for topic, schema_name, _payload in records:
            assert topic == "/rosclaw/events"
            assert schema_name == "rosclaw.practice.PracticeEventEnvelope"
        assert [r[2]["payload"]["index"] for r in records] == [0, 1, 2]


def test_context_manager_closes_file():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.mcap"
        with McapWriter(path) as writer:
            writer.write(
                {
                    "practice_id": "prac_002",
                    "event_type": "ping",
                    "timestamp_ns": 123,
                }
            )
        assert path.exists()
        records = _read_records(path)
        assert len(records) == 1
        assert records[0][2]["event_type"] == "ping"


@pytest.mark.parametrize("compression", ["zstd", "lz4", "none"])
def test_compression_variants(compression: str):
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.mcap"
        writer = McapWriter(path, compression=compression)
        writer.write(
            {
                "practice_id": "prac_003",
                "event_type": "compress.test",
                "timestamp_ns": 456,
                "payload": {"compression": compression},
            }
        )
        writer.close()

        records = _read_records(path)
        assert len(records) == 1
        assert records[0][2]["payload"]["compression"] == compression


def test_unknown_compression_falls_back_to_zstd():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.mcap"
        writer = McapWriter(path, compression="not-real")
        writer.write({"practice_id": "prac_004", "event_type": "fallback", "timestamp_ns": 789})
        writer.close()
        records = _read_records(path)
        assert len(records) == 1


def test_bad_record_is_dropped_without_crashing():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.mcap"
        writer = McapWriter(path)
        writer.write({"practice_id": "prac_005", "event_type": "ok", "timestamp_ns": 1})
        writer.write(object())  # type: ignore[arg-type]
        writer.write({"practice_id": "prac_005", "event_type": "ok", "timestamp_ns": 2})
        writer.close()

        records = _read_records(path)
        assert len(records) == 2
        assert records[0][2]["timestamp_ns"] == 1
        assert records[1][2]["timestamp_ns"] == 2


def test_close_is_idempotent():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.mcap"
        writer = McapWriter(path)
        writer.close()
        writer.close()  # should not raise
        assert path.exists()


def test_write_after_close_is_dropped():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.mcap"
        writer = McapWriter(path)
        writer.close()
        writer.write({"practice_id": "prac_006", "event_type": "late", "timestamp_ns": 1})
        records = _read_records(path)
        assert len(records) == 0
