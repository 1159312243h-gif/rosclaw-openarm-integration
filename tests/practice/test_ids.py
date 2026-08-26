"""Tests for canonical ID generators."""

from __future__ import annotations

import pytest

from rosclaw.practice.ids import (
    ID_PREFIXES,
    generate_artifact_id,
    generate_asset_id,
    generate_body_id,
    generate_candidate_id,
    generate_episode_id,
    generate_event_id,
    generate_policy_id,
    generate_practice_id,
    generate_session_id,
    generate_skill_id,
    generate_trace_id,
    is_id_of_kind,
)


@pytest.mark.parametrize(
    "generator,kind",
    [
        (generate_event_id, "event"),
        (generate_session_id, "session"),
        (generate_episode_id, "episode"),
        (generate_practice_id, "practice"),
        (generate_trace_id, "trace"),
        (lambda: generate_artifact_id("parquet"), "artifact"),
        (generate_candidate_id, "candidate"),
        (generate_policy_id, "policy"),
        (generate_asset_id, "asset"),
        (generate_skill_id, "skill"),
        (generate_body_id, "body"),
    ],
)
def test_id_prefix(generator, kind):
    value = generator()
    assert is_id_of_kind(value, kind)
    assert value.startswith(ID_PREFIXES[kind])


def test_event_id_is_unique():
    ids = {generate_event_id() for _ in range(200)}
    assert len(ids) == 200


def test_trace_id_longer_hash():
    tid = generate_trace_id()
    assert tid.startswith("trace_")
    # prefix 6 chars + 12 hex chars = 18 chars after underscore
    assert len(tid.split("_", 1)[1]) == 12


def test_artifact_id_includes_type():
    aid = generate_artifact_id("parquet")
    assert "parquet" in aid


def test_is_id_of_kind_rejects_unknown_kind():
    assert is_id_of_kind("evt_123", "unknown_kind") is False


def test_is_id_of_kind_rejects_non_string():
    assert is_id_of_kind(None, "event") is False  # type: ignore[arg-type]


def test_is_id_of_kind_false_when_prefix_mismatches():
    assert is_id_of_kind("sess_20240101T000000Z_abcdef", "event") is False
