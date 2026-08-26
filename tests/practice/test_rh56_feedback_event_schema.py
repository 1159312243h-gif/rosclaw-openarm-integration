"""RH56-specific tests for the generic feedback/contact event schema.

These tests verify that RH56 telemetry registers can be mapped into the
body-agnostic PracticeEventEnvelope without leaking RH56-specific types into
rosclaw core.
"""

from __future__ import annotations

from rosclaw.body.contact_event import CONTACT_EVENT_LABELS, PRIMARY_EVENT_PRIORITY
from rosclaw.practice.schemas import (
    ContactEventPayload,
    FailureEventPayload,
    PhysicalFeedbackPayload,
    PracticeEventEnvelope,
)


def test_rh56_registers_map_to_physical_feedback_payload():
    """Map RH56 ANGLE_ACT / FORCE_ACT / CURRENT / STATUS / ERROR / TEMP registers."""
    payload = PhysicalFeedbackPayload(
        frame_id="rh56_frame_1",
        body_id="body_rh56_left",
        timestamp=1.0,
        target={
            "thumb": 650.0,
            "index": 450.0,
            "middle": 1000.0,
            "ring": 1000.0,
            "pinky": 1000.0,
        },
        actual={
            "thumb": 992.0,
            "index": 995.0,
            "middle": 1000.0,
            "ring": 1000.0,
            "pinky": 1000.0,
        },
        position_error={
            "thumb": -342.0,
            "index": -545.0,
        },
        force_raw={"thumb": 150.0, "index": 180.0},
        force_baseline={"thumb": 50.0, "index": 60.0},
        force_net={"thumb": 100.0, "index": 120.0},
        current={"thumb": 120.0, "index": 130.0},
        status={"thumb": 0, "index": 0},
        error={"thumb": 0, "index": 0},
        temperature={"thumb": 35.0, "index": 36.0},
        primary_event="desired_contact",
        secondary_tags=["ok_gesture"],
    )
    assert payload.body_id == "body_rh56_left"
    assert payload.position_error["thumb"] == -342.0


def test_rh56_contact_event_payload():
    payload = ContactEventPayload(
        contact_id="contact_rh56_1",
        event_type="desired_contact",
        dofs=["thumb", "index"],
        timestamp=1.0,
        force_net={"thumb": 100.0, "index": 120.0},
        metadata={"ok_shape_score": 5, "asymmetric_search_step": 12},
    )
    assert payload.metadata["ok_shape_score"] == 5


def test_rh56_failure_event_payload_from_sandbox():
    payload = FailureEventPayload(
        failure_id="fail_rh56_over_contact_1",
        failure_type="over_contact",
        severity="high",
        source="sandbox",
        description="thumb force_net 320g exceeded safety threshold 250g",
        related_contact_id="contact_rh56_1",
        suggested_fix={"action": "backoff_force", "amount_g": 50.0},
    )
    env = PracticeEventEnvelope(
        practice_id="prac_rh56_001",
        robot_id="rh56_left",
        body_id="body_rh56_left",
        source="sandbox",
        event_type="failure_event",
        payload=payload.model_dump(),
    )
    assert env.payload["failure_type"] == "over_contact"


def test_primary_event_is_one_of_canonical_labels():
    payload = PhysicalFeedbackPayload(
        frame_id="f1",
        body_id="body_rh56_left",
        timestamp=1.0,
        primary_event="no_contact",
    )
    assert payload.primary_event in CONTACT_EVENT_LABELS


def test_contact_event_priority_order_has_unknown_last():
    assert PRIMARY_EVENT_PRIORITY[-1] == "unknown"
    assert "no_contact" in PRIMARY_EVENT_PRIORITY
    assert "desired_contact" in PRIMARY_EVENT_PRIORITY


def test_rh56_weak_index_contact_maps_to_failure_and_how():
    """A common RH56 failure mode: index contact too weak."""
    contact = ContactEventPayload(
        contact_id="contact_rh56_weak_index",
        event_type="weak_contact",
        dofs=["index"],
        timestamp=1.0,
        force_net={"index": 30.0},
    )
    failure = FailureEventPayload(
        failure_id="fail_weak_index",
        failure_type="weak_index_contact",
        severity="medium",
        source="runtime",
        description="index force_net 30g below desired 80g",
        related_contact_id=contact.contact_id,
    )
    assert failure.related_contact_id == contact.contact_id


def test_rh56_temperature_limited_event():
    payload = PhysicalFeedbackPayload(
        frame_id="f1",
        body_id="body_rh56_left",
        timestamp=1.0,
        temperature={"thumb": 52.0},
        primary_event="temperature_limited",
    )
    assert payload.primary_event == "temperature_limited"
