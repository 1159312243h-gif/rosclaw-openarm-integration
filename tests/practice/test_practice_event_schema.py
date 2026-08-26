"""Tests for the extended PracticeEventEnvelope schema."""

from __future__ import annotations

from rosclaw.practice.ids import (
    generate_episode_id,
    generate_policy_id,
    generate_session_id,
    is_id_of_kind,
)
from rosclaw.practice.schemas import (
    BodyCognitionPayload,
    CandidatePolicyPayload,
    ContactEventPayload,
    EpisodeSummaryPayload,
    FailureEventPayload,
    HowInterventionPayload,
    PhysicalFeedbackPayload,
    PracticeEventEnvelope,
    PromotionResultPayload,
    Sim2RealDeltaPayload,
)


def test_envelope_has_canonical_event_id():
    env = PracticeEventEnvelope(
        practice_id="prac_001",
        robot_id="r1",
        source="agent",
        event_type="agent.task_received",
    )
    assert is_id_of_kind(env.event_id, "event")


def test_envelope_optional_body_episode_policy_ids():
    env = PracticeEventEnvelope(
        practice_id="prac_001",
        session_id=generate_session_id(),
        episode_id=generate_episode_id(),
        robot_id="r1",
        body_id="body_rh56_left",
        policy_id=generate_policy_id(),
        source="runtime",
        event_type="physical_feedback_event",
    )
    assert env.body_id == "body_rh56_left"
    assert is_id_of_kind(env.episode_id, "episode")
    assert is_id_of_kind(env.policy_id, "policy")


def test_envelope_roundtrip_with_body_fields():
    env = PracticeEventEnvelope(
        practice_id="prac_001",
        session_id=generate_session_id(),
        episode_id=generate_episode_id(),
        robot_id="r1",
        body_id="body_rh56_left",
        source="runtime",
        event_type="physical_feedback_event",
        payload={"primary_event": "desired_contact"},
    )
    data = env.model_dump(mode="json")
    restored = PracticeEventEnvelope.model_validate(data)
    assert restored.body_id == "body_rh56_left"
    assert restored.payload["primary_event"] == "desired_contact"


def test_physical_feedback_payload_roundtrip():
    payload = PhysicalFeedbackPayload(
        frame_id="frame_1",
        body_id="body_rh56_left",
        timestamp=1.0,
        actual={"thumb": 10.0, "index": 20.0},
        force_net={"thumb": 150.0, "index": 180.0},
        primary_event="desired_contact",
    )
    env = PracticeEventEnvelope(
        practice_id="prac_001",
        robot_id="r1",
        body_id="body_rh56_left",
        source="runtime",
        event_type="physical_feedback_event",
        payload=payload.model_dump(),
    )
    restored = PracticeEventEnvelope.model_validate(env.model_dump(mode="json"))
    assert restored.payload["primary_event"] == "desired_contact"
    assert restored.payload["force_net"]["thumb"] == 150.0


def test_contact_event_payload():
    payload = ContactEventPayload(
        contact_id="contact_1",
        event_type="desired_contact",
        dofs=["thumb", "index"],
        timestamp=1.0,
        force_net={"thumb": 150.0},
    )
    assert payload.event_type == "desired_contact"


def test_failure_event_payload():
    payload = FailureEventPayload(
        failure_id="fail_1",
        failure_type="over_contact",
        severity="high",
        source="sandbox",
        description="force exceeded safety limit",
    )
    assert payload.severity == "high"


def test_how_intervention_payload():
    payload = HowInterventionPayload(
        intervention_id="how_1",
        failure_id="fail_1",
        description="reduce target force by 20%",
        action_taken={"target_force_delta": -20.0},
        outcome="resolved",
    )
    assert payload.outcome == "resolved"


def test_candidate_policy_payload():
    payload = CandidatePolicyPayload(
        candidate_id="cand_1",
        policy_type="ok_contact_pose",
        policy_params={"thumb_angle": 650, "index_angle": 450},
    )
    assert payload.policy_params["thumb_angle"] == 650


def test_promotion_result_payload():
    payload = PromotionResultPayload(
        promotion_id="promo_1",
        candidate_id="cand_1",
        policy_id="pol_1",
        passed=True,
        gate_name="rh56_ok_contact_repeatability",
        metrics={"contact_rate": 0.95},
    )
    assert payload.passed is True


def test_episode_summary_payload():
    payload = EpisodeSummaryPayload(
        episode_id="ep_1",
        outcome="success",
        success=True,
        event_count=42,
        primary_event_distribution={"desired_contact": 10, "no_contact": 2},
    )
    assert payload.primary_event_distribution["desired_contact"] == 10


def test_body_cognition_payload():
    payload = BodyCognitionPayload(
        body_id="body_rh56_left",
        cognition_id="cog_1",
        cognition_type="force_model",
        data={"thumb_baseline": 50.0},
    )
    assert payload.cognition_type == "force_model"


def test_sim2real_delta_payload():
    payload = Sim2RealDeltaPayload(
        delta_id="delta_1",
        body_id="body_rh56_left",
        dofs=["thumb"],
        sim_value={"thumb": 650.0},
        real_value={"thumb": 992.0},
        delta={"thumb": 342.0},
        unit="angle_act",
    )
    assert payload.delta["thumb"] == 342.0
