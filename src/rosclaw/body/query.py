"""Body query engine — answer questions from body state without LLM dependency."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from rosclaw.body.schema import BodyYaml, CalibrationYaml, EffectiveBody, MaintenanceEvent


@dataclass
class BodyQueryResult:
    """Result of a body query."""

    question: str
    answer: str
    evidence: dict[str, Any] = field(default_factory=dict)
    actionable_policy: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "question": self.question,
            "answer": self.answer,
            "evidence": self.evidence,
            "actionable_policy": self.actionable_policy,
        }


class BodyQueryEngine:
    """Keyword-based question answering against body state."""

    def __init__(
        self,
        effective: EffectiveBody,
        body_yaml: BodyYaml,
        calibration: CalibrationYaml,
        maintenance: list[MaintenanceEvent],
    ):
        self.effective = effective
        self.body_yaml = body_yaml
        self.calibration = calibration
        self.maintenance = maintenance

    def answer(self, question: str) -> BodyQueryResult:
        """Return an answer plus evidence and policy notes."""
        q = question.lower()

        identity = self.body_yaml.get_identity()

        # Identity / what robot
        if any(
            kw in q
            for kw in ("what robot", "which robot", "what body", "who is this", "what is this")
        ):
            return BodyQueryResult(
                question=question,
                answer=(
                    f"This is {identity.get('nickname') or identity.get('robot_instance_id')} "
                    f"({identity.get('robot_model') or 'unknown model'})."
                ),
                evidence={
                    "robot_instance_id": identity.get("robot_instance_id"),
                    "robot_model": identity.get("robot_model"),
                    "robot_vendor": identity.get("robot_vendor"),
                },
                actionable_policy=[],
            )

        # Sandbox bypass
        if any(
            kw in q
            for kw in ("bypass sandbox", "skip sandbox", "run without sandbox", "disable sandbox")
        ):
            return BodyQueryResult(
                question=question,
                answer="No. Sandbox validation is mandatory for physical execution on this body.",
                evidence={
                    "policy": "physical_execution_requires_sandbox",
                    "value": True,
                },
                actionable_policy=[
                    "Refused: sandbox bypass is not allowed.",
                    "To execute physically, provide validation evidence via 'capability enable <id> --after-validation <run_id>'.",
                ],
            )

        # Dexterous-hand / gripper safety queries
        forbidden_ids = self._forbidden_ids()
        real_hardware_asked = any(kw in q for kw in ("real hardware", "real robot", "on hardware"))

        # Fast full close
        if any(
            kw in q for kw in ("close fast", "fast full close", "full close fast", "snap close")
        ):
            if (
                "fast_full_close" in forbidden_ids
                or "fast_full_close" in self.effective.capabilities.get("blocked", [])
            ):
                return BodyQueryResult(
                    question=question,
                    answer="No. Fast full close is forbidden because it risks collision and motor overload.",
                    evidence={
                        "forbidden": "fast_full_close",
                        "reason": "Risk of collision and motor overload",
                    },
                    actionable_policy=[
                        "Use only slow, validated close motions with current limits active."
                    ],
                )
            return BodyQueryResult(
                question=question,
                answer="Fast full close is not declared as a safe capability for this body.",
                evidence={"capabilities": self.effective.capabilities},
                actionable_policy=["Validate any fast motion in sandbox before real hardware."],
            )

        # Forceful grasp without current limit
        if any(
            kw in q
            for kw in (
                "forceful grasp",
                "forceful_grasp",
                "forcefully grasp",
                "grasp forcefully",
                "grasp without current limit",
                "grasp without torque limit",
            )
        ):
            if (
                "forceful_grasp_without_current_limit" in forbidden_ids
                or "forceful_grasp_without_current_limit"
                in self.effective.capabilities.get("blocked", [])
            ):
                return BodyQueryResult(
                    question=question,
                    answer="No. Forceful grasping without active current/torque limits is forbidden.",
                    evidence={"forbidden": "forceful_grasp_without_current_limit"},
                    actionable_policy=[
                        "Enable current/torque monitoring and use a validated grasp policy."
                    ],
                )
            return BodyQueryResult(
                question=question,
                answer="Forceful grasping is not allowed without active current/torque limits.",
                evidence={"capabilities": self.effective.capabilities},
                actionable_policy=["Confirm current/torque limits are configured before grasping."],
            )

        # OK gesture
        if any(kw in q for kw in ("ok gesture", "ok sign", "ok pose")):
            if real_hardware_asked:
                return self._answer_gesture_real_hardware("OK gesture")
            status, reasons = self._capability_id_status("ok_gesture")
            if status == "unknown":
                return BodyQueryResult(
                    question=question,
                    answer="OK gesture is not a declared capability for this body.",
                    evidence={"capabilities": self.effective.capabilities},
                    actionable_policy=[
                        "Declare the capability in the body profile if the hardware supports it."
                    ],
                )
            return BodyQueryResult(
                question=question,
                answer=f"OK gesture is {status}; run it in sandbox first and validate per-pose before real hardware.",
                evidence={"capability_status": status, "reasons": reasons},
                actionable_policy=["Use sandbox validation and per-pose human confirmation."],
            )

        # Countdown gesture
        if any(kw in q for kw in ("countdown gesture", "count down", "countdown")):
            if real_hardware_asked:
                return self._answer_gesture_real_hardware("Countdown gesture")
            status, reasons = self._capability_id_status("countdown_gesture")
            if status == "unknown":
                return BodyQueryResult(
                    question=question,
                    answer="Countdown gesture is not a declared capability for this body.",
                    evidence={"capabilities": self.effective.capabilities},
                    actionable_policy=[
                        "Declare the capability in the body profile if the hardware supports it."
                    ],
                )
            return BodyQueryResult(
                question=question,
                answer=f"Countdown gesture is {status}; run it in sandbox first and validate each pose before real hardware.",
                evidence={"capability_status": status, "reasons": reasons},
                actionable_policy=["Use sandbox validation and per-pose human confirmation."],
            )

        # Can it walk / locomotion
        if any(kw in q for kw in ("walk", "run", "locomotion", "move around")):
            caps = self.effective.capabilities
            if "walk" in caps.get("enabled", []):
                return BodyQueryResult(
                    question=question,
                    answer="Yes, walking/locomotion is enabled.",
                    evidence={"capabilities": caps},
                    actionable_policy=[],
                )
            if "walk" in caps.get("degraded", []):
                return BodyQueryResult(
                    question=question,
                    answer="Walking is degraded; it may only be allowed under reduced limits or in simulation.",
                    evidence={"capabilities": caps, "open_faults": self._open_fault_ids()},
                    actionable_policy=[
                        "Resolve open faults and re-validate before full operation."
                    ],
                )
            return BodyQueryResult(
                question=question,
                answer="No, walking/locomotion is not enabled for this body.",
                evidence={"capabilities": caps},
                actionable_policy=["Enable the capability explicitly if the hardware supports it."],
            )

        # Can it see / vision
        if any(kw in q for kw in ("see", "vision", "camera", "navigate visually", "visual")):
            caps = self.effective.capabilities
            if "visual_navigation" in caps.get("enabled", []):
                return BodyQueryResult(
                    question=question,
                    answer="Yes, visual sensing/navigation is enabled.",
                    evidence={"capabilities": caps, "sensors": list(self.effective.sensors.keys())},
                    actionable_policy=[],
                )
            if "visual_navigation" in caps.get("degraded", []):
                return BodyQueryResult(
                    question=question,
                    answer="Visual capabilities are degraded, likely due to calibration or sensor state.",
                    evidence={
                        "capabilities": caps,
                        "calibration_status": self.calibration.overall_status(),
                    },
                    actionable_policy=["Validate calibration and check camera availability."],
                )
            return BodyQueryResult(
                question=question,
                answer="No, visual navigation/sensing is not enabled.",
                evidence={"capabilities": caps},
                actionable_policy=[],
            )

        # Calibration
        if any(kw in q for kw in ("calibration", "calibrated")):
            status = self.calibration.overall_status()
            if status in ("valid", "validated"):
                return BodyQueryResult(
                    question=question,
                    answer=f"Calibration status is '{status}'.",
                    evidence={"calibration_status": status},
                    actionable_policy=[],
                )
            return BodyQueryResult(
                question=question,
                answer=f"Calibration status is '{status}'; precision capabilities may be degraded.",
                evidence={"calibration_status": status},
                actionable_policy=[
                    "Run 'rosclaw body calibration update --file <path>' to update calibration."
                ],
            )

        # Faults
        if any(kw in q for kw in ("fault", "problem", "issue", "broken", "wrong")):
            open_faults = self._open_fault_ids()
            if open_faults:
                return BodyQueryResult(
                    question=question,
                    answer=f"There are {len(open_faults)} open fault(s): {', '.join(open_faults)}.",
                    evidence={"open_faults": open_faults},
                    actionable_policy=[
                        "Resolve faults via 'rosclaw body fault resolve <fault_id>'."
                    ],
                )
            return BodyQueryResult(
                question=question,
                answer="No open faults are recorded.",
                evidence={"open_faults": []},
                actionable_policy=[],
            )

        # Capabilities list
        if any(kw in q for kw in ("capability", "capabilities", "can it do", "what can")):
            caps = self.effective.capabilities
            enabled = caps.get("enabled", [])
            return BodyQueryResult(
                question=question,
                answer=f"Enabled capabilities: {', '.join(enabled) if enabled else 'none'}.",
                evidence={"capabilities": caps, "forbidden": self.effective.forbidden_capabilities},
                actionable_policy=[
                    "Use 'rosclaw body state --json' for the full capability matrix."
                ],
            )

        # Safety
        if any(kw in q for kw in ("safe", "safety", "limit", "emergency stop", "e-stop")):
            safety = self.effective.safety
            return BodyQueryResult(
                question=question,
                answer=f"Overall safety status is '{self.body_yaml.get_safety_status()}'.",
                evidence={
                    "safety_status": self.body_yaml.get_safety_status(),
                    "global_limits": safety.get("safety_limits") or safety.get("global_limits"),
                },
                actionable_policy=["Review EMBODIMENT.md section 7 for full safety limits."],
            )

        # Default fallback
        return BodyQueryResult(
            question=question,
            answer="I don't have a specific answer for that question. Try asking about identity, capabilities, calibration, faults, safety, or sandbox policy.",
            evidence={"capabilities": self.effective.capabilities},
            actionable_policy=["Run 'rosclaw body state --json' for structured body state."],
        )

    def _forbidden_ids(self) -> set[str]:
        ids: set[str] = set()
        for item in (
            self.effective.forbidden_capabilities or self.body_yaml.forbidden_capabilities or []
        ):
            cap_id = item.get("id") or item.get("capability")
            if cap_id:
                ids.add(cap_id)
        return ids

    def _real_robot_allowed(self) -> bool:
        return bool(
            self.body_yaml.agent_policy.get("direct_real_robot_execution_allowed", True)
            and self.effective.safety.get("environment", {}).get(
                "real_robot_execution_allowed", True
            )
        )

    def _calibration_valid(self) -> bool:
        return self.calibration.overall_status() in ("valid", "validated")

    def _resolve_capability(self, cap_id: str) -> str | None:
        """Find a declared capability matching ``cap_id`` by id or display name."""
        caps = self.effective.capabilities
        target = cap_id.lower().replace("_", " ")
        for bucket in (caps.get("blocked", []), caps.get("degraded", []), caps.get("enabled", [])):
            for cap in bucket:
                normalized = cap.lower().replace("_", " ")
                if normalized == target or cap_id.lower() in cap.lower():
                    return cap
        return None

    def _capability_id_status(self, cap_id: str) -> tuple[str, list[str]]:
        caps = self.effective.capabilities
        reasons: list[str] = []
        resolved = self._resolve_capability(cap_id)
        if resolved is None:
            return "unknown", reasons
        if resolved in caps.get("blocked", []):
            return "blocked", reasons
        if resolved in caps.get("degraded", []):
            if not self._real_robot_allowed():
                reasons.append("real robot execution is not allowed for this asset")
            if not self._calibration_valid():
                reasons.append("calibration is not validated")
            return "degraded", reasons
        return "enabled", reasons

    def _answer_gesture_real_hardware(self, gesture_name: str) -> BodyQueryResult:
        if not self._real_robot_allowed():
            return BodyQueryResult(
                question=f"Can this body perform {gesture_name} on real hardware?",
                answer=f"No. {gesture_name} is blocked on real hardware because real robot execution is not allowed for this asset.",
                evidence={
                    "gesture": gesture_name,
                    "real_robot_execution_allowed": False,
                    "calibration_status": self.calibration.overall_status(),
                },
                actionable_policy=[
                    "Complete sandbox validation and clear the real-robot execution gate before real hardware motion.",
                ],
            )
        if not self._calibration_valid():
            return BodyQueryResult(
                question=f"Can this body perform {gesture_name} on real hardware?",
                answer=f"No. {gesture_name} is blocked on real hardware until calibration is validated.",
                evidence={
                    "gesture": gesture_name,
                    "calibration_status": self.calibration.overall_status(),
                },
                actionable_policy=[
                    "Upload validated clearance calibration and run a low-speed range-of-motion check.",
                ],
            )
        return BodyQueryResult(
            question=f"Can this body perform {gesture_name} on real hardware?",
            answer=f"Yes, {gesture_name} is allowed on real hardware once per-pose human confirmation is obtained.",
            evidence={
                "gesture": gesture_name,
                "calibration_status": self.calibration.overall_status(),
            },
            actionable_policy=[
                "Confirm current/torque limits are active before executing the gesture."
            ],
        )

    def _open_fault_ids(self) -> list[str]:
        ids: list[str] = []
        for fault in self.effective.known_faults:
            if fault.get("status") == "open":
                ids.append(fault.get("id") or "unknown")
        return ids
