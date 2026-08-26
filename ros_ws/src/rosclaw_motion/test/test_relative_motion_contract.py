import math

import pytest
from geometry_msgs.msg import Quaternion, Vector3
from rosclaw_interfaces.action import ArmMotion

from rosclaw_motion.relative_motion_server import (
    ARM_BASE_LINKS,
    ARM_JOINTS,
    ARM_TIP_LINKS,
    ERROR_CANCELED,
    ERROR_EXECUTION,
    ERROR_INTERNAL,
    ERROR_INVALID_GOAL,
    ERROR_MOVEIT_REJECTED,
    ERROR_NOT_READY,
    ERROR_PATH,
    ERROR_SUCCESS,
    ERROR_TF,
    RelativeMotionServer,
)


def test_dual_arm_joint_contract_is_symmetric_and_disjoint():
    assert set(ARM_JOINTS) == {"left", "right"}
    assert len(ARM_JOINTS["left"]) == 7
    assert len(ARM_JOINTS["right"]) == 7
    assert set(ARM_JOINTS["left"]).isdisjoint(ARM_JOINTS["right"])

    for side in ("left", "right"):
        assert ARM_JOINTS[side] == [
            f"openarm_{side}_joint{index}" for index in range(1, 8)
        ]


def test_dual_arm_link_contract_is_complete():
    assert ARM_BASE_LINKS == {
        "left": "openarm_left_base_link",
        "right": "openarm_right_base_link",
    }
    assert ARM_TIP_LINKS == {
        "left": "openarm_left_ee_base_link",
        "right": "openarm_right_ee_base_link",
    }


def test_arm_motion_action_constants_are_stable():
    assert ArmMotion.Goal.MOVE_RELATIVE == 1
    assert ArmMotion.Goal.MOVE_ABSOLUTE == 2
    assert ArmMotion.Goal.HOME == 3


def test_error_codes_are_unique():
    error_codes = {
        ERROR_SUCCESS,
        ERROR_INVALID_GOAL,
        ERROR_NOT_READY,
        ERROR_TF,
        ERROR_PATH,
        ERROR_MOVEIT_REJECTED,
        ERROR_EXECUTION,
        ERROR_CANCELED,
        ERROR_INTERNAL,
    }
    assert len(error_codes) == 9
    assert ERROR_SUCCESS == 0
    assert all(code < 0 for code in error_codes if code != ERROR_SUCCESS)


def test_identity_quaternion_preserves_translation():
    vector = Vector3(x=0.1, y=-0.2, z=0.3)
    rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    rotated = RelativeMotionServer._rotate_vector(vector, rotation)

    assert rotated == pytest.approx((0.1, -0.2, 0.3))


def test_z_quarter_turn_rotates_x_axis_to_y_axis():
    half_angle = math.pi / 4.0
    vector = Vector3(x=1.0, y=0.0, z=0.0)
    rotation = Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_angle),
        w=math.cos(half_angle),
    )

    rotated = RelativeMotionServer._rotate_vector(vector, rotation)

    assert rotated == pytest.approx((0.0, 1.0, 0.0), abs=1.0e-12)


def test_result_contains_structured_verification_fields():
    result = RelativeMotionServer._result(
        True,
        ERROR_SUCCESS,
        "verified",
        final_error_m=0.001,
    )

    assert result.success is True
    assert result.error_code == ERROR_SUCCESS
    assert result.message == "verified"
    assert result.final_error_m == pytest.approx(0.001)
