from pathlib import Path

import numpy as np
import pytest

from scripts.openarm_absolute_action_safety import (
    load_joint_limits,
    project_arm_step_limit,
    project_absolute_position_targets,
    project_gripper_step_limit,
)


URDF = Path("repos/openarm_dev/openarm_urdf/openarm_bimanual.urdf")


def test_hardware_projection_preserves_absolute_chunk_and_clock() -> None:
    lower, upper = load_joint_limits(URDF)
    raw = np.tile((lower + upper) / 2.0, (30, 1)).astype(np.float32)
    raw[0, 7] = -0.004
    raw[1, 15] = 0.048
    original = raw.copy()

    safe, correction = project_absolute_position_targets(raw, lower, upper)

    assert raw.shape == safe.shape == correction.shape == (30, 16)
    assert np.array_equal(raw, original)
    assert safe[0, 7] == pytest.approx(0.0)
    assert safe[1, 15] == pytest.approx(0.044)
    unchanged = np.ones_like(raw, dtype=bool)
    unchanged[0, 7] = False
    unchanged[1, 15] = False
    assert np.array_equal(safe[unchanged], raw[unchanged])
    assert np.array_equal(correction[unchanged], np.zeros_like(correction[unchanged]))


def test_hardware_projection_is_idempotent() -> None:
    lower, upper = load_joint_limits(URDF)
    raw = np.tile((lower + upper) / 2.0, (30, 1)).astype(np.float32)
    raw[:, 7] = np.linspace(-0.01, 0.05, 30)
    safe, _ = project_absolute_position_targets(raw, lower, upper)
    second, correction = project_absolute_position_targets(safe, lower, upper)
    assert np.array_equal(second, safe)
    assert np.count_nonzero(correction) == 0


def test_hardware_projection_rejects_invalid_targets() -> None:
    lower, upper = load_joint_limits(URDF)
    with pytest.raises(ValueError, match="16"):
        project_absolute_position_targets(np.zeros((30, 15)), lower, upper)
    invalid = np.zeros((30, 16), dtype=np.float32)
    invalid[0, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        project_absolute_position_targets(invalid, lower, upper)


def test_gripper_step_projection_preserves_arms_and_absolute_semantics() -> None:
    reference = np.zeros(16, dtype=np.float32)
    reference[[7, 15]] = [0.004, 0.003]
    target = np.linspace(-0.4, 0.4, 16, dtype=np.float32)
    target[[7, 15]] = [0.040, 0.044]
    original = target.copy()

    safe, correction = project_gripper_step_limit(target, reference)

    assert np.array_equal(target, original)
    assert np.array_equal(safe[:7], target[:7])
    assert np.array_equal(safe[8:15], target[8:15])
    assert safe[7] == pytest.approx(0.024)
    assert safe[15] == pytest.approx(0.023)
    assert np.count_nonzero(correction[[*range(7), *range(8, 15)]]) == 0


def test_gripper_step_projection_advances_once_per_policy_step() -> None:
    target = np.zeros(16, dtype=np.float32)
    target[[7, 15]] = 0.044
    sent = np.zeros(16, dtype=np.float32)
    outputs = []
    for _ in range(3):
        sent, _ = project_gripper_step_limit(target, sent)
        outputs.append(sent[[7, 15]].copy())

    np.testing.assert_allclose(
        np.asarray(outputs),
        np.asarray([[0.020, 0.020], [0.040, 0.040], [0.044, 0.044]]),
        atol=1e-7,
        rtol=0.0,
    )


def test_arm_step_projection_preserves_direction_grippers_and_clock() -> None:
    reference = np.zeros(16, dtype=np.float32)
    target = np.zeros(16, dtype=np.float32)
    target[0] = 0.40
    target[1] = -0.20
    target[8] = 0.10
    target[[7, 15]] = [0.031, 0.042]

    safe, correction = project_arm_step_limit(target, reference)

    assert safe[0] == pytest.approx(0.20)
    assert safe[1] == pytest.approx(-0.10)
    assert safe[8] == pytest.approx(0.05)
    assert safe[[7, 15]] == pytest.approx(target[[7, 15]])
    assert np.max(np.abs(safe[[*range(7), *range(8, 15)]])) <= (
        0.20 + np.finfo(safe.dtype).eps
    )
    assert np.count_nonzero(correction[[7, 15]]) == 0


def test_arm_step_projection_is_noop_within_limit() -> None:
    reference = np.linspace(-0.3, 0.3, 16, dtype=np.float32)
    target = reference.copy()
    target[[0, 6, 8, 14]] += [0.19, -0.10, 0.05, -0.19]

    safe, correction = project_arm_step_limit(target, reference)

    assert np.array_equal(safe, target)
    assert np.count_nonzero(correction) == 0
