#!/usr/bin/env python3
"""Hardware-only safety processing for absolute OpenArm16 position targets.

This module does not implement policy semantics. It preserves action ordering,
shape, and clock; it only applies hardware projections to absolute targets.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


JOINT_NAMES = [
    *[f"openarm_left_joint{i}" for i in range(1, 8)],
    "openarm_left_finger_joint1",
    *[f"openarm_right_joint{i}" for i in range(1, 8)],
    "openarm_right_finger_joint1",
]
GRIPPER_INDICES = np.asarray((7, 15), dtype=np.int64)
ARM_INDICES = np.asarray((*range(7), *range(8, 15)), dtype=np.int64)


def load_joint_limits(urdf: Path) -> tuple[np.ndarray, np.ndarray]:
    root = ET.parse(urdf).getroot()
    values: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            continue
        values[joint.attrib["name"]] = (
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
    missing = [name for name in JOINT_NAMES if name not in values]
    if missing:
        raise ValueError(f"URDF is missing OpenArm16 limits: {missing}")
    lower = np.asarray([values[name][0] for name in JOINT_NAMES], dtype=np.float64)
    upper = np.asarray([values[name][1] for name in JOINT_NAMES], dtype=np.float64)
    return lower, upper


def project_absolute_position_targets(
    raw_targets: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return safe absolute targets and their additive safety correction.

    The final axis is the fixed OpenArm16 ordering. Leading dimensions (for
    example chunk time) are preserved exactly.
    """
    raw = np.asarray(raw_targets)
    low = np.asarray(lower)
    high = np.asarray(upper)
    if raw.ndim < 1 or raw.shape[-1] != len(JOINT_NAMES):
        raise ValueError(f"Expected (..., 16) absolute targets, got {raw.shape}")
    if low.shape != (16,) or high.shape != (16,):
        raise ValueError("OpenArm lower and upper limits must both have shape (16,)")
    if not np.isfinite(raw).all() or not np.isfinite(low).all() or not np.isfinite(high).all():
        raise ValueError("Targets and limits must be finite")
    if np.any(low > high):
        raise ValueError("Every lower joint limit must be <= its upper limit")
    safe = np.clip(raw, low.astype(raw.dtype), high.astype(raw.dtype))
    correction = safe - raw
    return safe, correction


def project_gripper_step_limit(
    absolute_target: np.ndarray,
    reference_target: np.ndarray,
    max_step_m: float = 0.02,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply a gripper-only hardware slew limit to one absolute target.

    This is controller safety post-processing, not an action representation:
    one output is produced for every policy-queue step, arm targets are
    unchanged, and no step is delayed, dropped, accumulated, or rebased.
    """
    target = np.asarray(absolute_target)
    reference = np.asarray(reference_target)
    if target.shape != (16,) or reference.shape != (16,):
        raise ValueError("Absolute target and reference target must both have shape (16,)")
    if not np.isfinite(target).all() or not np.isfinite(reference).all():
        raise ValueError("Absolute target and reference target must be finite")
    if not np.isfinite(max_step_m) or not 0.0 < max_step_m <= 0.02:
        raise ValueError("max_step_m must be finite and within (0, 0.02]")

    safe = target.copy()
    gripper_step = target[GRIPPER_INDICES] - reference[GRIPPER_INDICES]
    safe[GRIPPER_INDICES] = reference[GRIPPER_INDICES] + np.clip(
        gripper_step, -max_step_m, max_step_m
    ).astype(target.dtype, copy=False)
    correction = safe - target
    return safe, correction


def project_arm_step_limit(
    absolute_target: np.ndarray,
    reference_target: np.ndarray,
    max_step_rad: float = 0.20,
) -> tuple[np.ndarray, np.ndarray]:
    """Direction-preserving arm slew projection for one absolute target.

    Grippers and policy timing are untouched. When one arm component exceeds
    the limit, all 14 arm components are scaled by the same factor so the
    requested joint-space direction is preserved.
    """
    target = np.asarray(absolute_target)
    reference = np.asarray(reference_target)
    if target.shape != (16,) or reference.shape != (16,):
        raise ValueError("Absolute target and reference target must both have shape (16,)")
    if not np.isfinite(target).all() or not np.isfinite(reference).all():
        raise ValueError("Absolute target and reference target must be finite")
    if not np.isfinite(max_step_rad) or not 0.0 < max_step_rad <= 0.20:
        raise ValueError("max_step_rad must be finite and within (0, 0.20]")

    safe = target.copy()
    arm_step = target[ARM_INDICES] - reference[ARM_INDICES]
    peak = float(np.max(np.abs(arm_step)))
    if peak > max_step_rad:
        scale = max_step_rad / peak
        safe[ARM_INDICES] = reference[ARM_INDICES] + arm_step * scale
    correction = safe - target
    return safe, correction
