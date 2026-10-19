#!/usr/bin/env python3
"""Offline held-out validation of the standard absolute-action OpenArm ACT.

No ROS context, node, publisher, controller, or hardware command is created.
"""

from __future__ import annotations

import argparse
import inspect
import json
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow.parquet as pq
import torch
import xml.etree.ElementTree as ET
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import PolicyProcessorPipeline

from validate_openarm_act_offline import make_observation, postprocess_chunk
from openarm_absolute_action_safety import project_absolute_position_targets


ARM = np.asarray((*range(7), *range(8, 15)), dtype=np.int64)
GRIPPERS = np.asarray((7, 15), dtype=np.int64)
JOINT_NAMES = [
    *[f"openarm_left_joint{i}" for i in range(1, 8)],
    "openarm_left_finger_joint1",
    *[f"openarm_right_joint{i}" for i in range(1, 8)],
    "openarm_right_finger_joint1",
]
HORIZON = 30
FPS = 15



def load_joint_limits(urdf: Path) -> tuple[np.ndarray, np.ndarray]:
    root = ET.parse(urdf).getroot()
    values = {}
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

def first_sustained(mask: np.ndarray, samples: int = 3) -> int | None:
    for index in range(max(0, len(mask) - samples + 1)):
        if bool(np.all(mask[index : index + samples])):
            return index
    return None


def numeric_actions(root: Path) -> np.ndarray:
    paths = sorted((root / "data").glob("chunk-*/*.parquet"))
    if not paths:
        raise FileNotFoundError(root / "data")
    table = pq.read_table([str(path) for path in paths], columns=["action"])
    return np.asarray(table["action"].to_pylist(), dtype=np.float32)


def event_summary(actions: np.ndarray) -> dict[str, Any]:
    both_open = np.all(actions[:, GRIPPERS] >= 0.035, axis=1)
    opened = first_sustained(both_open)
    right_close = None
    left_close = None
    arm_progress = 0.0
    later_progress = 0.0
    if opened is not None:
        right_offset = first_sustained(actions[opened:, 15] <= 0.030)
        left_offset = first_sustained(actions[opened:, 7] <= 0.030)
        right_close = None if right_offset is None else opened + right_offset
        left_close = None if left_offset is None else opened + left_offset
        arm_progress = float(
            np.max(np.abs(actions[opened:, ARM] - actions[opened, ARM]))
        )
        if right_close is not None:
            later_progress = float(
                np.max(
                    np.abs(actions[right_close:, ARM] - actions[right_close, ARM])
                )
            )
    return {
        "initial_open_frame": opened,
        "initial_open_sec": None if opened is None else opened / FPS,
        "right_close_frame": right_close,
        "right_close_sec": None if right_close is None else right_close / FPS,
        "left_close_frame": left_close,
        "left_close_sec": None if left_close is None else left_close / FPS,
        "post_open_arm_progress_rad": arm_progress,
        "later_arm_progress_rad": later_progress,
    }


def load_policy(checkpoint: Path, device: torch.device):
    policy = ACTPolicy.from_pretrained(str(checkpoint)).to(device).eval()
    preprocessor = PolicyProcessorPipeline.from_pretrained(
        str(checkpoint), config_filename="policy_preprocessor.json"
    )
    postprocessor = PolicyProcessorPipeline.from_pretrained(
        str(checkpoint), config_filename="policy_postprocessor.json"
    )
    config = policy.config
    if config.chunk_size != HORIZON or config.n_action_steps != HORIZON:
        raise ValueError("Checkpoint is not configured for 30/30 ACT chunks")
    if config.temporal_ensemble_coeff is not None:
        raise ValueError("Temporal ensemble must be disabled")
    if tuple(config.input_features["observation.state"].shape) != (16,):
        raise ValueError("Checkpoint observation.state is not OpenArm16")
    if tuple(config.output_features["action"].shape) != (16,):
        raise ValueError("Checkpoint action is not OpenArm16")
    if not config.use_vae:
        raise ValueError("Standard ACT VAE is unexpectedly disabled")
    return policy, preprocessor, postprocessor


def queue_audit(
    policy: ACTPolicy,
    preprocessor: PolicyProcessorPipeline,
    postprocessor: PolicyProcessorPipeline,
    sample: dict[str, Any],
    device: torch.device,
    joint_lower: np.ndarray,
    joint_upper: np.ndarray,
) -> dict[str, Any]:
    observation = preprocessor(make_observation(sample, policy, device))
    policy.reset()
    full = postprocess_chunk(
        postprocessor, policy.predict_action_chunk(observation)
    )[0, :HORIZON].detach().cpu().numpy().astype(np.float32)
    policy.reset()
    queued = []
    queue_lengths = []
    for _ in range(HORIZON):
        normalized_action = policy.select_action(observation)
        action = postprocessor({"action": normalized_action})["action"]
        queued.append(action[0].detach().cpu().numpy().astype(np.float32))
        queue_lengths.append(len(policy._action_queue))
    queued_array = np.asarray(queued, dtype=np.float32)
    error = float(np.max(np.abs(full - queued_array)))
    safe_full, full_correction = project_absolute_position_targets(
        full, joint_lower, joint_upper
    )
    safe_queued, queued_correction = project_absolute_position_targets(
        queued_array, joint_lower, joint_upper
    )
    safe_error = float(np.max(np.abs(safe_full - safe_queued)))
    source = inspect.getsource(type(policy).select_action)
    return {
        "policy_reset_called_before_episode": True,
        "full_chunk_shape": list(full.shape),
        "select_action_sequence_shape": list(queued_array.shape),
        "max_abs_full_chunk_vs_select_action_sequence": error,
        "exact_sequence_match": bool(np.array_equal(full, queued_array)),
        "queue_lengths_after_each_action": queue_lengths,
        "queue_drained_after_30": queue_lengths[-1] == 0,
        "hardware_safety_sequence_shape": list(safe_queued.shape),
        "hardware_safety_max_abs_full_chunk_vs_select_action_sequence": safe_error,
        "hardware_safety_exact_sequence_match": bool(
            np.array_equal(safe_full, safe_queued)
        ),
        "hardware_safety_projection_count": int(np.count_nonzero(queued_correction)),
        "full_chunk_hardware_safety_projection_count": int(np.count_nonzero(full_correction)),
        "source_uses_action_queue": all(
            token in source for token in ("_action_queue", "predict_action_chunk", "popleft")
        ),
    }


def validate_split(
    split: str,
    root: Path,
    policy: ACTPolicy,
    preprocessor: PolicyProcessorPipeline,
    postprocessor: PolicyProcessorPipeline,
    device: torch.device,
    train_min: np.ndarray,
    train_max: np.ndarray,
    joint_lower: np.ndarray,
    joint_upper: np.ndarray,
    stride: int,
) -> tuple[dict[str, Any], list[str]]:
    dataset = LeRobotDataset(repo_id=f"openarm_absolute_final_{split}", root=root)
    if dataset.num_episodes != 1:
        raise ValueError(f"{split}: expected exactly one held-out episode")
    bounds = dataset.meta.episodes[0]
    start = int(bounds["dataset_from_index"])
    stop = int(bounds["dataset_to_index"])
    frames = np.arange(0, stop - start, stride, dtype=np.int64)
    recorded_all = np.asarray(
        [dataset.hf_dataset[start + i]["action"] for i in range(stop - start)],
        dtype=np.float32,
    )
    measured_all = np.asarray(
        [dataset.hf_dataset[start + i]["observation.state"] for i in range(stop - start)],
        dtype=np.float32,
    )
    predicted_first = []
    controller_first = []
    recorded_first = []
    measured_sampled = []
    chunk_errors = []
    chunk_arm_jumps = []
    chunk_gripper_jumps = []
    controller_chunk_arm_jumps = []
    controller_chunk_gripper_jumps = []
    all_predicted_min = np.full(16, np.inf, dtype=np.float64)
    all_predicted_max = np.full(16, -np.inf, dtype=np.float64)
    controller_target_min = np.full(16, np.inf, dtype=np.float64)
    controller_target_max = np.full(16, -np.inf, dtype=np.float64)
    hard_bound_violation_counts = np.zeros(16, dtype=np.int64)
    first_action_bound_violation_counts = np.zeros(16, dtype=np.int64)
    safety_projection_counts = np.zeros(16, dtype=np.int64)
    maximum_safety_correction = np.zeros(16, dtype=np.float64)
    controller_bound_violation_counts = np.zeros(16, dtype=np.int64)
    hard_bound_examples: list[dict[str, Any]] = []
    failures: list[str] = []
    queue = None

    with torch.inference_mode():
        for number, frame in enumerate(frames, start=1):
            sample = dataset[start + int(frame)]
            observation = preprocessor(make_observation(sample, policy, device))
            chunk = policy.predict_action_chunk(observation)
            actions = postprocess_chunk(postprocessor, chunk)[0].detach().cpu().numpy()
            actions = actions.astype(np.float32, copy=False)
            if actions.shape != (HORIZON, 16) or not np.isfinite(actions).all():
                failures.append(f"frame {frame}: invalid chunk {actions.shape}")
                break
            controller_actions, safety_correction = project_absolute_position_targets(
                actions, joint_lower, joint_upper
            )
            if queue is None:
                queue = queue_audit(
                    policy, preprocessor, postprocessor, sample, device,
                    joint_lower, joint_upper
                )
            predicted_first.append(actions[0])
            controller_first.append(controller_actions[0])
            recorded_first.append(recorded_all[frame])
            measured_sampled.append(measured_all[frame])
            valid = min(HORIZON, len(recorded_all) - int(frame))
            chunk_errors.append(np.abs(actions[:valid] - recorded_all[frame : frame + valid]))
            chunk_arm_jumps.append(
                float(np.max(np.abs(np.diff(actions[:, ARM], axis=0))))
            )
            chunk_gripper_jumps.append(
                float(np.max(np.abs(np.diff(actions[:, GRIPPERS], axis=0))))
            )
            all_predicted_min = np.minimum(all_predicted_min, actions.min(axis=0))
            all_predicted_max = np.maximum(all_predicted_max, actions.max(axis=0))
            controller_target_min = np.minimum(
                controller_target_min, controller_actions.min(axis=0)
            )
            controller_target_max = np.maximum(
                controller_target_max, controller_actions.max(axis=0)
            )
            controller_chunk_arm_jumps.append(
                float(np.max(np.abs(np.diff(controller_actions[:, ARM], axis=0))))
            )
            controller_chunk_gripper_jumps.append(
                float(np.max(np.abs(np.diff(controller_actions[:, GRIPPERS], axis=0))))
            )
            projected = safety_correction != 0.0
            safety_projection_counts += projected.sum(axis=0)
            maximum_safety_correction = np.maximum(
                maximum_safety_correction, np.max(np.abs(safety_correction), axis=0)
            )
            controller_out_of_bounds = (
                (controller_actions < joint_lower - 1e-6)
                | (controller_actions > joint_upper + 1e-6)
            )
            controller_bound_violation_counts += controller_out_of_bounds.sum(axis=0)
            out_of_bounds = (actions < joint_lower - 1e-6) | (actions > joint_upper + 1e-6)
            hard_bound_violation_counts += out_of_bounds.sum(axis=0)
            first_action_bound_violation_counts += out_of_bounds[0].astype(np.int64)
            if len(hard_bound_examples) < 30:
                for step, joint in np.argwhere(out_of_bounds):
                    if len(hard_bound_examples) >= 30:
                        break
                    hard_bound_examples.append(
                        {
                            "observation_frame": int(frame),
                            "chunk_step": int(step),
                            "joint": JOINT_NAMES[int(joint)],
                            "value": float(actions[step, joint]),
                            "lower": float(joint_lower[joint]),
                            "upper": float(joint_upper[joint]),
                        }
                    )
            if number % 100 == 0 or number == len(frames):
                print(f"{split}: {number}/{len(frames)} full chunks", flush=True)

    if queue is None:
        failures.append("no action chunk was evaluated")
        queue = {}
    elif not (
        queue["exact_sequence_match"]
        and queue["hardware_safety_exact_sequence_match"]
        and queue["queue_drained_after_30"]
        and queue["source_uses_action_queue"]
    ):
        failures.append("native policy.select_action did not preserve the 30-step chunk")

    predicted = np.asarray(predicted_first, dtype=np.float32)
    controller_predicted = np.asarray(controller_first, dtype=np.float32)
    recorded = np.asarray(recorded_first, dtype=np.float32)
    measured = np.asarray(measured_sampled, dtype=np.float32)
    errors = np.abs(predicted - recorded)
    chunk_error = np.concatenate(chunk_errors, axis=0)
    baseline_error = np.abs(measured - recorded)
    recorded_events = event_summary(recorded_all)
    predicted_events = event_summary(predicted)
    controller_events = event_summary(controller_predicted)
    for name in ("initial_open_frame", "right_close_frame", "left_close_frame"):
        if controller_events[name] is None:
            failures.append(f"hardware-safe controller sequence is missing {name}")
    if controller_events["post_open_arm_progress_rad"] < 0.05:
        failures.append("hardware-safe sequence has insufficient post-open arm progression")
    if controller_events["later_arm_progress_rad"] < 0.05:
        failures.append("hardware-safe sequence has insufficient later arm progression")

    max_arm_jump = max(chunk_arm_jumps, default=0.0)
    max_gripper_jump = max(chunk_gripper_jumps, default=0.0)
    controller_max_arm_jump = max(controller_chunk_arm_jumps, default=0.0)
    controller_max_gripper_jump = max(controller_chunk_gripper_jumps, default=0.0)
    if controller_max_arm_jump > 0.20:
        failures.append(f"controller chunk arm jump {controller_max_arm_jump:.6f} > 0.20 rad")
    if controller_max_gripper_jump > 0.02:
        failures.append(f"controller chunk gripper jump {controller_max_gripper_jump:.6f} > 0.02 m")

    margin = np.full(16, 0.05, dtype=np.float64)
    margin[GRIPPERS] = 0.005
    train_envelope_exceeded = (
        (all_predicted_min < train_min - margin)
        | (all_predicted_max > train_max + margin)
    )
    hard_bound_exceeded = hard_bound_violation_counts > 0
    controller_hard_bound_exceeded = controller_bound_violation_counts > 0
    if np.any(controller_hard_bound_exceeded):
        names = [JOINT_NAMES[i] for i in np.flatnonzero(controller_hard_bound_exceeded)]
        failures.append(f"hardware-safe controller targets leave URDF hard bounds: {names}")

    report = {
        "frames_in_episode": int(stop - start),
        "evaluated_frames": int(len(frames)),
        "stride": stride,
        "predicted_chunk_shape": [HORIZON, 16],
        "first_step_mae": {
            "arm_rad": float(np.mean(errors[:, ARM])),
            "gripper_m": float(np.mean(errors[:, GRIPPERS])),
            "per_joint": {
                name: float(errors[:, index].mean())
                for index, name in enumerate(JOINT_NAMES)
            },
        },
        "full_chunk_mae": {
            "arm_rad": float(np.mean(chunk_error[:, ARM])),
            "gripper_m": float(np.mean(chunk_error[:, GRIPPERS])),
        },
        "measured_state_hold_baseline_mae": {
            "arm_rad": float(np.mean(baseline_error[:, ARM])),
            "gripper_m": float(np.mean(baseline_error[:, GRIPPERS])),
        },
        "maximum_predicted_chunk_jump": {
            "arm_rad": max_arm_jump,
            "gripper_m": max_gripper_jump,
        },
        "recorded_events": recorded_events,
        "predicted_first_action_events": predicted_events,
        "hardware_safe_controller_events": controller_events,
        "hardware_safe_event_timing_error_sec": {
            key: (
                None
                if recorded_events[key] is None or controller_events[key] is None
                else (controller_events[key] * stride - recorded_events[key]) / FPS
            )
            for key in ("initial_open_frame", "right_close_frame", "left_close_frame")
        },
        "event_timing_error_sec": {
            key: (
                None
                if recorded_events[key] is None or predicted_events[key] is None
                else (predicted_events[key] * stride - recorded_events[key]) / FPS
            )
            for key in ("initial_open_frame", "right_close_frame", "left_close_frame")
        },
        "predicted_min": all_predicted_min.tolist(),
        "predicted_max": all_predicted_max.tolist(),
        "train_envelope_diagnostic": {
            "margin_arm_rad": 0.05,
            "margin_gripper_m": 0.005,
            "exceeded_joints": [
                JOINT_NAMES[i] for i in np.flatnonzero(train_envelope_exceeded)
            ],
            "is_safety_gate": False,
        },
        "urdf_hard_bound_audit": {
            "lower": joint_lower.tolist(),
            "upper": joint_upper.tolist(),
            "violation_counts_by_joint": {
                name: int(hard_bound_violation_counts[index])
                for index, name in enumerate(JOINT_NAMES)
            },
            "first_action_violation_counts_by_joint": {
                name: int(first_action_bound_violation_counts[index])
                for index, name in enumerate(JOINT_NAMES)
            },
            "examples": hard_bound_examples,
            "pass": not bool(np.any(hard_bound_exceeded)),
            "is_deployment_gate": False,
            "note": "Raw policy diagnostic; deployment gate is hardware safety output.",
        },
        "hardware_safety_postprocessing": {
            "type": "elementwise_urdf_hard_limit_projection",
            "input_semantics": "raw absolute OpenArm16 ACT target",
            "output_semantics": "hardware-safe absolute OpenArm16 controller target",
            "uses_measured_state": False,
            "changes_policy_clock": False,
            "changes_chunk_order_or_length": False,
            "controller_target_min": controller_target_min.tolist(),
            "controller_target_max": controller_target_max.tolist(),
            "projection_counts_by_joint": {
                name: int(safety_projection_counts[index])
                for index, name in enumerate(JOINT_NAMES)
            },
            "maximum_abs_correction_by_joint": {
                name: float(maximum_safety_correction[index])
                for index, name in enumerate(JOINT_NAMES)
            },
            "controller_bound_violation_counts_by_joint": {
                name: int(controller_bound_violation_counts[index])
                for index, name in enumerate(JOINT_NAMES)
            },
            "maximum_controller_chunk_jump": {
                "arm_rad": controller_max_arm_jump,
                "gripper_m": controller_max_gripper_jump,
            },
            "event_sequence_preserved": controller_events == predicted_events,
            "pass": not bool(np.any(controller_hard_bound_exceeded)),
        },
        "native_select_action_queue_audit": queue,
        "pass": not failures,
        "failures": failures,
    }
    return report, failures


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path(
            "output/openarm_vr_native_absolute_act_15fps_50k/"
            "checkpoints/050000/pretrained_model"
        ),
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("Dataset/Converted_dataset/openarm_vr_native_absolute_v1"),
    )
    parser.add_argument(
        "--dataset-contract",
        type=Path,
        default=Path("output/openarm_native_absolute_audit_v1/dataset_contract.json"),
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path("repos/openarm_dev/openarm_urdf/openarm_bimanual.urdf"),
    )
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument(
        "--splits",
        nargs="+",
        choices=("validation", "test"),
        default=("validation", "test"),
        help="Held-out splits to evaluate. Use validation only for checkpoint selection.",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.stride < 1:
        raise ValueError("--stride must be positive")
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite report: {args.output}")

    dataset_contract = json.loads(args.dataset_contract.read_text(encoding="utf-8"))
    if not dataset_contract["pass"]:
        raise ValueError("Absolute-controller dataset contract has not passed")
    for split in ("train", "validation", "test"):
        if dataset_contract["splits"][split]["max_raw_action_comparison_error"] != 0.0:
            raise ValueError(f"{split}: dataset action is not exact raw command")

    config = json.loads((args.checkpoint / "config.json").read_text(encoding="utf-8"))
    if config.get("pretrained_path") is not None:
        raise ValueError("Checkpoint unexpectedly records pretrained weights")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    policy, preprocessor, postprocessor = load_policy(args.checkpoint, device)
    train_actions = numeric_actions(args.dataset_root / "train")
    train_min = train_actions.min(axis=0).astype(np.float64)
    train_max = train_actions.max(axis=0).astype(np.float64)
    joint_lower, joint_upper = load_joint_limits(args.urdf)

    report: dict[str, Any] = {
        "mode": "offline_standard_act_absolute_action_validation",
        "checkpoint": str(args.checkpoint.resolve()),
        "dataset_root": str(args.dataset_root.resolve()),
        "hardware_commands_published": False,
        "ros_node_created": False,
        "observation_semantics": "three RGB cameras plus measured OpenArm16",
        "action_semantics": "absolute OpenArm16 ros2_control position target",
        "hardware_safety_contract": (
            "ACT raw absolute q_target -> elementwise URDF hard-limit projection -> "
            "forward_position_controller absolute q_target"
        ),
        "hardware_safety_changes_action_representation": False,
        "hardware_safety_changes_policy_clock": False,
        "chunk_size": policy.config.chunk_size,
        "n_action_steps": policy.config.n_action_steps,
        "temporal_ensemble_coeff": policy.config.temporal_ensemble_coeff,
        "policy_reset_required_per_episode": True,
        "splits": {},
    }
    all_failures: list[str] = []
    for split in args.splits:
        split_report, failures = validate_split(
            split,
            args.dataset_root / split,
            policy,
            preprocessor,
            postprocessor,
            device,
            train_min,
            train_max,
            joint_lower,
            joint_upper,
            args.stride,
        )
        report["splits"][split] = split_report
        all_failures.extend(f"{split}: {failure}" for failure in failures)
    report["pass"] = not all_failures
    report["failures"] = all_failures
    report["limitation"] = (
        "Teacher-forced held-out observations validate the learned absolute-action "
        "contract and sequence, not physical closed-loop task success."
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"pass": report["pass"], "failures": all_failures}, indent=2))
    print("OFFLINE ONLY: no ROS node or hardware command publisher was created.")
    print(f"REPORT: {args.output.resolve()}")
    if all_failures:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
