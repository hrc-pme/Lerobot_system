#!/usr/bin/env python3
"""Offline validation for an ACT policy and matching LeRobot dataset.

This tool never initializes ROS and never publishes robot commands.  It feeds
recorded observations into an ACT checkpoint, compares the predicted action
chunk with the recorded OpenArm action, and writes machine-readable reports.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import torch

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import PolicyProcessorPipeline


DEFAULT_CHECKPOINT = Path(
    "/home/hrc/Lerobot_system/output/openarm_act_80k/checkpoints/080000/pretrained_model"
)
DEFAULT_DATASET = Path(
    "/home/hrc/Lerobot_system/Dataset/Converted_dataset/openarm_wipe_30ep"
)
JOINT_NAMES = [
    *[f"openarm_left_joint{i}" for i in range(1, 8)],
    "openarm_left_finger_joint1",
    *[f"openarm_right_joint{i}" for i in range(1, 8)],
    "openarm_right_finger_joint1",
]
KOCH_JOINT_NAMES = [
    *[f"left_joint{i}" for i in range(1, 6)],
    "left_joint_gripper",
    *[f"right_joint{i}" for i in range(1, 6)],
    "right_joint_gripper",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate an ACT checkpoint without ROS or hardware."
    )
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--repo-id", default="openarm_wipe_30ep")
    parser.add_argument(
        "--episodes",
        type=int,
        nargs="*",
        help="Episode indices to evaluate (default: all episodes).",
    )
    parser.add_argument(
        "--samples-per-episode",
        type=int,
        default=10,
        help="Evenly spaced inference points per episode (default: 10).",
    )
    parser.add_argument(
        "--chunk-steps",
        type=int,
        default=0,
        help="Number of predicted future steps to compare; 0 uses model chunk_size.",
    )
    parser.add_argument(
        "--device", choices=("auto", "cuda", "cpu"), default="auto"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/openarm_act_80k/offline_eval_080000"),
    )
    parser.add_argument(
        "--fail-mae",
        type=float,
        default=None,
        help="Return exit code 2 when overall chunk MAE exceeds this value (rad).",
    )
    return parser.parse_args()


def choose_device(requested: str) -> torch.device:
    if requested == "auto":
        requested = "cuda" if torch.cuda.is_available() else "cpu"
    if requested == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but torch.cuda.is_available() is false")
    return torch.device(requested)


def reset_if_supported(obj: object) -> None:
    reset = getattr(obj, "reset", None)
    if callable(reset):
        reset()


def postprocess_chunk(
    postprocessor: PolicyProcessorPipeline, chunk: torch.Tensor
) -> torch.Tensor:
    batch, steps, action_dim = chunk.shape
    flat = chunk.reshape(batch * steps, action_dim)
    result = postprocessor({"action": flat})["action"]
    return result.reshape(batch, steps, action_dim)


def episode_sample_indices(start: int, stop: int, count: int) -> list[int]:
    if count <= 0:
        raise ValueError("--samples-per-episode must be positive")
    length = stop - start
    if length <= 0:
        return []
    actual_count = min(count, length)
    return sorted(
        set(np.linspace(start, stop - 1, actual_count, dtype=np.int64).tolist())
    )


def make_observation(
    sample: dict, policy: ACTPolicy, device: torch.device
) -> dict[str, torch.Tensor]:
    observation = {}
    for key in policy.config.input_features:
        if key not in sample:
            raise KeyError(f"Dataset sample is missing model input {key!r}")
        value = sample[key]
        if not isinstance(value, torch.Tensor):
            value = torch.as_tensor(value)
        observation[key] = value.unsqueeze(0).to(device, non_blocking=True)
    return observation


def write_plot(
    output: Path, per_joint_mae: np.ndarray, joint_names: list[str]
) -> str | None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    figure, axis = plt.subplots(figsize=(12, 5))
    axis.bar(np.arange(len(joint_names)), per_joint_mae)
    axis.set_xticks(np.arange(len(joint_names)))
    axis.set_xticklabels(joint_names, rotation=70, ha="right", fontsize=8)
    axis.set_ylabel("MAE (rad or gripper position unit)")
    axis.set_title("OpenArm ACT offline action error by joint")
    axis.grid(axis="y", alpha=0.3)
    figure.tight_layout()
    plot_path = output / "per_joint_mae.png"
    figure.savefig(plot_path, dpi=160)
    plt.close(figure)
    return str(plot_path)


def main() -> int:
    args = parse_args()
    checkpoint = args.checkpoint.expanduser().resolve()
    dataset_root = args.dataset.expanduser().resolve()
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    if not (checkpoint / "model.safetensors").is_file():
        raise FileNotFoundError(f"Checkpoint is incomplete: {checkpoint}")
    if not (dataset_root / "meta" / "info.json").is_file():
        raise FileNotFoundError(f"LeRobot dataset not found: {dataset_root}")

    device = choose_device(args.device)
    print(f"Loading dataset: {dataset_root}")
    dataset = LeRobotDataset(repo_id=args.repo_id, root=dataset_root)
    print(f"Loading ACT checkpoint on {device}: {checkpoint}")
    policy = ACTPolicy.from_pretrained(str(checkpoint)).to(device).eval()
    preprocessor = PolicyProcessorPipeline.from_pretrained(
        str(checkpoint), config_filename="policy_preprocessor.json"
    )
    postprocessor = PolicyProcessorPipeline.from_pretrained(
        str(checkpoint), config_filename="policy_postprocessor.json"
    )

    input_state = policy.config.input_features.get("observation.state")
    output_action = policy.config.output_features.get("action")
    if input_state is None or len(tuple(input_state.shape)) != 1:
        raise ValueError(f"Expected a 1-D observation.state, got {input_state}")
    if output_action is None or len(tuple(output_action.shape)) != 1:
        raise ValueError(f"Expected a 1-D action, got {output_action}")
    state_dim = int(input_state.shape[0])
    action_dim = int(output_action.shape[0])
    if state_dim != action_dim:
        raise ValueError(
            f"State/action dimensions differ: state={state_dim}, action={action_dim}"
        )
    if action_dim == 16:
        joint_names = JOINT_NAMES
    elif action_dim == 12:
        joint_names = KOCH_JOINT_NAMES
    else:
        joint_names = [f"action_{index}" for index in range(action_dim)]

    all_episode_ids = list(range(dataset.num_episodes))
    episode_ids = args.episodes if args.episodes is not None else all_episode_ids
    invalid = sorted(set(episode_ids) - set(all_episode_ids))
    if invalid:
        raise ValueError(f"Episode indices out of range: {invalid}")
    if not episode_ids:
        raise ValueError("No episodes selected")

    requested_chunk_steps = args.chunk_steps or int(policy.config.chunk_size)
    requested_chunk_steps = min(requested_chunk_steps, int(policy.config.chunk_size))
    if requested_chunk_steps <= 0:
        raise ValueError("--chunk-steps must be positive or zero")

    absolute_errors: list[np.ndarray] = []
    first_step_errors: list[np.ndarray] = []
    squared_error_sum = np.zeros(action_dim, dtype=np.float64)
    error_count = 0
    sample_rows: list[dict[str, float | int]] = []
    episode_errors: dict[int, list[np.ndarray]] = defaultdict(list)

    with torch.inference_mode():
        for episode_number, episode_id in enumerate(episode_ids, start=1):
            meta = dataset.meta.episodes[episode_id]
            start = int(meta["dataset_from_index"])
            stop = int(meta["dataset_to_index"])
            indices = episode_sample_indices(
                start, stop, args.samples_per_episode
            )
            reset_if_supported(policy)
            reset_if_supported(preprocessor)
            reset_if_supported(postprocessor)

            for sample_number, index in enumerate(indices, start=1):
                sample = dataset[index]
                observation = make_observation(sample, policy, device)
                observation = preprocessor(observation)
                predicted_normalized = policy.predict_action_chunk(observation)
                if predicted_normalized.ndim != 3:
                    raise RuntimeError(
                        f"Expected ACT output [B,T,{action_dim}], got "
                        f"{predicted_normalized.shape}"
                    )
                predicted = postprocess_chunk(
                    postprocessor, predicted_normalized
                )[0].detach().cpu()

                available = min(requested_chunk_steps, stop - index, predicted.shape[0])
                targets = torch.stack(
                    [dataset.hf_dataset[index + offset]["action"] for offset in range(available)]
                ).float()
                errors = (predicted[:available] - targets).numpy()
                abs_errors = np.abs(errors)
                absolute_errors.append(abs_errors)
                first_step_errors.append(abs_errors[0])
                squared_error_sum += np.square(errors).sum(axis=0)
                error_count += available
                episode_errors[episode_id].append(abs_errors)
                sample_rows.append(
                    {
                        "episode": episode_id,
                        "dataset_index": index,
                        "frame_index": int(sample["frame_index"]),
                        "compared_steps": available,
                        "first_step_mae": float(abs_errors[0].mean()),
                        "chunk_mae": float(abs_errors.mean()),
                        "chunk_max_error": float(abs_errors.max()),
                    }
                )
                print(
                    f"\rEpisode {episode_number}/{len(episode_ids)}, "
                    f"sample {sample_number}/{len(indices)}, chunk MAE={abs_errors.mean():.4f}",
                    end="",
                    flush=True,
                )
    print()

    all_abs = np.concatenate(absolute_errors, axis=0)
    first_abs = np.stack(first_step_errors)
    per_joint_mae = all_abs.mean(axis=0)
    per_joint_rmse = np.sqrt(squared_error_sum / error_count)
    per_joint_max = all_abs.max(axis=0)
    episode_summary = {}
    for episode_id, chunks in episode_errors.items():
        values = np.concatenate(chunks, axis=0)
        episode_summary[str(episode_id)] = {
            "mae": float(values.mean()),
            "p95_absolute_error": float(np.quantile(values, 0.95)),
            "max_absolute_error": float(values.max()),
        }

    summary = {
        "checkpoint": str(checkpoint),
        "dataset": str(dataset_root),
        "device": str(device),
        "episodes": episode_ids,
        "inference_samples": len(sample_rows),
        "compared_action_steps": int(error_count),
        "model_chunk_size": int(policy.config.chunk_size),
        "evaluated_chunk_steps": requested_chunk_steps,
        "all_outputs_finite": bool(np.isfinite(all_abs).all()),
        "chunk": {
            "mae": float(all_abs.mean()),
            "rmse": float(np.sqrt(np.square(all_abs).mean())),
            "p95_absolute_error": float(np.quantile(all_abs, 0.95)),
            "max_absolute_error": float(all_abs.max()),
        },
        "first_step": {
            "mae": float(first_abs.mean()),
            "rmse": float(np.sqrt(np.square(first_abs).mean())),
            "p95_absolute_error": float(np.quantile(first_abs, 0.95)),
            "max_absolute_error": float(first_abs.max()),
        },
        "per_episode": episode_summary,
        "note": (
            "These episodes were also used for training. This is a reproduction/sanity "
            "test, not an unbiased held-out success estimate."
        ),
    }

    with (output / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, ensure_ascii=False)
    with (output / "per_joint.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("joint", "mae", "rmse", "max_absolute_error"))
        for name, mae, rmse, maximum in zip(
            joint_names, per_joint_mae, per_joint_rmse, per_joint_max, strict=True
        ):
            writer.writerow((name, float(mae), float(rmse), float(maximum)))
    with (output / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(sample_rows[0]))
        writer.writeheader()
        writer.writerows(sample_rows)
    plot_path = write_plot(output, per_joint_mae, joint_names)

    print(json.dumps(summary["chunk"], indent=2))
    print(f"First-step MAE: {summary['first_step']['mae']:.6f}")
    print(f"Report: {output / 'summary.json'}")
    print(f"Per-joint metrics: {output / 'per_joint.csv'}")
    if plot_path:
        print(f"Plot: {plot_path}")

    if not summary["all_outputs_finite"]:
        print("FAIL: model produced NaN or Inf", file=sys.stderr)
        return 3
    if args.fail_mae is not None and summary["chunk"]["mae"] > args.fail_mae:
        print(
            f"FAIL: chunk MAE {summary['chunk']['mae']:.6f} exceeds {args.fail_mae}",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
