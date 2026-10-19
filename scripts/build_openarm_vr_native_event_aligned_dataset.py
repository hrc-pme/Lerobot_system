#!/usr/bin/env python3
"""Build a VR-native LeRobot dataset starting immediately before initial opening.

Only the variable, motionless recording prefix is removed. The original
datasets remain untouched, and no ROS node or hardware publisher is created.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq


SPLITS = {"train": 8, "validation": 1, "test": 1}
IMAGE_KEYS = tuple(f"observation.images.camera{i}" for i in (1, 2, 3))
ARM_INDICES = np.asarray((*range(7), *range(8, 15)), dtype=np.int64)
GRIPPER_INDICES = (7, 15)
OPEN_THRESHOLD_M = 0.008
TASK = "Wipe the table with a cloth and place the cloth in the box"


def read_numeric_episodes(root: Path, split: str) -> tuple[dict, list[dict]]:
    info_path = root / split / "meta" / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(info_path)
    info = json.loads(info_path.read_text(encoding="utf-8"))
    if info["fps"] != 15 or info["total_episodes"] != SPLITS[split]:
        raise ValueError(f"Unexpected {split} fps or episode count")
    features = info["features"]
    if tuple(sorted(key for key in features if key.startswith("observation.images."))) != IMAGE_KEYS:
        raise ValueError(f"Unexpected {split} camera features")
    for key in ("observation.state", "action"):
        if features[key]["shape"] != [16]:
            raise ValueError(f"Expected 16-D {split} {key}")
    for key in IMAGE_KEYS:
        video_files = sorted((root / split / "videos" / key).glob("chunk-*/file-*.mp4"))
        if not video_files:
            raise FileNotFoundError(f"No local {split} video for {key}")

    data_files = sorted((root / split / "data").glob("chunk-*/file-*.parquet"))
    if not data_files:
        raise FileNotFoundError(f"No {split} data parquet files")
    tables = [
        pq.read_table(
            path,
            columns=["episode_index", "frame_index", "observation.state", "action"],
        )
        for path in data_files
    ]
    episode_index = np.concatenate([t["episode_index"].to_numpy() for t in tables])
    frame_index = np.concatenate([t["frame_index"].to_numpy() for t in tables])
    states = np.concatenate(
        [np.asarray(t["observation.state"].to_pylist(), dtype=np.float32) for t in tables]
    )
    actions = np.concatenate(
        [np.asarray(t["action"].to_pylist(), dtype=np.float32) for t in tables]
    )
    if len(states) != info["total_frames"]:
        raise ValueError(f"{split} row count does not match metadata")

    episodes = []
    for episode in range(SPLITS[split]):
        indices = np.flatnonzero(episode_index == episode)
        if not len(indices) or not np.array_equal(frame_index[indices], np.arange(len(indices))):
            raise ValueError(f"{split}/{episode}: missing or reordered frames")
        if not np.array_equal(indices, np.arange(indices[0], indices[-1] + 1)):
            raise ValueError(f"{split}/{episode}: episode rows are not contiguous")
        state = states[indices]
        action = actions[indices]
        open_mask = np.all(action[:, GRIPPER_INDICES] >= OPEN_THRESHOLD_M, axis=1)
        crossings = np.flatnonzero(open_mask)
        if not len(crossings):
            raise ValueError(f"{split}/{episode}: no initial two-gripper opening")
        onset = int(crossings[0])
        either_open = np.flatnonzero(
            np.any(action[:, GRIPPER_INDICES] >= OPEN_THRESHOLD_M, axis=1)
        )
        if int(either_open[0]) != onset:
            raise ValueError(f"{split}/{episode}: grippers did not begin opening together")
        start = onset - 1
        if start < 2 or onset + 30 >= len(indices):
            raise ValueError(f"{split}/{episode}: opening too near a dataset boundary")
        if not np.all(open_mask[onset : onset + 3]):
            raise ValueError(f"{split}/{episode}: opening threshold is not sustained")
        if np.any(state[start, GRIPPER_INDICES] >= OPEN_THRESHOLD_M):
            raise ValueError(f"{split}/{episode}: start observation is already open")
        if np.any(action[start, GRIPPER_INDICES] >= OPEN_THRESHOLD_M):
            raise ValueError(f"{split}/{episode}: start action is already open")
        preopen_arm_drift = float(
            np.max(np.abs(state[:start, ARM_INDICES] - state[0, ARM_INDICES]))
        )
        if preopen_arm_drift > 0.005:
            raise ValueError(
                f"{split}/{episode}: prefix arm drift {preopen_arm_drift:.6f} rad "
                "is not a motionless recording wait"
            )
        episodes.append(
            {
                "split": split,
                "episode_index": episode,
                "source_episode": f"{1 + episode if split == 'train' else 9 if split == 'validation' else 10:03d}",
                "source_row_start": int(indices[0]),
                "source_frames": len(indices),
                "first_two_gripper_open_action_frame": onset,
                "new_source_start_frame": start,
                "removed_prefix_frames": start,
                "retained_frames": len(indices) - start,
                "first_open_action_frame_after_alignment": 1,
                "preopen_arm_max_drift_rad": preopen_arm_drift,
                "start_observed_grippers_m": state[start, GRIPPER_INDICES].astype(float).tolist(),
                "start_target_grippers_m": action[start, GRIPPER_INDICES].astype(float).tolist(),
                "source_start_arm": state[start, ARM_INDICES].astype(float).tolist(),
            }
        )
    return info, episodes


def image_to_uint8_hwc(value: object) -> np.ndarray:
    import torch

    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().numpy()
    image = np.asarray(value)
    if image.shape == (3, 360, 640):
        image = np.transpose(image, (1, 2, 0))
    if image.shape != (360, 640, 3):
        raise ValueError(f"Unexpected decoded image shape: {image.shape}")
    if np.issubdtype(image.dtype, np.floating):
        if not np.isfinite(image).all() or image.min() < -0.001 or image.max() > 1.001:
            raise ValueError("Decoded image must have finite 0..1 values")
        image = np.rint(np.clip(image, 0, 1) * 255).astype(np.uint8)
    elif image.dtype != np.uint8:
        raise ValueError(f"Unexpected decoded image dtype: {image.dtype}")
    return np.ascontiguousarray(image)


def materialize_split(source_root: Path, output_root: Path, split: str, info: dict, episodes: list[dict]) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source = LeRobotDataset(
        repo_id=f"openarm_vr_native_unaligned_{split}", root=source_root / split
    )
    features = {}
    for key in (*IMAGE_KEYS, "observation.state", "action"):
        spec = info["features"][key]
        # The JSON metadata stores shapes as lists, while LeRobot's
        # validate_frame compares ndarray.shape to the feature shape directly.
        # Match the tuple-valued schema used by the original ROS bag converter.
        features[key] = {
            "dtype": spec["dtype"],
            "shape": tuple(spec["shape"]),
            "names": spec["names"],
        }
    target = LeRobotDataset.create(
        repo_id=f"openarm_vr_native_event_aligned_v2_{split}",
        fps=15,
        robot_type="openarm_bimanual",
        features=features,
        root=output_root / split,
        image_writer_processes=4,
        image_writer_threads=4,
    )
    for episode in episodes:
        first = episode["source_row_start"] + episode["new_source_start_frame"]
        stop = episode["source_row_start"] + episode["source_frames"]
        for index in range(first, stop):
            sample = source[index]
            frame = {
                "task": TASK,
                "observation.state": sample["observation.state"].cpu().numpy().astype(np.float32),
                "action": sample["action"].cpu().numpy().astype(np.float32),
            }
            for key in IMAGE_KEYS:
                frame[key] = image_to_uint8_hwc(sample[key])
            target.add_frame(frame)
        target.save_episode()
        print(
            f"{split}/{episode['source_episode']}: removed {episode['removed_prefix_frames']} "
            f"idle frames, retained {episode['retained_frames']}",
            flush=True,
        )
    target.finalize()
    output_info = json.loads(
        (output_root / split / "meta" / "info.json").read_text(encoding="utf-8")
    )
    expected_frames = sum(row["retained_frames"] for row in episodes)
    if (
        output_info["total_episodes"] != len(episodes)
        or output_info["total_frames"] != expected_frames
    ):
        raise RuntimeError(f"{split}: output episode/frame count verification failed")
    numeric_files = sorted((output_root / split / "data").glob("chunk-*/file-*.parquet"))
    numeric_tables = [
        pq.read_table(path, columns=["episode_index", "frame_index", "observation.state", "action"])
        for path in numeric_files
    ]
    actual_episode = np.concatenate([table["episode_index"].to_numpy() for table in numeric_tables])
    actual_frame = np.concatenate([table["frame_index"].to_numpy() for table in numeric_tables])
    actual_actions = np.concatenate(
        [np.asarray(table["action"].to_pylist(), dtype=np.float32) for table in numeric_tables]
    )
    if len(actual_frame) != expected_frames:
        raise RuntimeError(f"{split}: output parquet row count verification failed")
    for episode in episodes:
        indices = np.flatnonzero(actual_episode == episode["episode_index"])
        if len(indices) != episode["retained_frames"] or not np.array_equal(
            actual_frame[indices], np.arange(len(indices))
        ):
            raise RuntimeError(f"{split}/{episode['source_episode']}: output frame indices invalid")
        if np.any(actual_actions[indices[0], GRIPPER_INDICES] >= OPEN_THRESHOLD_M) or not np.all(
            actual_actions[indices[1], GRIPPER_INDICES] >= OPEN_THRESHOLD_M
        ):
            raise RuntimeError(f"{split}/{episode['source_episode']}: first-open alignment invalid")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path("Dataset/Converted_dataset/openarm_vr_native_baseline_v1"),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("Dataset/Converted_dataset/openarm_vr_native_event_aligned_v2"),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("output/openarm_native_baseline_audit_v1/event_alignment_v2.json"),
    )
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()
    source_root = args.source_root.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    report_path = args.report.expanduser().resolve()
    if source_root == output_root or source_root in output_root.parents:
        raise ValueError("Output must not overwrite or nest inside the source dataset")
    if not args.audit_only and output_root.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output_root}")
    if report_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing report: {report_path}")

    split_data = {}
    for split in SPLITS:
        split_data[split] = read_numeric_episodes(source_root, split)
    report = {
        "mode": "offline_vr_native_initial_open_event_alignment",
        "source_root": str(source_root),
        "output_root": str(output_root),
        "materialized": not args.audit_only,
        "fps": 15,
        "first_open_action_frame_after_alignment": 1,
        "split_integrity": "train=001..008, validation=009, test=010; unchanged",
        "hardware_commands_published": False,
        "episodes": [row for split in SPLITS for row in split_data[split][1]],
    }
    if not args.audit_only:
        output_root.mkdir(parents=True)
        for split, (info, episodes) in split_data.items():
            materialize_split(source_root, output_root, split, info, episodes)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        f"EVENT ALIGNMENT {'AUDIT' if args.audit_only else 'BUILD'} PASS: "
        f"{len(report['episodes'])} episodes, first open at new action frame 1; "
        "no ROS publisher created."
    )
    print(f"REPORT: {report_path}")


if __name__ == "__main__":
    main()
