#!/usr/bin/env python3
"""Build an event-aligned copy of the absolute-command OpenArm dataset.

The only transformation is removal of the variable, motionless prefix before
the initial two-gripper opening.  Numeric labels and decoded camera frames are
copied from the source dataset.  Absolute-command provenance is retained so
every output action remains auditable against the original rosbag command.

No ROS node, publisher, policy, or hardware process is created.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow.parquet as pq

from build_openarm_vr_native_event_aligned_dataset import (
    IMAGE_KEYS,
    OPEN_THRESHOLD_M,
    SPLITS,
    TASK,
    image_to_uint8_hwc,
    read_numeric_episodes,
)


SOURCE_ROOT = Path("Dataset/Converted_dataset/openarm_vr_native_absolute_v1")
OUTPUT_ROOT = Path(
    "Dataset/Converted_dataset/openarm_vr_native_absolute_event_aligned_v1"
)
REPORT = Path("output/openarm_native_absolute_audit_v1/event_alignment_v1.json")
REPO_PREFIX = "openarm_vr_native_absolute_event_aligned_v1"


def read_provenance(root: Path) -> list[dict[str, Any]]:
    path = root / "absolute_command_provenance.jsonl"
    if not path.is_file():
        raise FileNotFoundError(path)
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    rows.sort(key=lambda row: (row["output_episode_index"], row["output_frame_index"]))
    return rows


def read_output_numeric(root: Path) -> dict[str, np.ndarray]:
    paths = sorted((root / "data").glob("chunk-*/*.parquet"))
    if not paths:
        raise FileNotFoundError(root / "data")
    table = pq.read_table(
        [str(path) for path in paths],
        columns=["episode_index", "frame_index", "observation.state", "action"],
    )
    return {
        "episode_index": np.asarray(table["episode_index"].to_numpy(), dtype=np.int64),
        "frame_index": np.asarray(table["frame_index"].to_numpy(), dtype=np.int64),
        "state": np.asarray(table["observation.state"].to_pylist(), dtype=np.float32),
        "action": np.asarray(table["action"].to_pylist(), dtype=np.float32),
    }


def materialize_split(
    source_root: Path,
    output_root: Path,
    split: str,
    info: dict[str, Any],
    episodes: list[dict[str, Any]],
) -> dict[str, Any]:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source_split = source_root / split
    output_split = output_root / split
    source_provenance = read_provenance(source_split)
    if len(source_provenance) != int(info["total_frames"]):
        raise RuntimeError(f"{split}: source provenance length mismatch")

    source = LeRobotDataset(
        repo_id=f"openarm_vr_native_absolute_source_{split}", root=source_split
    )
    features: dict[str, dict[str, Any]] = {}
    for key in (*IMAGE_KEYS, "observation.state", "action"):
        spec = info["features"][key]
        features[key] = {
            "dtype": spec["dtype"],
            "shape": tuple(spec["shape"]),
            "names": spec["names"],
        }
    target = LeRobotDataset.create(
        repo_id=f"{REPO_PREFIX}_{split}",
        fps=15,
        robot_type="openarm_bimanual",
        features=features,
        root=output_split,
        image_writer_processes=4,
        image_writer_threads=4,
    )

    output_provenance: list[dict[str, Any]] = []
    expected_states: list[np.ndarray] = []
    expected_actions: list[np.ndarray] = []
    for output_episode, episode in enumerate(episodes):
        if output_episode != int(episode["episode_index"]):
            raise RuntimeError(f"{split}: episode ordering changed")
        first = int(episode["source_row_start"] + episode["new_source_start_frame"])
        stop = int(episode["source_row_start"] + episode["source_frames"])
        for output_frame, source_index in enumerate(range(first, stop)):
            sample = source[source_index]
            state = sample["observation.state"].cpu().numpy().astype(np.float32)
            action = sample["action"].cpu().numpy().astype(np.float32)
            frame: dict[str, Any] = {
                "task": TASK,
                "observation.state": state,
                "action": action,
            }
            for key in IMAGE_KEYS:
                frame[key] = image_to_uint8_hwc(sample[key])
            target.add_frame(frame)
            expected_states.append(state.copy())
            expected_actions.append(action.copy())

            record = dict(source_provenance[source_index])
            if (
                int(record["output_episode_index"]) != output_episode
                or int(record["output_frame_index"])
                != int(episode["new_source_start_frame"]) + output_frame
            ):
                raise RuntimeError(
                    f"{split}/{episode['source_episode']}: source provenance ordering mismatch"
                )
            record["source_dataset_episode_index"] = int(record["output_episode_index"])
            record["source_dataset_frame_index"] = int(record["output_frame_index"])
            record["output_episode_index"] = output_episode
            record["output_frame_index"] = output_frame
            record["event_alignment"] = "prefix_trim_only_no_relabel"
            output_provenance.append(record)
        target.save_episode()
        print(
            f"{split}/{episode['source_episode']}: removed "
            f"{episode['removed_prefix_frames']} idle frames, retained "
            f"{episode['retained_frames']}",
            flush=True,
        )
    target.finalize()

    numeric = read_output_numeric(output_split)
    expected_state = np.stack(expected_states)
    expected_action = np.stack(expected_actions)
    expected_frames = len(expected_action)
    if numeric["state"].shape != expected_state.shape or numeric["action"].shape != expected_action.shape:
        raise RuntimeError(f"{split}: output numeric shape mismatch")
    state_error = float(np.max(np.abs(numeric["state"] - expected_state)))
    action_error = float(np.max(np.abs(numeric["action"] - expected_action)))
    if state_error != 0.0 or action_error != 0.0:
        raise RuntimeError(
            f"{split}: copied numeric data changed: state={state_error}, action={action_error}"
        )
    if len(output_provenance) != expected_frames:
        raise RuntimeError(f"{split}: output provenance length mismatch")
    provenance_action_error = max(
        float(np.max(np.abs(np.asarray(row["action"], dtype=np.float32) - expected_action[index])))
        for index, row in enumerate(output_provenance)
    )
    if provenance_action_error != 0.0:
        raise RuntimeError(f"{split}: provenance action changed")

    provenance_path = output_split / "absolute_command_provenance.jsonl"
    provenance_path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in output_provenance),
        encoding="utf-8",
    )
    source_contract = json.loads(
        (source_split / "absolute_command_contract.json").read_text(encoding="utf-8")
    )
    source_contract["frames"] = expected_frames
    source_contract["event_alignment"] = {
        "operation": "prefix_trim_only_no_relabel",
        "open_initiation_threshold_m": OPEN_THRESHOLD_M,
        "sustained_samples": 3,
        "new_frame_zero": "one source frame before simultaneous gripper opening initiation",
        "new_frame_one": "first simultaneous gripper opening initiation action",
        "interpolation_added": False,
        "synthetic_labels_added": False,
    }
    (output_split / "absolute_command_contract.json").write_text(
        json.dumps(source_contract, indent=2) + "\n", encoding="utf-8"
    )
    alignment_contract = {
        "schema_version": 1,
        "source_split": str(source_split.resolve()),
        "output_split": str(output_split.resolve()),
        "operation": "prefix_trim_only_no_relabel",
        "open_initiation_threshold_m": OPEN_THRESHOLD_M,
        "sustained_samples": 3,
        "maximum_allowed_removed_prefix_arm_drift_rad": 0.005,
        "state_copy_max_error": state_error,
        "action_copy_max_error": action_error,
        "provenance_action_max_error": provenance_action_error,
        "episodes": episodes,
    }
    (output_split / "event_alignment_contract.json").write_text(
        json.dumps(alignment_contract, indent=2) + "\n", encoding="utf-8"
    )
    return {
        "frames": expected_frames,
        "episodes": len(episodes),
        "state_copy_max_error": state_error,
        "action_copy_max_error": action_error,
        "provenance_action_max_error": provenance_action_error,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=SOURCE_ROOT)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--report", type=Path, default=REPORT)
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

    split_data: dict[str, tuple[dict[str, Any], list[dict[str, Any]]]] = {}
    for split in SPLITS:
        split_data[split] = read_numeric_episodes(source_root, split)

    report: dict[str, Any] = {
        "mode": "offline_absolute_command_initial_open_event_alignment",
        "source_root": str(source_root),
        "output_root": str(output_root),
        "materialized": not args.audit_only,
        "fps": 15,
        "operation": "prefix_trim_only_no_relabel",
        "open_initiation_threshold_m": OPEN_THRESHOLD_M,
        "first_open_action_frame_after_alignment": 1,
        "split_integrity": "train=001..008, validation=009, test=010; unchanged",
        "action_semantics": "absolute ros2_control position target",
        "future_controller_commands_used": False,
        "interpolation_added": False,
        "synthetic_labels_added": False,
        "hardware_commands_published": False,
        "episodes": [row for split in SPLITS for row in split_data[split][1]],
    }
    if not args.audit_only:
        output_root.mkdir(parents=True)
        report["materialized_splits"] = {
            split: materialize_split(source_root, output_root, split, info, episodes)
            for split, (info, episodes) in split_data.items()
        }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        f"ABSOLUTE EVENT ALIGNMENT {'AUDIT' if args.audit_only else 'BUILD'} PASS: "
        f"{len(report['episodes'])} episodes; prefix trim only; exact absolute labels retained."
    )
    print(f"REPORT: {report_path}")
    print("OFFLINE ONLY: no ROS node or hardware command publisher was created.")


if __name__ == "__main__":
    main()
