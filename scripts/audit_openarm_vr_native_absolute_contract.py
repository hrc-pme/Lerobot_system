#!/usr/bin/env python3
"""Offline audit for the OpenArm16 absolute-controller LeRobot dataset.

The audit reads existing rosbag and parquet files only. It creates no ROS node,
publisher, controller command, model, or hardware process.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sqlite3
import sys
from typing import Any

import numpy as np
import pyarrow.parquet as pq
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


SCRIPT = Path(__file__).resolve().parent / "ros2_to_lerobot_openarm.py"
SPEC = importlib.util.spec_from_file_location("openarm_absolute_converter", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CONVERTER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CONVERTER
SPEC.loader.exec_module(CONVERTER)

ARM = np.asarray((*range(7), *range(8, 15)), dtype=np.int64)
GRIPPERS = np.asarray((7, 15), dtype=np.int64)
CAMERA_KEYS = (
    "observation.images.camera1",
    "observation.images.camera2",
    "observation.images.camera3",
)
EXPECTED_SPLITS = {
    "train": tuple(f"{index:03d}" for index in range(1, 9)),
    "validation": ("009",),
    "test": ("010",),
}


def first_sustained(mask: np.ndarray, samples: int = 3) -> int | None:
    for index in range(max(0, len(mask) - samples + 1)):
        if bool(np.all(mask[index : index + samples])):
            return index
    return None


def read_numeric_dataset(root: Path) -> dict[str, np.ndarray]:
    paths = sorted((root / "data").glob("chunk-*/*.parquet"))
    if not paths:
        raise FileNotFoundError(root / "data")
    table = pq.read_table(
        [str(path) for path in paths],
        columns=["episode_index", "frame_index", "observation.state", "action"],
    )
    values = {
        "episode_index": np.asarray(table["episode_index"].to_numpy(), dtype=np.int64),
        "frame_index": np.asarray(table["frame_index"].to_numpy(), dtype=np.int64),
        "state": np.asarray(table["observation.state"].to_pylist(), dtype=np.float32),
        "action": np.asarray(table["action"].to_pylist(), dtype=np.float32),
    }
    order = np.lexsort((values["frame_index"], values["episode_index"]))
    return {key: value[order] for key, value in values.items()}


def read_provenance(root: Path) -> list[dict[str, Any]]:
    path = root / "absolute_command_provenance.jsonl"
    if not path.is_file():
        raise FileNotFoundError(path)
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    return sorted(
        records,
        key=lambda row: (row["output_episode_index"], row["output_frame_index"]),
    )


def bag_topic_timestamps(bag: Path, topics: list[str]) -> dict[str, set[int]]:
    db = CONVERTER.db_path_for_bag(bag)
    connection = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        result: dict[str, set[int]] = {}
        for topic in topics:
            rows = connection.execute(
                """
                SELECT messages.timestamp
                FROM messages JOIN topics ON messages.topic_id = topics.id
                WHERE topics.name = ?
                """,
                (topic,),
            ).fetchall()
            result[topic] = {int(row[0]) for row in rows}
    finally:
        connection.close()
    return result


def read_raw_numeric(
    bag: Path,
) -> tuple[dict[int, np.ndarray], dict[str, dict[int, np.ndarray]]]:
    state: dict[int, np.ndarray] = {}
    commands: dict[str, dict[int, np.ndarray]] = {
        topic: {} for topic in CONVERTER.COMMAND_TOPICS
    }
    topics = [CONVERTER.STATE_TOPIC, *CONVERTER.COMMAND_TOPICS]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        if topic == CONVERTER.STATE_TOPIC:
            value = CONVERTER.state_vector(deserialize_message(payload, JointState))
            state[int(timestamp)] = value
        else:
            expected = 7 if "forward_position" in topic else 1
            value = CONVERTER.command_vector(
                deserialize_message(payload, Float64MultiArray), expected, topic
            )
            commands[topic][int(timestamp)] = value
    return state, commands


def max_or_zero(values: list[float]) -> float:
    return max(values) if values else 0.0


def audit_split(
    split: str,
    dataset_root: Path,
    raw_root: Path,
    max_camera_skew_ns: int,
    max_command_age_ns: int,
) -> tuple[dict[str, Any], list[str]]:
    expected_bags = EXPECTED_SPLITS[split]
    info = json.loads((dataset_root / "meta" / "info.json").read_text(encoding="utf-8"))
    contract = json.loads(
        (dataset_root / "absolute_command_contract.json").read_text(encoding="utf-8")
    )
    numeric = read_numeric_dataset(dataset_root)
    provenance = read_provenance(dataset_root)
    failures: list[str] = []

    features = info["features"]
    if features["observation.state"]["shape"] != [16]:
        failures.append("observation.state is not 16D")
    if features["action"]["shape"] != [16]:
        failures.append("action is not 16D")
    if features["observation.state"]["names"] != list(CONVERTER.JOINT_ORDER):
        failures.append("observation.state joint ordering mismatch")
    if features["action"]["names"] != list(CONVERTER.JOINT_ORDER):
        failures.append("action joint ordering mismatch")
    if tuple(sorted(key for key in features if key.startswith("observation.images."))) != CAMERA_KEYS:
        failures.append("camera feature set mismatch")
    if contract["controller_command_sampling"] != "previous":
        failures.append("controller command sampling is not previous/ZOH")
    if contract["future_controller_commands_used"] is not False:
        failures.append("contract does not forbid future controller commands")

    rows = len(numeric["action"])
    if numeric["state"].shape != (rows, 16) or numeric["action"].shape != (rows, 16):
        failures.append(f"numeric shape mismatch: state={numeric['state'].shape}, action={numeric['action'].shape}")
    if len(provenance) != rows:
        failures.append(f"provenance rows {len(provenance)} != dataset rows {rows}")
        return {"frames": rows}, failures
    if int(info["total_episodes"]) != len(expected_bags):
        failures.append("episode count does not match frozen split")

    action_errors: list[float] = []
    state_errors: list[float] = []
    provenance_action_errors: list[float] = []
    command_ages: list[float] = []
    camera_skews: list[float] = []
    sequence_rows: list[dict[str, Any]] = []
    raw_cache: dict[str, tuple[dict[int, np.ndarray], dict[str, dict[int, np.ndarray]]]] = {}
    camera_cache: dict[str, dict[str, set[int]]] = {}

    for row_index, record in enumerate(provenance):
        episode = int(numeric["episode_index"][row_index])
        frame = int(numeric["frame_index"][row_index])
        if (record["output_episode_index"], record["output_frame_index"]) != (episode, frame):
            failures.append(f"row {row_index}: provenance episode/frame mismatch")
            break
        bag_name = str(record["source_bag"])
        if episode >= len(expected_bags) or bag_name != expected_bags[episode]:
            failures.append(f"row {row_index}: bag/split mapping mismatch: {bag_name}")
            break
        bag = raw_root / bag_name
        if bag_name not in raw_cache:
            raw_cache[bag_name] = read_raw_numeric(bag)
            camera_cache[bag_name] = bag_topic_timestamps(
                bag, list(record["camera_sources"])
            )
        raw_state, raw_commands = raw_cache[bag_name]
        command_parts = []
        for topic in CONVERTER.COMMAND_TOPICS:
            source = record["command_sources"][topic]
            if source["sampling"] != "previous":
                failures.append(f"row {row_index}: {topic} was not ZOH sampled")
                continue
            source_time = int(source["source_timestamp_ns"])
            dataset_time = int(record["dataset_timestamp_ns"])
            age = dataset_time - source_time
            command_ages.append(age / 1e9)
            if age != int(source["age_ns"]) or age < 0 or age > max_command_age_ns:
                failures.append(f"row {row_index}: invalid {topic} command age {age} ns")
            try:
                command_parts.append(raw_commands[topic][source_time])
            except KeyError:
                failures.append(f"row {row_index}: command timestamp absent from raw bag")
        if len(command_parts) == 4:
            raw_action = np.concatenate(command_parts).astype(np.float32, copy=False)
            action_errors.append(float(np.max(np.abs(raw_action - numeric["action"][row_index]))))
        provenance_action = np.asarray(record["action"], dtype=np.float32)
        provenance_action_errors.append(
            float(np.max(np.abs(provenance_action - numeric["action"][row_index])))
        )

        state_source = record["state_source"]
        before_time = int(state_source["before_timestamp_ns"])
        after_time = int(state_source["after_timestamp_ns"])
        dataset_time = int(record["dataset_timestamp_ns"])
        try:
            before = raw_state[before_time]
            after = raw_state[after_time]
        except KeyError:
            failures.append(f"row {row_index}: state bracket absent from raw bag")
        else:
            if after_time == before_time:
                expected_state = before
            else:
                alpha = (dataset_time - before_time) / (after_time - before_time)
                expected_state = (before + (after - before) * alpha).astype(np.float32)
            state_errors.append(
                float(np.max(np.abs(expected_state - numeric["state"][row_index])))
            )

        for topic, source in record["camera_sources"].items():
            source_time = int(source["source_timestamp_ns"])
            skew = int(source["skew_ns"])
            camera_skews.append(abs(skew) / 1e9)
            if source_time not in camera_cache[bag_name][topic]:
                failures.append(f"row {row_index}: camera timestamp absent from raw bag")
            if abs(skew) > max_camera_skew_ns:
                failures.append(f"row {row_index}: camera skew exceeds bound")

    if max_or_zero(action_errors) > 1e-7:
        failures.append("dataset action differs from exact raw controller command")
    if max_or_zero(provenance_action_errors) > 0.0:
        failures.append("dataset action differs from converter provenance")
    if max_or_zero(state_errors) > 1e-6:
        failures.append("dataset state differs from measured-state interpolation")

    for episode, bag_name in enumerate(expected_bags):
        indices = np.flatnonzero(numeric["episode_index"] == episode)
        if not len(indices):
            failures.append(f"{bag_name}: missing episode rows")
            continue
        frames = numeric["frame_index"][indices]
        if not np.array_equal(frames, np.arange(len(frames))):
            failures.append(f"{bag_name}: nonconsecutive frame ordering")
        actions = numeric["action"][indices]
        both_open = np.all(actions[:, GRIPPERS] >= 0.035, axis=1)
        initial_open = first_sustained(both_open)
        right_close = None
        left_close = None
        arm_progress = 0.0
        later_progress = 0.0
        if initial_open is None:
            failures.append(f"{bag_name}: no sustained initial two-gripper opening")
        else:
            right_close_offset = first_sustained(actions[initial_open:, 15] <= 0.030)
            left_close_offset = first_sustained(actions[initial_open:, 7] <= 0.030)
            right_close = None if right_close_offset is None else initial_open + right_close_offset
            left_close = None if left_close_offset is None else initial_open + left_close_offset
            if right_close is None:
                failures.append(f"{bag_name}: no sustained right-gripper close after opening")
            if left_close is None:
                failures.append(f"{bag_name}: no sustained left-gripper close after opening")
            arm_progress = float(np.max(np.abs(actions[initial_open:, ARM] - actions[initial_open, ARM])))
            if arm_progress < 0.05:
                failures.append(f"{bag_name}: insufficient arm progression after opening")
            if right_close is not None:
                later_progress = float(np.max(np.abs(actions[right_close:, ARM] - actions[right_close, ARM])))
                if later_progress < 0.05:
                    failures.append(f"{bag_name}: insufficient later task progression")
        full_chunks = len(actions) // 30
        queued = actions[: full_chunks * 30].reshape(full_chunks, 30, 16).reshape(-1, 16)
        queue_match = bool(np.array_equal(queued, actions[: len(queued)]))
        if full_chunks < 1 or not queue_match:
            failures.append(f"{bag_name}: 30-step chunk sequence cannot be read in order")
        arm_jump = float(np.max(np.abs(np.diff(actions[:, ARM], axis=0))))
        gripper_jump = float(np.max(np.abs(np.diff(actions[:, GRIPPERS], axis=0))))
        if arm_jump > 0.20:
            failures.append(f"{bag_name}: arm action jump {arm_jump:.6f} exceeds 0.20 rad")
        if gripper_jump > 0.02:
            failures.append(f"{bag_name}: gripper action jump {gripper_jump:.6f} exceeds 0.02 m")
        sequence_rows.append(
            {
                "source_bag": bag_name,
                "frames": int(len(actions)),
                "initial_open_frame": initial_open,
                "right_close_frame": right_close,
                "left_close_frame": left_close,
                "post_open_arm_progress_rad": arm_progress,
                "later_arm_progress_rad": later_progress,
                "max_arm_action_jump_rad": arm_jump,
                "max_gripper_action_jump_m": gripper_jump,
                "full_30_step_chunks": full_chunks,
                "queue_reconstruction_exact": queue_match,
            }
        )

    report = {
        "dataset_root": str(dataset_root.resolve()),
        "source_bags": list(expected_bags),
        "episodes": int(info["total_episodes"]),
        "frames": rows,
        "fps": int(info["fps"]),
        "state_shape": list(numeric["state"].shape[1:]),
        "action_shape": list(numeric["action"].shape[1:]),
        "joint_order": list(CONVERTER.JOINT_ORDER),
        "action_semantics": "absolute ros2_control position target",
        "command_sampling": "zero_order_hold_previous",
        "future_controller_commands_used": False,
        "max_raw_action_comparison_error": max_or_zero(action_errors),
        "max_provenance_action_error": max_or_zero(provenance_action_errors),
        "max_measured_state_interpolation_error": max_or_zero(state_errors),
        "max_command_age_sec": max_or_zero(command_ages),
        "max_camera_skew_sec": max_or_zero(camera_skews),
        "chunk_size_audited": 30,
        "episodes_sequence_audit": sequence_rows,
        "pass": not failures,
        "failures": failures,
    }
    return report, failures


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("Dataset/Converted_dataset/openarm_vr_native_absolute_v1"),
    )
    parser.add_argument(
        "--raw-root",
        type=Path,
        default=Path("Dataset/Raw_rosbag/openarm_vr_native_baseline_v1"),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report: dict[str, Any] = {
        "mode": "offline_openarm_absolute_controller_contract",
        "hardware_commands_published": False,
        "ros_node_created": False,
        "splits": {},
    }
    all_failures: list[str] = []
    for split in ("train", "validation", "test"):
        split_report, failures = audit_split(
            split,
            args.dataset_root / split,
            args.raw_root,
            max_camera_skew_ns=120_000_000,
            max_command_age_ns=50_000_000,
        )
        report["splits"][split] = split_report
        all_failures.extend(f"{split}: {failure}" for failure in failures)
        print(
            f"{split}: episodes={split_report.get('episodes')}, "
            f"frames={split_report.get('frames')}, pass={split_report.get('pass')}",
            flush=True,
        )
    report["pass"] = not all_failures
    report["failures"] = all_failures
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"pass": report["pass"], "failures": all_failures}, indent=2))
    print("OFFLINE ONLY: no ROS node or hardware command publisher was created.")
    print(f"REPORT: {args.output.resolve()}")
    if all_failures:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
