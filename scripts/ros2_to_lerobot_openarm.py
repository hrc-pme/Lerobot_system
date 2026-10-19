#!/usr/bin/env python3
"""Convert native OpenArm ROS 2 bags into a LeRobot joint-space dataset.

The absolute-controller baseline uses measured ``/joint_states`` as
``observation.state`` and the four ros2_control command topics as ``action``.
All samples are reordered by joint name into a stable 16-D layout:

    left joint1..7, left gripper, right joint1..7, right gripper

Run validation on a machine with ROS 2 Humble:

    python3 scripts/ros2_to_lerobot_openarm.py \
      --config scripts/ros2_to_lerobot_openarm_config.yaml --validate-only

Run conversion inside the project's GPU container:

    python3 scripts/ros2_to_lerobot_openarm.py \
      --config scripts/ros2_to_lerobot_openarm_config.yaml --overwrite
"""

from __future__ import annotations

import argparse
import io
import json
import math
import shutil
import sqlite3
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray


LEFT_ARM_NAMES = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_ARM_NAMES = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_GRIPPER_NAME = "openarm_left_finger_joint1"
RIGHT_GRIPPER_NAME = "openarm_right_finger_joint1"
JOINT_ORDER = (
    LEFT_ARM_NAMES
    + [LEFT_GRIPPER_NAME]
    + RIGHT_ARM_NAMES
    + [RIGHT_GRIPPER_NAME]
)

STATE_TOPIC = "/joint_states"
LEFT_ARM_COMMAND_TOPIC = "/left_forward_position_controller/commands"
RIGHT_ARM_COMMAND_TOPIC = "/right_forward_position_controller/commands"
LEFT_GRIPPER_COMMAND_TOPIC = "/left_gripper_controller/commands"
RIGHT_GRIPPER_COMMAND_TOPIC = "/right_gripper_controller/commands"

COMMAND_TOPICS = [
    LEFT_ARM_COMMAND_TOPIC,
    LEFT_GRIPPER_COMMAND_TOPIC,
    RIGHT_ARM_COMMAND_TOPIC,
    RIGHT_GRIPPER_COMMAND_TOPIC,
]

EXPECTED_TYPES = {
    STATE_TOPIC: "sensor_msgs/msg/JointState",
    LEFT_ARM_COMMAND_TOPIC: "std_msgs/msg/Float64MultiArray",
    RIGHT_ARM_COMMAND_TOPIC: "std_msgs/msg/Float64MultiArray",
    LEFT_GRIPPER_COMMAND_TOPIC: "std_msgs/msg/Float64MultiArray",
    RIGHT_GRIPPER_COMMAND_TOPIC: "std_msgs/msg/Float64MultiArray",
}


@dataclass(frozen=True)
class GripperEvent:
    """One stable hysteresis transition in a gripper command stream."""

    timestamp_ns: int
    direction: str


@dataclass(frozen=True)
class PhaseTrimWindow:
    """Selected task interval, expressed in source rosbag time."""

    start_ns: int
    stop_ns: int
    right_events: tuple[GripperEvent, ...]
    left_events: tuple[GripperEvent, ...]


TASK_PHASE_NAMES = (
    "right_approach",
    "right_grasp_transport",
    "right_release_transition",
    "bimanual_grasp_sync",
    "bimanual_wipe",
    "handoff",
    "left_transport_to_box",
    "left_place_in_box",
    "retreat",
)
EVENT_MEMORY_NAMES = (
    "event_right_pick_close",
    "event_right_pick_release",
    "event_left_wipe_close",
    "event_right_wipe_close",
    "event_left_wipe_release",
    "event_left_box_close",
    "event_right_wipe_release",
    "event_left_box_release",
)


def resolve_path(value: str) -> Path:
    return Path(value).expanduser().resolve()


def detect_gripper_hysteresis_events(
    samples: list[tuple[int, float]],
    close_threshold_m: float,
    open_threshold_m: float,
    minimum_stable_ns: int,
) -> list[GripperEvent]:
    """Return stable open/close transitions from scalar gripper commands.

    Values between the two thresholds retain the previous candidate and avoid
    chatter.  A state must remain beyond its threshold for
    ``minimum_stable_ns`` before it is accepted; the event timestamp is the
    beginning of that stable run, not the delayed confirmation time.
    """

    if close_threshold_m >= open_threshold_m:
        raise ValueError("close_threshold_m must be below open_threshold_m")
    if minimum_stable_ns < 0:
        raise ValueError("minimum_stable_ns must be non-negative")
    if not samples:
        raise ValueError("Gripper command stream is empty")

    previous_timestamp: int | None = None
    stable_state: str | None = None
    candidate_state: str | None = None
    candidate_start: int | None = None
    events: list[GripperEvent] = []
    for timestamp, value in samples:
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise ValueError("Gripper command timestamps are not strictly increasing")
        if not math.isfinite(value):
            raise ValueError("Gripper command stream contains NaN or Inf")
        previous_timestamp = timestamp

        if value <= close_threshold_m:
            observed = "close"
        elif value >= open_threshold_m:
            observed = "open"
        else:
            # The hysteresis band neither starts nor interrupts a candidate.
            continue

        if observed != candidate_state:
            candidate_state = observed
            candidate_start = timestamp
        if candidate_start is None or timestamp - candidate_start < minimum_stable_ns:
            continue
        if stable_state == candidate_state:
            continue

        stable_state = candidate_state
        if len(events) == 0 and stable_state == "open":
            # Initial open establishes the baseline and is not a task event.
            continue
        events.append(GripperEvent(candidate_start, stable_state))
    return events


def gripper_sequence_candidates(
    events: list[GripperEvent],
) -> list[tuple[GripperEvent, ...]]:
    """Find complete close/open/close/open subsequences."""

    pattern = ("close", "open", "close", "open")
    return [
        tuple(events[index : index + len(pattern)])
        for index in range(len(events) - len(pattern) + 1)
        if tuple(
            event.direction for event in events[index : index + len(pattern)]
        )
        == pattern
    ]


def select_complete_task_sequences(
    right_events: list[GripperEvent],
    left_events: list[GripperEvent],
    bimanual_close_reorder_tolerance_ns: int = 0,
) -> tuple[tuple[GripperEvent, ...], tuple[GripperEvent, ...]]:
    """Select the first complete, chronologically valid bimanual task.

    The demonstrations encode:

      R close, R open, L close, R close, L open, L close, R open, L open.

    The first left close and second right close form a bimanual synchronization
    boundary.  Some valid demonstrations issue the right close slightly first;
    that single pair may be reordered within the explicitly configured
    tolerance.  No other event-order relaxation is permitted.

    Requiring the entire sequence prevents a startup gripper transient from
    being mistaken for the first grasp and makes incomplete recordings fail
    loudly instead of silently producing a partial training episode.
    """

    if bimanual_close_reorder_tolerance_ns < 0:
        raise ValueError("bimanual close reorder tolerance must be non-negative")

    right_candidates = gripper_sequence_candidates(right_events)
    left_candidates = gripper_sequence_candidates(left_events)
    valid: list[
        tuple[tuple[GripperEvent, ...], tuple[GripperEvent, ...]]
    ] = []
    for right in right_candidates:
        for left in left_candidates:
            left_sync_close = left[0].timestamp_ns
            right_sync_close = right[2].timestamp_ns
            reordered_sync = right_sync_close < left_sync_close
            if (
                reordered_sync
                and left_sync_close - right_sync_close
                > bimanual_close_reorder_tolerance_ns
            ):
                continue
            ordered = (
                right[0].timestamp_ns,
                right[1].timestamp_ns,
                min(left_sync_close, right_sync_close),
                max(left_sync_close, right_sync_close),
                left[1].timestamp_ns,
                left[2].timestamp_ns,
                right[3].timestamp_ns,
                left[3].timestamp_ns,
            )
            if all(after > before for before, after in zip(ordered, ordered[1:])):
                valid.append((right, left))
    if not valid:
        raise ValueError(
            "No complete task sequence found; required right C/O/C/O and left "
            "C/O/C/O with the demonstrated cross-arm event order (apart from "
            "the configured bimanual-close reorder tolerance)"
        )
    return min(
        valid,
        key=lambda pair: (
            pair[0][0].timestamp_ns,
            pair[1][0].timestamp_ns,
        ),
    )


def phase_consistent_trim_window(
    right_events: list[GripperEvent],
    left_events: list[GripperEvent],
    overlap_start_ns: int,
    overlap_stop_ns: int,
    pre_roll_ns: int,
    post_roll_ns: int,
    bimanual_close_reorder_tolerance_ns: int = 0,
) -> PhaseTrimWindow:
    """Return a full-task window without clipping requested context."""

    if pre_roll_ns < 0 or post_roll_ns < 0:
        raise ValueError("Phase trim pre/post roll must be non-negative")
    if overlap_stop_ns <= overlap_start_ns:
        raise ValueError("Required topics have no common time range")
    right, left = select_complete_task_sequences(
        right_events,
        left_events,
        bimanual_close_reorder_tolerance_ns,
    )
    start_ns = right[0].timestamp_ns - pre_roll_ns
    stop_ns = left[3].timestamp_ns + post_roll_ns
    if start_ns < overlap_start_ns:
        raise ValueError(
            "Complete task lacks the requested pre-roll inside the common topic range"
        )
    if stop_ns > overlap_stop_ns:
        raise ValueError(
            "Complete task lacks the requested post-roll inside the common topic range"
        )
    if stop_ns <= start_ns:
        raise ValueError("Computed phase trim window is empty")
    return PhaseTrimWindow(start_ns, stop_ns, right, left)


def discover_bags(
    root: Path,
    selected: list[str] | None,
    excluded: list[str] | None = None,
) -> list[Path]:
    if (root / "metadata.yaml").is_file():
        bags = [root]
    else:
        bags = sorted(
            p for p in root.iterdir() if p.is_dir() and (p / "metadata.yaml").is_file()
        )
    if selected:
        wanted = set(selected)
        bags = [p for p in bags if p.name in wanted]
        missing = sorted(wanted - {p.name for p in bags})
        if missing:
            raise ValueError(f"Selected episode(s) not found: {', '.join(missing)}")
    if excluded:
        bags = [p for p in bags if p.name not in set(excluded)]
    if not bags:
        raise ValueError(f"No ROS 2 bags containing metadata.yaml found under {root}")
    return bags


def db_path_for_bag(bag: Path) -> Path:
    databases = sorted(bag.glob("*.db3"))
    if len(databases) != 1:
        raise ValueError(f"Expected exactly one .db3 in {bag}, found {len(databases)}")
    return databases[0]


def metadata_from_db(db_path: Path) -> dict[str, dict[str, Any]]:
    connection = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        rows = connection.execute(
            """
            SELECT topics.name, topics.type, COUNT(messages.id),
                   MIN(messages.timestamp), MAX(messages.timestamp)
            FROM topics
            LEFT JOIN messages ON messages.topic_id = topics.id
            GROUP BY topics.id, topics.name, topics.type
            """
        ).fetchall()
    finally:
        connection.close()
    return {
        name: {"type": msg_type, "count": count, "first": first, "last": last}
        for name, msg_type, count, first, last in rows
    }


def read_first_required_messages(bag: Path, camera_topics: list[str]) -> dict[str, Any]:
    message_types = {
        STATE_TOPIC: JointState,
        LEFT_ARM_COMMAND_TOPIC: Float64MultiArray,
        RIGHT_ARM_COMMAND_TOPIC: Float64MultiArray,
        LEFT_GRIPPER_COMMAND_TOPIC: Float64MultiArray,
        RIGHT_GRIPPER_COMMAND_TOPIC: Float64MultiArray,
        **{topic: CompressedImage for topic in camera_topics},
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(message_types)))
    found: dict[str, Any] = {}
    while reader.has_next() and len(found) < len(message_types):
        topic, payload, _ = reader.read_next()
        if topic not in found:
            found[topic] = deserialize_message(payload, message_types[topic])
    return found


def read_gripper_command_samples(
    bag: Path,
) -> dict[str, list[tuple[int, float]]]:
    """Read both scalar controller-command streams for phase detection."""

    topics = [LEFT_GRIPPER_COMMAND_TOPIC, RIGHT_GRIPPER_COMMAND_TOPIC]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    samples: dict[str, list[tuple[int, float]]] = {topic: [] for topic in topics}
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        message = deserialize_message(payload, Float64MultiArray)
        value = command_vector(message, 1, topic)
        samples[topic].append((timestamp, float(value[0])))
    for topic in topics:
        if not samples[topic]:
            raise ValueError(f"{bag.name}: no gripper commands on {topic}")
    return samples


def phase_trim_window_for_bag(
    bag: Path,
    metadata: dict[str, dict[str, Any]],
    required_topics: list[str],
    phase_trim: dict[str, Any],
) -> PhaseTrimWindow:
    """Detect and validate the configured full-task trim interval."""

    close_threshold = float(phase_trim.get("close_threshold_m", 0.030))
    open_threshold = float(phase_trim.get("open_threshold_m", 0.035))
    minimum_stable_ns = round(
        float(phase_trim.get("minimum_stable_sec", 0.25)) * 1e9
    )
    pre_roll_ns = round(float(phase_trim.get("pre_roll_sec", 1.0)) * 1e9)
    post_roll_ns = round(float(phase_trim.get("post_roll_sec", 1.0)) * 1e9)
    bimanual_close_reorder_tolerance_ns = round(
        float(phase_trim.get("bimanual_close_reorder_tolerance_sec", 0.0)) * 1e9
    )
    if minimum_stable_ns < 0:
        raise ValueError("phase_trim.minimum_stable_sec must be non-negative")

    overlap_start = max(int(metadata[topic]["first"]) for topic in required_topics)
    overlap_stop = min(int(metadata[topic]["last"]) for topic in required_topics)
    streams = read_gripper_command_samples(bag)
    left_events = detect_gripper_hysteresis_events(
        streams[LEFT_GRIPPER_COMMAND_TOPIC],
        close_threshold,
        open_threshold,
        minimum_stable_ns,
    )
    right_events = detect_gripper_hysteresis_events(
        streams[RIGHT_GRIPPER_COMMAND_TOPIC],
        close_threshold,
        open_threshold,
        minimum_stable_ns,
    )
    try:
        return phase_consistent_trim_window(
            right_events,
            left_events,
            overlap_start,
            overlap_stop,
            pre_roll_ns,
            post_roll_ns,
            bimanual_close_reorder_tolerance_ns,
        )
    except ValueError as exc:
        raise ValueError(f"{bag.name}: phase-consistent trim failed: {exc}") from exc


def task_phase_windows_for_bag(
    bag: Path,
    metadata: dict[str, dict[str, Any]],
    required_topics: list[str],
    phase_trim: dict[str, Any],
    requested_phases: tuple[str, ...],
    context_before_ns: int,
    context_after_ns: int,
) -> dict[str, PhaseTrimWindow]:
    """Return event-aligned phase clips without requiring full pre-roll.

    This is intentionally separate from :func:`phase_trim_window_for_bag`.
    A supplemental recording may contain the complete demonstrated task but
    start less than ``phase_trim.pre_roll_sec`` before the first right grasp.
    Such a recording is unsafe as a full episode because it would alter the
    training-start distribution, but its post-grasp phases are still valid
    demonstrations.  The complete C/O/C/O sequence and cross-arm event order
    remain mandatory.
    """

    if context_before_ns < 0 or context_after_ns < 0:
        raise ValueError("Supplemental phase context must be non-negative")
    unknown = set(requested_phases) - set(TASK_PHASE_NAMES)
    if unknown:
        raise ValueError(f"Unknown supplemental phases: {sorted(unknown)}")

    close_threshold = float(phase_trim.get("close_threshold_m", 0.030))
    open_threshold = float(phase_trim.get("open_threshold_m", 0.035))
    minimum_stable_ns = round(
        float(phase_trim.get("minimum_stable_sec", 0.25)) * 1e9
    )
    bimanual_close_reorder_tolerance_ns = round(
        float(phase_trim.get("bimanual_close_reorder_tolerance_sec", 0.0)) * 1e9
    )
    overlap_start = max(int(metadata[topic]["first"]) for topic in required_topics)
    overlap_stop = min(int(metadata[topic]["last"]) for topic in required_topics)
    if overlap_stop <= overlap_start:
        raise ValueError(f"{bag.name}: required topics have no common time range")

    streams = read_gripper_command_samples(bag)
    left_events = detect_gripper_hysteresis_events(
        streams[LEFT_GRIPPER_COMMAND_TOPIC],
        close_threshold,
        open_threshold,
        minimum_stable_ns,
    )
    right_events = detect_gripper_hysteresis_events(
        streams[RIGHT_GRIPPER_COMMAND_TOPIC],
        close_threshold,
        open_threshold,
        minimum_stable_ns,
    )
    try:
        right, left = select_complete_task_sequences(
            right_events,
            left_events,
            bimanual_close_reorder_tolerance_ns,
        )
    except ValueError as exc:
        raise ValueError(
            f"{bag.name}: supplemental task sequence validation failed: {exc}"
        ) from exc

    sync_start_ns = min(left[0].timestamp_ns, right[2].timestamp_ns)
    sync_stop_ns = max(left[0].timestamp_ns, right[2].timestamp_ns)
    anchors = (
        overlap_start,
        right[0].timestamp_ns,
        right[1].timestamp_ns,
        sync_start_ns,
        sync_stop_ns,
        left[1].timestamp_ns,
        left[2].timestamp_ns,
        right[3].timestamp_ns,
        left[3].timestamp_ns,
        overlap_stop,
    )
    if any(after <= before for before, after in zip(anchors, anchors[1:])):
        raise ValueError(
            f"{bag.name}: supplemental task anchors are not strictly ordered"
        )

    windows: dict[str, PhaseTrimWindow] = {}
    for name, phase_start, phase_stop in zip(
        TASK_PHASE_NAMES, anchors[:-1], anchors[1:], strict=True
    ):
        if name not in requested_phases:
            continue
        start_ns = max(overlap_start, phase_start - context_before_ns)
        stop_ns = min(overlap_stop, phase_stop + context_after_ns)
        if stop_ns <= start_ns:
            raise ValueError(f"{bag.name}: empty supplemental clip for {name}")
        windows[name] = PhaseTrimWindow(start_ns, stop_ns, right, left)
    return windows


def state_vector(message: JointState) -> np.ndarray:
    if len(message.name) != len(message.position):
        raise ValueError(
            f"JointState name/position lengths differ: {len(message.name)} != {len(message.position)}"
        )
    positions = dict(zip(message.name, message.position, strict=True))
    missing = [name for name in JOINT_ORDER if name not in positions]
    if missing:
        raise ValueError(f"JointState is missing required joints: {missing}")
    result = np.asarray([positions[name] for name in JOINT_ORDER], dtype=np.float32)
    if not np.isfinite(result).all():
        raise ValueError("JointState contains NaN or Inf")
    return result


def command_vector(message: Float64MultiArray, expected_size: int, topic: str) -> np.ndarray:
    result = np.asarray(message.data, dtype=np.float32)
    if result.shape != (expected_size,):
        raise ValueError(f"{topic} expected {expected_size} value(s), got {result.shape}")
    if not np.isfinite(result).all():
        raise ValueError(f"{topic} contains NaN or Inf")
    return result


def validate_bag(
    bag: Path, camera_topics: list[str], max_skew_sec: float, fps: float
) -> dict[str, Any]:
    db_path = db_path_for_bag(bag)
    metadata = metadata_from_db(db_path)
    required_types = {
        **EXPECTED_TYPES,
        **{topic: "sensor_msgs/msg/CompressedImage" for topic in camera_topics},
    }
    errors: list[str] = []
    for topic, expected_type in required_types.items():
        details = metadata.get(topic)
        if details is None:
            errors.append(f"missing topic {topic}")
        elif details["type"] != expected_type:
            errors.append(f"{topic}: expected {expected_type}, got {details['type']}")
        elif not details["count"]:
            errors.append(f"{topic}: zero messages")
    if errors:
        raise ValueError("; ".join(errors))

    starts = [metadata[t]["first"] for t in required_types]
    ends = [metadata[t]["last"] for t in required_types]
    overlap_start = max(starts)
    overlap_end = min(ends)
    if overlap_end <= overlap_start:
        raise ValueError("Required topics have no common time range")

    messages = read_first_required_messages(bag, camera_topics)
    state_vector(messages[STATE_TOPIC])
    command_vector(messages[LEFT_ARM_COMMAND_TOPIC], 7, LEFT_ARM_COMMAND_TOPIC)
    command_vector(messages[RIGHT_ARM_COMMAND_TOPIC], 7, RIGHT_ARM_COMMAND_TOPIC)
    command_vector(messages[LEFT_GRIPPER_COMMAND_TOPIC], 1, LEFT_GRIPPER_COMMAND_TOPIC)
    command_vector(messages[RIGHT_GRIPPER_COMMAND_TOPIC], 1, RIGHT_GRIPPER_COMMAND_TOPIC)

    # Pillow avoids importing OpenCV during host-only validation.
    from PIL import Image

    image_sizes = {}
    for topic in camera_topics:
        image = Image.open(io.BytesIO(bytes(messages[topic].data)))
        image.verify()
        image_sizes[topic] = image.size

    estimated_frames = max(0, math.floor((overlap_end - overlap_start) / 1e9 * fps))
    return {
        "overlap_sec": (overlap_end - overlap_start) / 1e9,
        "estimated_frames": estimated_frames,
        "image_sizes": image_sizes,
        "max_skew_sec": max_skew_sec,
    }


class OpenArmConverter:
    def __init__(
        self,
        config: dict[str, Any],
        output: Path,
        overwrite: bool,
        append_event_memory_state: bool = False,
    ):
        self.config = config
        self.output = output
        configured_fps = float(config["dataset"]["fps"])
        if not configured_fps.is_integer():
            raise ValueError(
                "This LeRobot/PyAV environment requires an integer dataset FPS"
            )
        self.fps = int(configured_fps)
        self.period_ns = round(1e9 / self.fps)
        self.half_period_ns = self.period_ns // 2
        self.max_skew_ns = round(float(config["sync"]["max_skew_sec"]) * 1e9)
        self.max_numeric_gap_ns = round(
            float(config["sync"].get("max_numeric_gap_sec", 0.15)) * 1e9
        )
        self.max_command_age_ns = round(
            float(
                config["sync"].get(
                    "max_command_age_sec",
                    config["sync"].get("max_numeric_gap_sec", 0.15),
                )
            )
            * 1e9
        )
        self.controller_command_sampling = str(
            config["sync"].get("controller_command_sampling", "linear")
        )
        if self.controller_command_sampling not in {"linear", "previous"}:
            raise ValueError(
                "sync.controller_command_sampling must be linear or previous"
            )
        self.fail_on_skipped_frames = bool(
            config["sync"].get("fail_on_skipped_frames", True)
        )
        self.camera_configs = config["mapping"]["cameras"]
        self.camera_topics = list(self.camera_configs)
        self.action_source = config["mapping"].get("action_source", "controller_command")
        if self.action_source not in {
            "controller_command",
            "next_measured_state",
            "measured_state_delta",
        }:
            raise ValueError(
                "mapping.action_source must be controller_command, "
                "next_measured_state, or measured_state_delta"
            )
        self.action_lead_steps = int(config["mapping"].get("action_lead_steps", 1))
        if self.action_lead_steps < 1:
            raise ValueError("mapping.action_lead_steps must be at least 1")
        self.required_topics = [STATE_TOPIC, *COMMAND_TOPICS, *self.camera_topics]
        self.numeric_topics = [STATE_TOPIC, *COMMAND_TOPICS]
        validation = config.get("validation", {})
        self.raw_state_delta_limit = float(
            validation.get("max_raw_state_arm_delta_rad", 0.20)
        )
        self.raw_command_delta_limit = float(
            validation.get("max_raw_arm_command_delta_rad", 0.20)
        )
        self.raw_gripper_delta_limit = float(
            validation.get("max_raw_gripper_delta_m", 0.02)
        )
        self.previous_numeric: dict[str, tuple[int, np.ndarray]] = {}
        self.append_event_memory_state = append_event_memory_state
        self.event_memory_timestamps: tuple[int, ...] | None = None

        self.write_provenance = bool(
            config.get("audit", {}).get("write_frame_provenance", False)
        )
        self.provenance_records: list[dict[str, Any]] = []
        self.output_episode_index = 0
        self.current_bag_name: str | None = None

        if output.exists():
            if not overwrite:
                raise FileExistsError(
                    f"Output already exists: {output}. Pass --overwrite to replace it."
                )
            shutil.rmtree(output)
        output.parent.mkdir(parents=True, exist_ok=True)

        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise RuntimeError(
                "LeRobot is not importable. Run conversion inside docker/GPU/run.sh."
            ) from exc

        features: dict[str, Any] = {}
        for camera in self.camera_configs.values():
            features[camera["key"]] = {
                "dtype": "video",
                "shape": (3, int(camera["height"]), int(camera["width"])),
                "names": ["channel", "height", "width"],
            }
        state_names = list(JOINT_ORDER)
        if self.append_event_memory_state:
            state_names.extend(EVENT_MEMORY_NAMES)
        features[config["mapping"]["state_key"]] = {
            "dtype": "float32",
            "shape": (len(state_names),),
            "names": state_names,
        }
        features[config["mapping"]["action_key"]] = {
            "dtype": "float32",
            "shape": (16,),
            "names": JOINT_ORDER,
        }
        writer = config.get("image_writer", {})
        self.dataset = LeRobotDataset.create(
            repo_id=config["dataset"]["repo_id"],
            fps=self.fps,
            robot_type=config["robot"]["type"],
            features=features,
            root=output,
            image_writer_processes=int(writer.get("processes", 4)),
            image_writer_threads=int(writer.get("threads", 4)),
        )

    @staticmethod
    def nearest(buffer: deque[tuple[int, Any]], timestamp: int) -> tuple[int, Any]:
        return min(buffer, key=lambda item: abs(item[0] - timestamp))

    @staticmethod
    def interpolate(
        buffer: deque[tuple[int, Any]], timestamp: int
    ) -> tuple[np.ndarray, int]:
        """Linearly interpolate a numeric topic at an exact dataset timestamp."""
        before: tuple[int, np.ndarray] | None = None
        after: tuple[int, np.ndarray] | None = None
        for sample_time, value in buffer:
            if sample_time <= timestamp:
                before = (sample_time, value)
            if sample_time >= timestamp:
                after = (sample_time, value)
                break
        if before is None or after is None:
            raise ValueError("Numeric topic does not bracket the requested timestamp")
        before_time, before_value = before
        after_time, after_value = after
        gap = after_time - before_time
        if gap == 0:
            return before_value.copy(), 0
        alpha = (timestamp - before_time) / gap
        value = before_value + (after_value - before_value) * alpha
        return value.astype(np.float32, copy=False), gap

    @staticmethod
    def interpolation_bracket(
        buffer: deque[tuple[int, Any]], timestamp: int
    ) -> tuple[np.ndarray, int, int]:
        """Interpolate and return the two real source timestamps used."""

        before: tuple[int, np.ndarray] | None = None
        after: tuple[int, np.ndarray] | None = None
        for sample_time, value in buffer:
            if sample_time <= timestamp:
                before = (sample_time, value)
            if sample_time >= timestamp:
                after = (sample_time, value)
                break
        if before is None or after is None:
            raise ValueError("Numeric topic does not bracket the requested timestamp")
        before_time, before_value = before
        after_time, after_value = after
        gap = after_time - before_time
        if gap == 0:
            return before_value.copy(), before_time, after_time
        alpha = (timestamp - before_time) / gap
        value = before_value + (after_value - before_value) * alpha
        return value.astype(np.float32, copy=False), before_time, after_time

    @staticmethod
    def previous(
        buffer: deque[tuple[int, Any]], timestamp: int
    ) -> tuple[np.ndarray, int]:
        """Return the latest real sample at or before ``timestamp``.

        This is a zero-order hold. It never interpolates command values and
        never reads a future controller target.
        """

        selected: tuple[int, np.ndarray] | None = None
        for sample_time, value in buffer:
            if sample_time > timestamp:
                break
            selected = (sample_time, value)
        if selected is None:
            raise ValueError("Numeric topic has no sample at/before timestamp")
        source_time, value = selected
        return value.copy(), source_time

    def validate_raw_numeric(self, topic: str, timestamp: int, value: np.ndarray) -> None:
        previous = self.previous_numeric.get(topic)
        self.previous_numeric[topic] = (timestamp, value.copy())
        if previous is None:
            return
        previous_time, previous_value = previous
        if timestamp <= previous_time:
            raise RuntimeError(f"{topic} timestamps are not strictly increasing")
        delta = np.abs(value - previous_value)
        if topic == STATE_TOPIC:
            arm_delta = float(delta[list(range(7)) + list(range(8, 15))].max())
            limit = self.raw_state_delta_limit
        elif "forward_position" in topic:
            arm_delta = float(delta.max())
            limit = self.raw_command_delta_limit
        else:
            arm_delta = float(delta.max())
            limit = self.raw_gripper_delta_limit
        if arm_delta > limit:
            raise RuntimeError(
                f"{topic} raw delta {arm_delta:.4f} exceeds configured limit "
                f"{limit:.4f} at {(timestamp - previous_time) / 1e9:.4f}s interval"
            )

    def make_frame(
        self, buffers: dict[str, deque[tuple[int, Any]]], timestamp: int
    ) -> tuple[dict[str, Any], dict[str, Any]] | None:
        camera_samples = {
            topic: self.nearest(buffers[topic], timestamp)
            for topic in self.camera_topics
        }
        worst_camera_skew = max(
            abs(sample_time - timestamp) for sample_time, _ in camera_samples.values()
        )
        if worst_camera_skew > self.max_skew_ns:
            return None

        try:
            state, state_before_ns, state_after_ns = self.interpolation_bracket(
                buffers[STATE_TOPIC], timestamp
            )
        except ValueError:
            return None
        state_source_gap = state_after_ns - state_before_ns
        if state_source_gap > self.max_numeric_gap_ns:
            return None

        numeric_samples: dict[str, np.ndarray] = {STATE_TOPIC: state}
        command_sources: dict[str, dict[str, int | str]] = {}
        for topic in COMMAND_TOPICS:
            try:
                if self.controller_command_sampling == "previous":
                    value, source_time = self.previous(buffers[topic], timestamp)
                    age_ns = timestamp - source_time
                    if age_ns < 0 or age_ns > self.max_command_age_ns:
                        return None
                    command_sources[topic] = {
                        "sampling": "previous",
                        "source_timestamp_ns": source_time,
                        "age_ns": age_ns,
                    }
                else:
                    value, before_ns, after_ns = self.interpolation_bracket(
                        buffers[topic], timestamp
                    )
                    if after_ns - before_ns > self.max_numeric_gap_ns:
                        return None
                    command_sources[topic] = {
                        "sampling": "linear",
                        "before_timestamp_ns": before_ns,
                        "after_timestamp_ns": after_ns,
                    }
            except ValueError:
                return None
            numeric_samples[topic] = value

        if self.action_source in {"next_measured_state", "measured_state_delta"}:
            action_timestamp = timestamp + self.action_lead_steps * self.period_ns
            try:
                action_sample, action_source_gap = self.interpolate(
                    buffers[STATE_TOPIC], action_timestamp
                )
            except ValueError:
                return None
            if action_source_gap > self.max_numeric_gap_ns:
                return None
            if self.action_source == "measured_state_delta":
                action_sample = (
                    action_sample - numeric_samples[STATE_TOPIC]
                ).astype(np.float32, copy=False)
        else:
            action_sample = np.concatenate(
                [numeric_samples[topic] for topic in COMMAND_TOPICS]
            ).astype(np.float32, copy=False)

        import cv2

        frame: dict[str, Any] = {"task": self.config["dataset"]["task"]}
        for topic, camera in self.camera_configs.items():
            payload = camera_samples[topic][1]
            image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"Failed to decode image from {topic}")
            target_size = (int(camera["width"]), int(camera["height"]))
            if (image.shape[1], image.shape[0]) != target_size:
                image = cv2.resize(image, target_size, interpolation=cv2.INTER_AREA)
            frame[camera["key"]] = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        state = numeric_samples[STATE_TOPIC]
        if self.append_event_memory_state:
            if self.event_memory_timestamps is None:
                raise RuntimeError("Event-memory conversion requires a phase trim window")
            memory = np.asarray(
                [timestamp >= event_time for event_time in self.event_memory_timestamps],
                dtype=np.float32,
            )
            state = np.concatenate((state, memory)).astype(np.float32, copy=False)
        frame[self.config["mapping"]["state_key"]] = state
        frame[self.config["mapping"]["action_key"]] = action_sample
        provenance = {
            "source_bag": self.current_bag_name,
            "dataset_timestamp_ns": timestamp,
            "camera_sources": {
                topic: {
                    "source_timestamp_ns": sample_time,
                    "skew_ns": sample_time - timestamp,
                }
                for topic, (sample_time, _) in camera_samples.items()
            },
            "state_source": {
                "sampling": "linear",
                "before_timestamp_ns": state_before_ns,
                "after_timestamp_ns": state_after_ns,
                "bracket_gap_ns": state_source_gap,
            },
            "command_sources": command_sources,
        }
        return frame, provenance

    @staticmethod
    def prune_buffers(
        buffers: dict[str, deque[tuple[int, Any]]], timestamp: int
    ) -> None:
        # Preserve the last sample at/before timestamp plus all future samples.
        for buffer in buffers.values():
            while len(buffer) > 2 and buffer[1][0] <= timestamp:
                buffer.popleft()

    def convert_bag(
        self,
        bag: Path,
        trim_window: PhaseTrimWindow | None = None,
    ) -> tuple[int, int]:
        buffers = {topic: deque() for topic in self.required_topics}
        first_seen: dict[str, int] = {}
        next_sample_ns: int | None = None
        saved = 0
        skipped = 0
        self.previous_numeric.clear()
        self.current_bag_name = bag.name
        if self.append_event_memory_state:
            if trim_window is None:
                raise ValueError("Event-memory conversion requires phase_trim.enabled")
            right = trim_window.right_events
            left = trim_window.left_events
            if len(right) != 4 or len(left) != 4:
                raise ValueError("Event-memory conversion requires complete C/O/C/O sequences")
            self.event_memory_timestamps = (
                right[0].timestamp_ns,
                right[1].timestamp_ns,
                left[0].timestamp_ns,
                right[2].timestamp_ns,
                left[1].timestamp_ns,
                left[2].timestamp_ns,
                right[3].timestamp_ns,
                left[3].timestamp_ns,
            )
        else:
            self.event_memory_timestamps = None

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        reader.set_filter(rosbag2_py.StorageFilter(topics=self.required_topics))

        while reader.has_next():
            topic, payload, timestamp = reader.read_next()
            if topic == STATE_TOPIC:
                value = state_vector(deserialize_message(payload, JointState))
            elif topic in COMMAND_TOPICS:
                expected_size = 7 if "forward_position" in topic else 1
                value = command_vector(
                    deserialize_message(payload, Float64MultiArray), expected_size, topic
                )
            else:
                message = deserialize_message(payload, CompressedImage)
                value = bytes(message.data)
            if topic in self.numeric_topics:
                self.validate_raw_numeric(topic, timestamp, value)
            buffers[topic].append((timestamp, value))
            first_seen.setdefault(topic, timestamp)

            if next_sample_ns is None and len(first_seen) == len(self.required_topics):
                common_start = max(first_seen.values())
                if trim_window is not None:
                    common_start = max(common_start, trim_window.start_ns)
                next_sample_ns = (
                    (common_start + self.period_ns - 1) // self.period_ns
                ) * self.period_ns

            while (
                next_sample_ns is not None
                and all(buffers[t] for t in self.required_topics)
                and min(buffers[t][-1][0] for t in self.required_topics)
                >= next_sample_ns
                + self.half_period_ns
                + (
                    self.action_lead_steps * self.period_ns
                    if self.action_source
                    in {"next_measured_state", "measured_state_delta"}
                    else 0
                )
            ):
                if (
                    trim_window is not None
                    and next_sample_ns >= trim_window.stop_ns
                ):
                    break
                result = self.make_frame(buffers, next_sample_ns)
                if result is None:
                    if self.fail_on_skipped_frames:
                        raise RuntimeError(
                            f"Unable to synchronize {bag.name} at "
                            f"timestamp {next_sample_ns}; refusing to compress time"
                        )
                    skipped += 1
                else:
                    frame, provenance = result
                    self.dataset.add_frame(frame)
                    if self.write_provenance:
                        provenance["output_episode_index"] = self.output_episode_index
                        provenance["output_frame_index"] = saved
                        provenance["action"] = frame[
                            self.config["mapping"]["action_key"]
                        ].astype(float).tolist()
                        self.provenance_records.append(provenance)
                    saved += 1
                self.prune_buffers(buffers, next_sample_ns)
                next_sample_ns += self.period_ns

            if (
                trim_window is not None
                and next_sample_ns is not None
                and next_sample_ns >= trim_window.stop_ns
            ):
                break

        if saved == 0:
            raise RuntimeError(f"No synchronized frames produced for {bag.name}")
        self.dataset.save_episode()
        self.output_episode_index += 1
        return saved, skipped

    def finalize(self) -> None:
        self.dataset.finalize()
        if self.write_provenance:
            provenance_path = self.output / "absolute_command_provenance.jsonl"
            with provenance_path.open("w", encoding="utf-8") as stream:
                for record in self.provenance_records:
                    stream.write(json.dumps(record, sort_keys=True) + "\n")
            contract = {
                "schema_version": 1,
                "observation_state": "linearly interpolated measured /joint_states",
                "action": "zero-order-held real ros2_control absolute position command",
                "controller_command_sampling": self.controller_command_sampling,
                "future_controller_commands_used": False,
                "joint_order": JOINT_ORDER,
                "state_units": ["rad"] * 7 + ["m"] + ["rad"] * 7 + ["m"],
                "action_units": ["rad"] * 7 + ["m"] + ["rad"] * 7 + ["m"],
                "frame_provenance": provenance_path.name,
                "frames": len(self.provenance_records),
            }
            (self.output / "absolute_command_contract.json").write_text(
                json.dumps(contract, indent=2) + "\n", encoding="utf-8"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="scripts/ros2_to_lerobot_openarm_config.yaml",
        help="YAML configuration path",
    )
    parser.add_argument(
        "--episodes",
        help="Comma-separated episode names, e.g. 0001,0002. Default: all bags",
    )
    parser.add_argument(
        "--exclude-episodes",
        help="Comma-separated episode names to omit (in addition to config rosbag.exclude)",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Validate bags without creating a LeRobot dataset",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete and replace an existing output dataset",
    )
    parser.add_argument(
        "--append-event-memory-state",
        action="store_true",
        help=(
            "Append eight causal, monotonic gripper-event latch values to the "
            "OpenArm16 measured state. Intended first for oracle offline diagnosis."
        ),
    )
    parser.add_argument(
        "--output",
        help="Override output.path from the YAML file (useful for a one-episode test)",
    )
    parser.add_argument(
        "--repo-id",
        help=(
            "Override dataset.repo_id from the YAML file. Use this together with "
            "--episodes/--output when materializing leakage-free train, validation, "
            "and test datasets."
        ),
    )
    parser.add_argument(
        "--phase-balance-manifest",
        type=Path,
        help=(
            "After each selected full episode, append train-only phase clips "
            "balanced from this split manifest. This preserves the full task "
            "while increasing exposure to short critical phases."
        ),
    )
    parser.add_argument(
        "--phase-only-names",
        default="",
        help=(
            "Materialize only these comma-separated event-aligned phases from "
            "every --episodes recording, instead of full episodes. Intended "
            "for controlled overfit diagnostics, not full-task training."
        ),
    )
    parser.add_argument(
        "--phase-only-context-before-sec",
        type=float,
        default=0.0,
        help="Context retained before each --phase-only-names clip.",
    )
    parser.add_argument(
        "--phase-only-context-after-sec",
        type=float,
        default=0.0,
        help="Context retained after each --phase-only-names clip.",
    )
    parser.add_argument(
        "--phase-balance-names",
        default=(
            "right_approach,right_grasp_transport,bimanual_grasp_sync,"
            "bimanual_wipe,left_transport_to_box,left_place_in_box"
        ),
        help="Comma-separated phases to balance when --phase-balance-manifest is set.",
    )
    parser.add_argument(
        "--phase-context-before-sec",
        type=float,
        default=0.50,
        help="Context retained before every balanced phase clip (default: 0.50).",
    )
    parser.add_argument(
        "--phase-context-after-sec",
        type=float,
        default=1.75,
        help=(
            "Context retained after every balanced phase clip. The default is "
            "longer than one 50-step ACT chunk at 30 Hz."
        ),
    )
    parser.add_argument(
        "--supplemental-phase-episodes",
        help=(
            "Comma-separated recordings used only as event-aligned phase clips. "
            "They are never converted as full episodes, so recordings with "
            "insufficient pre-grasp context cannot change the training start."
        ),
    )
    parser.add_argument(
        "--supplemental-phase-names",
        default=(
            "right_grasp_transport,right_release_transition,bimanual_grasp_sync,"
            "bimanual_wipe,handoff,left_transport_to_box,left_place_in_box"
        ),
        help=(
            "Comma-separated task phases materialized from each supplemental "
            "recording. right_approach and retreat are excluded by default."
        ),
    )
    parser.add_argument(
        "--supplemental-phase-context-before-sec",
        type=float,
        default=0.50,
        help="Context before each supplemental phase clip (default: 0.50).",
    )
    parser.add_argument(
        "--supplemental-phase-context-after-sec",
        type=float,
        default=1.00,
        help="Context after each supplemental phase clip (default: 1.00).",
    )
    args = parser.parse_args()

    if args.phase_context_before_sec < 0 or args.phase_context_after_sec < 0:
        raise ValueError("Phase context durations must be non-negative")
    if (
        args.phase_only_context_before_sec < 0
        or args.phase_only_context_after_sec < 0
    ):
        raise ValueError("Phase-only context durations must be non-negative")
    if (
        args.supplemental_phase_context_before_sec < 0
        or args.supplemental_phase_context_after_sec < 0
    ):
        raise ValueError("Supplemental phase context durations must be non-negative")
    if args.phase_balance_manifest and args.validate_only:
        raise ValueError(
            "--phase-balance-manifest materializes extra episodes and cannot be "
            "combined with --validate-only"
        )
    if args.supplemental_phase_episodes and args.validate_only:
        raise ValueError(
            "--supplemental-phase-episodes materializes extra episodes and cannot "
            "be combined with --validate-only"
        )
    phase_only_names = tuple(
        item.strip() for item in args.phase_only_names.split(",") if item.strip()
    )
    if phase_only_names and args.phase_balance_manifest:
        raise ValueError(
            "--phase-only-names cannot be combined with --phase-balance-manifest"
        )
    if phase_only_names and args.supplemental_phase_episodes:
        raise ValueError(
            "--phase-only-names cannot be combined with supplemental phases"
        )

    with open(args.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if args.repo_id:
        config["dataset"]["repo_id"] = args.repo_id
    bag_root = resolve_path(config["rosbag"]["path"])
    output = resolve_path(args.output or config["output"]["path"])
    configured_selected = [
        str(item) for item in config.get("rosbag", {}).get("include", [])
    ]
    selected = (
        [item.strip() for item in args.episodes.split(",")]
        if args.episodes
        else configured_selected or None
    )
    excluded = [str(item) for item in config.get("rosbag", {}).get("exclude", [])]
    if args.exclude_episodes:
        excluded.extend(item.strip() for item in args.exclude_episodes.split(","))
    bags = discover_bags(bag_root, selected, excluded)
    camera_topics = list(config["mapping"]["cameras"])
    max_skew_sec = float(config["sync"]["max_skew_sec"])
    phase_trim = config.get("phase_trim", {})
    phase_trim_enabled = bool(phase_trim.get("enabled", False))
    if phase_only_names and not phase_trim_enabled:
        raise ValueError("--phase-only-names requires phase_trim.enabled")
    required_topics = [STATE_TOPIC, *COMMAND_TOPICS, *camera_topics]
    supplemental_names = (
        [
            item.strip()
            for item in args.supplemental_phase_episodes.split(",")
            if item.strip()
        ]
        if args.supplemental_phase_episodes
        else []
    )
    supplemental_phases = tuple(
        item.strip()
        for item in args.supplemental_phase_names.split(",")
        if item.strip()
    )
    supplemental_bags: list[Path] = []
    if supplemental_names:
        if not phase_trim_enabled:
            raise ValueError(
                "Supplemental phase episodes require phase_trim.enabled in config"
            )
        if len(supplemental_names) != len(set(supplemental_names)):
            raise ValueError("Duplicate --supplemental-phase-episodes are not allowed")
        if not supplemental_phases:
            raise ValueError("--supplemental-phase-names must not be empty")
        supplemental_bags = discover_bags(bag_root, supplemental_names)
        overlap = {bag.name for bag in bags} & {bag.name for bag in supplemental_bags}
        if overlap:
            raise ValueError(
                "Episodes cannot be both full and supplemental: "
                f"{sorted(overlap)}"
            )

    print(f"Input:  {bag_root}")
    print(f"Output: {output}")
    print(f"Episodes: {len(bags)}")
    if excluded:
        print(f"Excluded: {', '.join(sorted(set(excluded)))}")
    if phase_trim_enabled:
        print(
            "Phase trim: full R C/O/C/O + L C/O/C/O, "
            f"pre={float(phase_trim.get('pre_roll_sec', 1.0)):.3f}s, "
            f"post={float(phase_trim.get('post_roll_sec', 1.0)):.3f}s"
        )
    total_overlap = 0.0
    trim_windows: dict[Path, PhaseTrimWindow] = {}
    phase_only_windows: dict[Path, dict[str, PhaseTrimWindow]] = {}
    for bag in bags:
        result = validate_bag(
            bag, camera_topics, max_skew_sec, float(config["dataset"]["fps"])
        )
        duration = result["overlap_sec"]
        trim_summary = ""
        if phase_trim_enabled:
            metadata = metadata_from_db(db_path_for_bag(bag))
            trim_window = phase_trim_window_for_bag(
                bag,
                metadata,
                required_topics,
                phase_trim,
            )
            trim_windows[bag] = trim_window
            duration = (trim_window.stop_ns - trim_window.start_ns) / 1e9
            trim_summary = (
                f", phase_trim={duration:.2f}s"
                f" (R1 close={trim_window.right_events[0].timestamp_ns},"
                f" L2 open={trim_window.left_events[-1].timestamp_ns})"
            )
            if phase_only_names:
                raw_windows = task_phase_windows_for_bag(
                    bag,
                    metadata,
                    required_topics,
                    phase_trim,
                    phase_only_names,
                    round(args.phase_only_context_before_sec * 1e9),
                    round(args.phase_only_context_after_sec * 1e9),
                )
                windows: dict[str, PhaseTrimWindow] = {}
                for name in phase_only_names:
                    raw = raw_windows[name]
                    start_ns = max(raw.start_ns, trim_window.start_ns)
                    stop_ns = min(raw.stop_ns, trim_window.stop_ns)
                    if stop_ns <= start_ns:
                        raise ValueError(f"Empty phase-only clip for {bag.name}/{name}")
                    windows[name] = PhaseTrimWindow(
                        start_ns,
                        stop_ns,
                        trim_window.right_events,
                        trim_window.left_events,
                    )
                phase_only_windows[bag] = windows
                duration = sum(
                    (window.stop_ns - window.start_ns) / 1e9
                    for window in windows.values()
                )
                trim_summary += (
                    f", phase_only={','.join(phase_only_names)}"
                    f" ({duration:.2f}s)"
                )
        total_overlap += duration
        print(
            f"[OK] {bag.name}: overlap={result['overlap_sec']:.2f}s, "
            f"estimated_frames={result['estimated_frames']}, "
            f"images={sorted(set(result['image_sizes'].values()))}"
            f"{trim_summary}"
        )
    print(
        f"Validation passed: {len(bags)} episode(s), "
        f"common duration={total_overlap / 60.0:.2f} min"
    )
    supplemental_windows: dict[Path, dict[str, PhaseTrimWindow]] = {}
    supplemental_duration = 0.0
    for bag in supplemental_bags:
        result = validate_bag(
            bag, camera_topics, max_skew_sec, float(config["dataset"]["fps"])
        )
        metadata = metadata_from_db(db_path_for_bag(bag))
        windows = task_phase_windows_for_bag(
            bag,
            metadata,
            required_topics,
            phase_trim,
            supplemental_phases,
            round(args.supplemental_phase_context_before_sec * 1e9),
            round(args.supplemental_phase_context_after_sec * 1e9),
        )
        if set(windows) != set(supplemental_phases):
            raise ValueError(
                f"{bag.name}: did not produce every requested supplemental phase"
            )
        supplemental_windows[bag] = windows
        clip_duration = sum(
            (window.stop_ns - window.start_ns) / 1e9
            for window in windows.values()
        )
        supplemental_duration += clip_duration
        print(
            f"[OK supplemental] {bag.name}: overlap={result['overlap_sec']:.2f}s, "
            f"phases={len(windows)}, clip_duration={clip_duration:.2f}s"
        )
    if supplemental_bags:
        print(
            "Supplemental validation passed: "
            f"recordings={len(supplemental_bags)}, "
            f"phase_clips={sum(len(value) for value in supplemental_windows.values())}, "
            f"materialized_duration={supplemental_duration / 60.0:.2f} min"
        )
    if args.validate_only:
        return

    if output == bag_root or bag_root in output.parents:
        raise ValueError("Output must not be the input bag directory or one of its children")
    converter = OpenArmConverter(
        config,
        output,
        args.overwrite,
        append_event_memory_state=args.append_event_memory_state,
    )
    total_frames = 0
    total_skipped = 0
    base_episodes = 0
    for index, bag in enumerate(bags, start=1):
        if phase_only_names:
            for phase_name in phase_only_names:
                print(
                    f"Converting phase-only {index}/{len(bags)}: "
                    f"{bag.name}/{phase_name}"
                )
                frames, skipped = converter.convert_bag(
                    bag, phase_only_windows[bag][phase_name]
                )
                base_episodes += 1
                total_frames += frames
                total_skipped += skipped
                print(f"  saved={frames}, skipped_for_sync={skipped}")
        else:
            print(f"Converting {index}/{len(bags)}: {bag.name}")
            frames, skipped = converter.convert_bag(bag, trim_windows.get(bag))
            base_episodes += 1
            total_frames += frames
            total_skipped += skipped
            print(f"  saved={frames}, skipped_for_sync={skipped}")

    if args.phase_balance_manifest:
        manifest_path = args.phase_balance_manifest.expanduser().resolve()
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        requested_phases = tuple(
            value.strip()
            for value in args.phase_balance_names.split(",")
            if value.strip()
        )
        if not requested_phases:
            raise ValueError("--phase-balance-names must not be empty")
        unknown = set(requested_phases) - {
            str(phase["name"])
            for episode in manifest.get("episodes", [])
            for phase in episode.get("trimmed_phases", [])
        }
        if unknown:
            raise ValueError(f"Unknown balanced phases: {sorted(unknown)}")

        details_by_source = {
            str(episode["source_episode"]): episode
            for episode in manifest.get("episodes", [])
            if episode.get("split") == "train"
        }
        selected_names = {bag.name for bag in bags}
        missing = selected_names - set(details_by_source)
        if missing:
            raise ValueError(
                "Phase balance manifest does not mark selected bags as train: "
                f"{sorted(missing)}"
            )

        fps = float(config["dataset"]["fps"])
        before_frames = round(args.phase_context_before_sec * fps)
        after_frames = round(args.phase_context_after_sec * fps)
        clip_frames_by_phase: dict[str, int] = {name: 0 for name in requested_phases}
        phase_rows: dict[tuple[str, str], dict[str, Any]] = {}
        for bag in bags:
            details = details_by_source[bag.name]
            full_frames = int(details["trim"]["frames"])
            for phase in details.get("trimmed_phases", []):
                name = str(phase["name"])
                if name not in clip_frames_by_phase:
                    continue
                start = max(0, int(phase["start_frame"]) - before_frames)
                stop = min(full_frames, int(phase["stop_frame"]) + after_frames)
                if stop <= start:
                    raise ValueError(f"Empty balanced clip for {bag.name}/{name}")
                phase_rows[(bag.name, name)] = {
                    "start_frame": start,
                    "stop_frame": stop,
                }
                clip_frames_by_phase[name] += stop - start

        if any(value <= 0 for value in clip_frames_by_phase.values()):
            raise ValueError(
                f"Balanced phases are missing train frames: {clip_frames_by_phase}"
            )
        target_frames = max(clip_frames_by_phase.values())
        extra_repeats = {
            name: max(0, math.ceil(target_frames / frames) - 1)
            for name, frames in clip_frames_by_phase.items()
        }
        print(
            "Phase balance plan: "
            + ", ".join(
                f"{name}=+{extra_repeats[name]} clip copy/copies "
                f"({clip_frames_by_phase[name]} frames/copy-set)"
                for name in requested_phases
            )
        )

        extra_episodes = 0
        extra_frames = 0
        for name in requested_phases:
            for repeat in range(extra_repeats[name]):
                for bag in bags:
                    full_window = trim_windows.get(bag)
                    if full_window is None:
                        raise ValueError(
                            "Phase balancing requires phase_trim.enabled in the config"
                        )
                    row = phase_rows[(bag.name, name)]
                    clip = PhaseTrimWindow(
                        start_ns=full_window.start_ns
                        + round(row["start_frame"] / fps * 1e9),
                        stop_ns=full_window.start_ns
                        + round(row["stop_frame"] / fps * 1e9),
                        right_events=full_window.right_events,
                        left_events=full_window.left_events,
                    )
                    print(
                        f"Balanced clip {name} repeat {repeat + 1}/"
                        f"{extra_repeats[name]}: {bag.name}"
                    )
                    frames, skipped = converter.convert_bag(bag, clip)
                    extra_episodes += 1
                    extra_frames += frames
                    total_frames += frames
                    total_skipped += skipped
        print(
            f"Phase balance materialized: extra_episodes={extra_episodes}, "
            f"extra_frames={extra_frames}"
        )
    supplemental_episodes = 0
    supplemental_frames = 0
    for bag in supplemental_bags:
        for phase_name in supplemental_phases:
            print(f"Supplemental phase clip {bag.name}/{phase_name}")
            frames, skipped = converter.convert_bag(
                bag, supplemental_windows[bag][phase_name]
            )
            supplemental_episodes += 1
            supplemental_frames += frames
            total_frames += frames
            total_skipped += skipped
    if supplemental_bags:
        print(
            "Supplemental phases materialized: "
            f"episodes={supplemental_episodes}, frames={supplemental_frames}"
        )
    converter.finalize()
    print(
        f"Done: base_episodes={base_episodes}, frames={total_frames}, "
        f"skipped={total_skipped}, output={output}"
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise
