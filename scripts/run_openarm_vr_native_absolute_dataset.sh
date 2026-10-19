#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

MODE="${1:-}"
CONFIG="scripts/ros2_to_lerobot_openarm_vr_native_absolute_15fps_config.yaml"
DATASET_ROOT="Dataset/Converted_dataset/openarm_vr_native_absolute_v1"
STAGING_ROOT="${DATASET_ROOT}.building"
RAW_ROOT="Dataset/Raw_rosbag/openarm_vr_native_baseline_v1"
AUDIT_ROOT="output/openarm_native_absolute_audit_v1"
REPORT="$AUDIT_ROOT/dataset_contract.json"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

usage() {
  echo "Usage: $0 {unit|validate|build|audit|contract|all}"
  echo "Every mode is offline: no ROS node or hardware command publisher is created."
}

unit_checks() {
  python3 -m py_compile \
    scripts/ros2_to_lerobot_openarm.py \
    scripts/audit_openarm_vr_native_absolute_contract.py
  python3 -m unittest discover \
    -s tests \
    -p 'test_openarm_absolute_command_sampling.py' \
    -v
}

validate_bags() {
  python3 scripts/ros2_to_lerobot_openarm.py \
    --config "$CONFIG" \
    --validate-only
}

build_dataset() {
  if [[ -e "$DATASET_ROOT" ]]; then
    echo "Refusing to overwrite existing absolute dataset: $DATASET_ROOT" >&2
    return 2
  fi
  if [[ -e "$STAGING_ROOT" ]]; then
    echo "Refusing to reuse incomplete staging directory: $STAGING_ROOT" >&2
    return 2
  fi

  python3 scripts/ros2_to_lerobot_openarm.py \
    --config "$CONFIG" \
    --episodes 001,002,003,004,005,006,007,008 \
    --repo-id openarm_vr_native_absolute_train_001_008 \
    --output "$STAGING_ROOT/train"

  python3 scripts/ros2_to_lerobot_openarm.py \
    --config "$CONFIG" \
    --episodes 009 \
    --repo-id openarm_vr_native_absolute_validation_009 \
    --output "$STAGING_ROOT/validation"

  python3 scripts/ros2_to_lerobot_openarm.py \
    --config "$CONFIG" \
    --episodes 010 \
    --repo-id openarm_vr_native_absolute_test_010 \
    --output "$STAGING_ROOT/test"

  mv "$STAGING_ROOT" "$DATASET_ROOT"
  echo "ABSOLUTE DATASET BUILD COMPLETE: $DATASET_ROOT"
  echo "No ROS node or hardware command publisher was created."
}

audit_dataset() {
  mkdir -p "$AUDIT_ROOT"
  python3 scripts/audit_openarm_vr_native_absolute_contract.py \
    --dataset-root "$DATASET_ROOT" \
    --raw-root "$RAW_ROOT" \
    --output "$REPORT"
}

contract_check() {
  python3 - "$CONFIG" "$DATASET_ROOT" "$REPORT" <<'PY'
import json
from pathlib import Path
import sys
import yaml

config = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
root = Path(sys.argv[2])
report = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
assert config["mapping"]["action_source"] == "controller_command"
assert config["sync"]["controller_command_sampling"] == "previous"
assert config["phase_trim"]["enabled"] is False
assert config["robot"]["state_dim"] == 16
assert config["robot"]["action_dim"] == 16
assert report["pass"] is True, report["failures"]
expected = {"train": 8, "validation": 1, "test": 1}
for split, episodes in expected.items():
    row = report["splits"][split]
    assert row["episodes"] == episodes
    assert row["state_shape"] == [16]
    assert row["action_shape"] == [16]
    assert row["future_controller_commands_used"] is False
    assert row["max_raw_action_comparison_error"] <= 1e-7
    assert row["max_measured_state_interpolation_error"] <= 1e-6
    assert row["max_command_age_sec"] <= 0.05
    assert row["max_camera_skew_sec"] <= 0.12
    assert all(item["queue_reconstruction_exact"] for item in row["episodes_sequence_audit"])
    assert (root / split / "absolute_command_contract.json").is_file()
    assert (root / split / "absolute_command_provenance.jsonl").is_file()
print("OPENARM VR-NATIVE ABSOLUTE-CONTROLLER DATASET CONTRACT PASS")
print("state=measured OpenArm16; action=real ZOH absolute controller OpenArm16")
print("train=001..008; validation=009; test=010; chunk audit=30 ordered steps")
print("OFFLINE ONLY: no ROS node or hardware command publisher was created.")
PY
}

case "$MODE" in
  unit)
    unit_checks
    ;;
  validate)
    validate_bags
    ;;
  build)
    build_dataset
    ;;
  audit)
    audit_dataset
    ;;
  contract)
    contract_check
    ;;
  all)
    unit_checks
    validate_bags
    build_dataset
    audit_dataset
    contract_check
    ;;
  *)
    usage
    exit 2
    ;;
esac
