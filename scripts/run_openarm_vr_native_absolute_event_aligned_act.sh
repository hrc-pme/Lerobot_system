#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

MODE="${1:-}"
TRAIN_REPO="openarm_vr_native_absolute_event_aligned_v1_train"
DATASET_ROOT="Dataset/Converted_dataset/openarm_vr_native_absolute_event_aligned_v1"
TRAIN_DATASET="$DATASET_ROOT/train"
DATASET_REPORT="output/openarm_native_absolute_audit_v1/event_aligned_dataset_contract_v1.json"
ALIGNMENT_REPORT="output/openarm_native_absolute_audit_v1/event_alignment_v1.json"
SMOKE_OUTPUT="output/openarm_vr_native_absolute_event_aligned_act_15fps_smoke"
TRAIN_OUTPUT="output/openarm_vr_native_absolute_event_aligned_act_15fps_50k"
OFFLINE_REPORT="output/openarm_native_absolute_audit_v1/event_aligned_checkpoint_050000_offline_validation.json"

usage() {
  echo "Usage: $0 {contract|smoke|smoke-contract|train|train-contract|offline-validation|monitor}"
  echo "Training/validation only; no ROS node or hardware command publisher is created."
}

contract() {
  python3 - "$TRAIN_DATASET" "$DATASET_REPORT" "$ALIGNMENT_REPORT" <<'PY'
import json
from pathlib import Path
import sys
import pyarrow.parquet as pq
from lerobot.policies.act.configuration_act import ACTConfig

dataset = Path(sys.argv[1])
dataset_report = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
alignment = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
assert dataset_report["pass"] is True, dataset_report["failures"]
assert alignment["materialized"] is True
assert alignment["operation"] == "prefix_trim_only_no_relabel"
assert alignment["action_semantics"] == "absolute ros2_control position target"
assert alignment["future_controller_commands_used"] is False
assert alignment["interpolation_added"] is False
assert alignment["synthetic_labels_added"] is False
for split in ("train", "validation", "test"):
    row = dataset_report["splits"][split]
    assert row["action_semantics"] == "absolute ros2_control position target"
    assert row["future_controller_commands_used"] is False
    assert row["max_raw_action_comparison_error"] == 0.0
    assert row["max_provenance_action_error"] == 0.0
    assert row["max_measured_state_interpolation_error"] <= 1e-6
    assert all(item["queue_reconstruction_exact"] for item in row["episodes_sequence_audit"])
    assert (dataset.parent / split / "absolute_command_contract.json").is_file()
    assert (dataset.parent / split / "absolute_command_provenance.jsonl").is_file()
    assert (dataset.parent / split / "event_alignment_contract.json").is_file()

paths = sorted((dataset / "data").glob("chunk-*/*.parquet"))
table = pq.read_table([str(path) for path in paths], columns=["episode_index", "frame_index", "action"])
episode = table["episode_index"].to_numpy()
frame = table["frame_index"].to_numpy()
actions = table["action"].to_pylist()
for index in range(8):
    positions = [i for i, value in enumerate(episode) if int(value) == index]
    assert int(frame[positions[0]]) == 0 and int(frame[positions[1]]) == 1
    assert max(actions[positions[0]][7], actions[positions[0]][15]) < 0.008
    assert min(actions[positions[1]][7], actions[positions[1]][15]) >= 0.008

config = ACTConfig(
    chunk_size=30,
    n_action_steps=30,
    temporal_ensemble_coeff=None,
    push_to_hub=False,
)
assert config.chunk_size == 30
assert config.n_action_steps == 30
assert config.temporal_ensemble_coeff is None
assert config.use_vae is True
print("OPENARM EVENT-ALIGNED ABSOLUTE ACT TRAINING CONTRACT PASS")
print("Dataset transform: prefix trim only; frame0=pre-open, frame1=open initiation")
print("Standard LeRobot ACT scratch training: VAE on, chunk=30, n_action_steps=30, temporal ensemble=None")
print("Target: exact absolute ros2_control OpenArm16 command; no delta/latch/rebase/residual")
print("No ROS node or hardware command publisher was created.")
PY
}

train_command() {
  local output_dir="$1"
  local steps="$2"
  local save_freq="$3"
  local log_freq="$4"

  contract
  if [[ -e "$output_dir" || -e "${output_dir}.log" ]]; then
    echo "Refusing to overwrite existing training output: $output_dir" >&2
    return 2
  fi
  mkdir -p "$(dirname "$output_dir")"

  PYTHONUNBUFFERED=1 \
  PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
  lerobot-train \
    --dataset.repo_id "$TRAIN_REPO" \
    --dataset.root "$(pwd)/$TRAIN_DATASET" \
    --policy.type act \
    --policy.chunk_size 30 \
    --policy.n_action_steps 30 \
    --policy.push_to_hub false \
    --policy.device cuda \
    --policy.use_amp true \
    --output_dir "$(pwd)/$output_dir" \
    --job_name "$(basename "$output_dir")" \
    --batch_size 2 \
    --num_workers 2 \
    --steps "$steps" \
    --log_freq "$log_freq" \
    --save_freq "$save_freq" \
    --eval_freq "$steps" \
    --wandb.enable false \
    2>&1 | tee "${output_dir}.log"
}

checkpoint_contract() {
  local checkpoint="$1"
  python3 - "$checkpoint/config.json" <<'PY'
import json
from pathlib import Path
import sys

config = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
assert config["type"] == "act"
assert config["chunk_size"] == 30
assert config["n_action_steps"] == 30
assert config["temporal_ensemble_coeff"] is None
assert config["use_vae"] is True
assert config.get("pretrained_path") is None
assert config["input_features"]["observation.state"]["shape"] == [16]
assert config["output_features"]["action"]["shape"] == [16]
assert sorted(k for k in config["input_features"] if k.startswith("observation.images.")) == [
    "observation.images.camera1",
    "observation.images.camera2",
    "observation.images.camera3",
]
print("EVENT-ALIGNED STANDARD ACT CHECKPOINT CONTRACT PASS")
print("16D measured state -> 16D absolute controller target; native 30-action queue")
PY
}

case "$MODE" in
  contract)
    contract
    ;;
  smoke)
    train_command "$SMOKE_OUTPUT" 20 20 5
    checkpoint_contract "$SMOKE_OUTPUT/checkpoints/000020/pretrained_model"
    ;;
  smoke-contract)
    checkpoint_contract "$SMOKE_OUTPUT/checkpoints/000020/pretrained_model"
    ;;
  train)
    train_command "$TRAIN_OUTPUT" 50000 5000 100
    checkpoint_contract "$TRAIN_OUTPUT/checkpoints/050000/pretrained_model"
    ;;
  train-contract)
    checkpoint_contract "$TRAIN_OUTPUT/checkpoints/050000/pretrained_model"
    ;;
  offline-validation)
    contract
    checkpoint_contract "$TRAIN_OUTPUT/checkpoints/050000/pretrained_model"
    python3 scripts/validate_openarm_vr_native_absolute_act_offline.py \
      --checkpoint "$TRAIN_OUTPUT/checkpoints/050000/pretrained_model" \
      --dataset-root "$DATASET_ROOT" \
      --dataset-contract "$DATASET_REPORT" \
      --stride 1 \
      --splits validation test \
      --output "$OFFLINE_REPORT"
    ;;
  monitor)
    if [[ ! -f "${TRAIN_OUTPUT}.log" ]]; then
      echo "Training log not found: ${TRAIN_OUTPUT}.log" >&2
      exit 2
    fi
    tail -n 40 "${TRAIN_OUTPUT}.log"
    echo "===== CHECKPOINTS ====="
    find "$TRAIN_OUTPUT/checkpoints" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort
    ;;
  *)
    usage
    exit 2
    ;;
esac
