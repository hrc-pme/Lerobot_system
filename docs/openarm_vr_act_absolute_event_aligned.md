# OpenArm VR ACT — event-aligned absolute-action pipeline

This branch contains the offline OpenArm VR → LeRobot ACT pipeline. It does **not** contain rosbag recordings, converted datasets, checkpoints, experiment outputs, calibration files, or a hardware-execution entrypoint. Nothing in the workflow below initializes a robot command publisher.

## Contract

- Input: ten recorded VR episodes `001`–`010`, three RGB cameras, and measured OpenArm16 joint state.
- Target: the previous real `ros2_control` **absolute position command** at each policy tick, not a state delta or a future/interpolated command.
- Split: TRAIN `001`–`008`, validation `009`, test `010`.
- Policy clock: 15 Hz. The converter linearly samples measured state only; it zero-order holds the previous recorded controller command.
- Event alignment: remove each episode's motionless prefix so frame 0 precedes the initial two-gripper opening and frame 1 is the opening action. Original numeric states/actions and images are copied, not relabeled.
- Model: standard LeRobot ACT, trained from scratch with 16D state/action, three RGB cameras, 30-step chunk/action horizon, VAE enabled, no temporal ensemble.

## Files and order

1. `scripts/ros2_to_lerobot_openarm.py` and `scripts/ros2_to_lerobot_openarm_vr_native_absolute_15fps_config.yaml`: convert the recorded bags. `scripts/run_openarm_vr_native_absolute_dataset.sh` orchestrates validation, build and provenance audit; `scripts/audit_openarm_vr_native_absolute_contract.py` checks the absolute-action contract.
2. `scripts/build_openarm_vr_native_event_aligned_dataset.py` provides the shared event detector and dataset helpers. `scripts/build_openarm_vr_native_absolute_event_aligned_dataset.py` makes the absolute-action event-aligned copy and a source-provenance report.
3. `scripts/run_openarm_vr_native_absolute_event_aligned_act.sh` checks the materialized dataset, trains ACT, and runs offline validation. `scripts/validate_openarm_vr_native_absolute_act_offline.py` uses the two included OpenArm offline/safety helpers.

The scripts expect the original bags under `Dataset/Raw_rosbag/openarm_vr_native_baseline_v1/` and LeRobot/ROS 2 Humble dependencies in the project environment. The raw dataset and model weights must be supplied separately with appropriate sharing permission; they are intentionally absent from Git. Offline joint-limit validation also expects an OpenArm URDF at `repos/openarm_dev/openarm_urdf/openarm_bimanual.urdf`. **`openarm_dev` is not present in the upstream `main` tree or included in this code-only package**; obtain it separately under the project's access and licensing rules before running that validation or its URDF-dependent tests.

For a fresh offline build, first run `scripts/run_openarm_vr_native_absolute_dataset.sh all`. Then run `python3 scripts/build_openarm_vr_native_absolute_event_aligned_dataset.py`, followed by `python3 scripts/audit_openarm_vr_native_absolute_contract.py --dataset-root Dataset/Converted_dataset/openarm_vr_native_absolute_event_aligned_v1 --raw-root Dataset/Raw_rosbag/openarm_vr_native_baseline_v1 --output output/openarm_native_absolute_audit_v1/event_aligned_dataset_contract_v1.json`. Finally, run `scripts/run_openarm_vr_native_absolute_event_aligned_act.sh contract` before `smoke`, `train`, or `offline-validation`.

All build/train steps create local `Dataset/` or `output/` artifacts. Do not stage those directories for this code-only branch. This package is **not** an authorization or a complete recipe for controlling physical OpenArm hardware.
