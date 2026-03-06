# ROS2 Bag to LeRobot Dataset Conversion Tools | ROS2 轉 LeRobot 數據集轉換工具

This directory contains tools to convert ROS2 bag files (likely from Koch robot teleoperation) into the LeRobot dataset format. There are two conversion modes available: **Joint Space** and **End-Effector (SE3) Space**.

本目錄包含將 ROS2 bag 檔案（通常來自 Koch 機器人遙操作）轉換為 LeRobot 數據集格式的工具。提供兩種轉換模式：**關節空間 (Joint Space)** 與 **末端執行器空間 (End-Effector Space)**。

## 1. Dependencies | 相依套件

To run these scripts, you need a standard ROS 2 environment (Humble suggested) with Python 3.

若要執行這些腳本，您需要標準的 ROS 2 環境（建議使用 Humble 版本）以及 Python 3。

### Python Packages | Python 套件
Add the following to your `requirements.txt` or install them in your Dockerfile:

請將以下內容加入您的 `requirements.txt` 或在 Dockerfile 中安裝：

```txt
numpy
opencv-python
pyyaml
torch
lerobot          # Assuming working within the LeRobot repo context | 假設在 LeRobot repo 環境下工作
kinpy            # Required for End-Effector conversion (Forward Kinematics) | 末端轉換（正向運動學）所需
scipy            # Required for Rotation conversions | 旋轉轉換所需
```

### ROS Dependencies | ROS 相依套件
Ensure the following ROS packages are installed (usually part of `ros-humble-desktop` or standard installation)：

請確保已安裝以下 ROS 套件（通常包含在 `ros-humble-desktop` 或標準安裝中）：

- `rosbag2_py`
- `cv_bridge`
- `sensor_msgs`
- `rclpy`

### Dockerfile Snippet | Dockerfile 片段
If you are updating your `Dockerfile`, add `kinpy` and `scipy`:

如果您正在更新 `Dockerfile`，請加入 `kinpy` 和 `scipy`：

```dockerfile
# ... existing installation ...
RUN pip install kinpy scipy
```

---

## 2. Tools Overview | 工具總覽

### Mode A: Joint Space Conversion | 模式 A：關節空間轉換
Converts raw motor angles directly to the dataset state. Best for basic imitation learning.
直接將原始馬達角度轉換為數據集狀態。最適合基礎的模仿學習。

*   **Script | 腳本**: `ros2_to_lerobot.py`
*   **Config | 設定檔**: `ros2_to_lerobot_config.yaml`
*   **State/Action | 狀態/動作**: [joint1, joint2, ..., joint6] (Dimension: 6 or 12 depending on bimanual | 維度：6 或 12，視雙臂而定)

### Mode B: End-Effector Space Conversion (Cross-Embodiment) | 模式 B：末端執行器空間轉換（跨實體）
Calculates Forward Kinematics (FK) using the robot's URDF to convert joint angles into End-Effector Pose (x, y, z, rx, ry, rz) + Gripper.
利用機器人的 URDF 計算正向運動學 (FK)，將關節角度轉換為末端姿態 (x, y, z, rx, ry, rz) + 夾爪狀態。

*   **Script | 腳本**: `ros2_to_lerobot_ee.py`
*   **Config | 設定檔**: `ros2_to_lerobot_ee_config.yaml`
*   **State/Action | 狀態/動作**: [x, y, z, rx, ry, rz, gripper] (Dimension: 7 per arm, 14 total | 維度：每臂 7，總計 14)
*   **Note | 備註**: Requires valid URDF files located at `/tmp/koch_left.urdf` and `/tmp/koch_right.urdf`. | 需要有效的 URDF 檔案位於 `/tmp/koch_left.urdf` 和 `/tmp/koch_right.urdf`。

---

## 3. Configuration Files | 設定檔說明

### `ros2_to_lerobot_config.yaml`
Standard configuration for joint-space training.
用於關節空間訓練的標準設定。

*   **root**: Where to save the converted dataset. | 儲存轉換後數據集的位置。
*   **topics**: Maps ROS topics (`/left_follower/joint_states`) to dataset keys. | 將 ROS 主題對應到數據集鍵值。

### `ros2_to_lerobot_ee_config.yaml`
Configuration for end-effector space training.
用於末端執行器空間訓練的設定。

*   **state_dim / action_dim**: Set to **14** (7 per arm). | 設定為 **14**（每臂 7）。
*   **Note**: The script uses `kinpy` to compute FK on the fly based on the topics defined in 'topics'. | 腳本會使用 `kinpy` 根據 'topics' 定義的主題即時計算 FK。

---

## 4. Usage Instructions | 使用說明

### Prerequisite: Generate URDFs (For EE Mode only) | 前置作業：生成 URDF（僅限 EE 模式）
If running the **End-Effector** conversion, ensure URDFs exist in `/tmp/`:

若執行 **末端執行器** 轉換，請確保 `/tmp/` 中存在 URDF 檔案：

```bash
# Example command to generate URDFs from Xacro | 從 Xacro 生成 URDF 的範例指令
source ~/Lerobot_system/ros2_ws/install/setup.bash
xacro ~/Lerobot_system/ros2_ws/src/koch_simulation/urdf/low_cost_robot.xacro prefix:=left_ > /tmp/koch_left.urdf
xacro ~/Lerobot_system/ros2_ws/src/koch_simulation/urdf/low_cost_robot.xacro prefix:=right_ > /tmp/koch_right.urdf
```

### Run Conversion | 執行轉換

**Option 1: Joint Space | 選項 1：關節空間**
```bash
python3 scripts/ros2_to_lerobot.py --config scripts/ros2_to_lerobot_config.yaml
```

**Option 2: End-Effector Space | 選項 2：末端執行器空間**
```bash
python3 scripts/ros2_to_lerobot_ee.py --config scripts/ros2_to_lerobot_ee_config.yaml
```

**Note**: Ignore "Unknown tag 'hardwareInterface'" warnings when running the EE script; they are harmless `kinpy` logs.
**備註**：執行 EE 腳本時請忽略 "Unknown tag 'hardwareInterface'" 警告；這些是無害的 `kinpy` 日誌。

---

## 5. Output | 輸出結果
The scripts will generate a folder structure compatible with LeRobot training:
腳本將生成與 LeRobot 訓練相容的資料夾結構：

```
Dataset/
  converted_dataset_ee/
    meta/
      info.json
      stats.json
    videos/
    data/
```
You can then train using:
接著您可以使用以下指令進行訓練：

```bash
python3 lerobot/scripts/lerobot_train.py 
  --dataset.repo_id koch_bi_wipe_water_ee 
  --dataset.root /home/hrc/Lerobot_system/Dataset/converted_dataset_ee 
  ...
```
