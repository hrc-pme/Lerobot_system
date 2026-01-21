# Koch Robot MoveIt 整合套件說明手冊

本文件詳細說明 `/src/koch_MoveIt` 資料夾下的套件架構、邏輯與使用方式。此套使 Koch 機器手臂能夠利用 ROS2 MoveIt 2 框架進行路徑規劃 (Motion Planning) 與末端點控制 (End-Effector Control)。

---

## 📂 套件結構概覽

`/src/koch_MoveIt` 包含兩個主要的子套件：

### 1. `koch_description`
*   **功能**: 存放機器人的物理描述文件。
*   **關鍵檔案**:
    *   `urdf/koch.urdf`: 機器人的主要 URDF 模型。**注意**: 此檔案已針對真實機器人進行修正，包含正確的關節限制 (Limits) 和原點偏移 (Origins)，以確保與真實機器人姿態同步。
    *   `meshes/`: 存放機器人外觀的 3D 模型檔案 (.stl)。

### 2. `koch_moveit_config`
*   **功能**: 由 MoveIt Setup Assistant 生成的設定檔，定義了規劃群組、控制器與啟動流程。
*   **關鍵設定**:
    *   `config/koch_v1.1.srdf`: 定義語意資訊，如規劃群組 (`koch_arm`, `gripper`)、預設姿態 (Home) 與碰撞免除規則。
    *   `config/ros2_controllers.yaml`: 定義 ROS2 Control 的控制器參數 (`JointTrajectoryController`)。
    *   `launch/`: 存放各種啟動腳本。

---

## 🚀 核心啟動檔：`real_robot_visualization.launch.py`

這是主要用於真實機器人控制與可視化的啟動檔。

### 啟動指令
```bash
ros2 launch koch_moveit_config real_robot_visualization.launch.py
```

### 系統架構與資料流
此啟動檔會同時拉起以下四個關鍵模組，並串聯成完整的控制迴路：

1.  **真實機器人驅動 (`koch_follower_control`)**
    *   負責與 Dynamixel 馬達通訊。
    *   讀取 `/calibration` 下的校準檔，將馬達原始值 (ticks) 轉換為角度 (radians)。
    *   **發布**: `/right_follower/joint_states` (原始名稱，如 `right_follower_shoulder_pan`)

2.  **MoveIt 橋接器 (`moveit_bridge`)** **(關鍵元件)**
    *   **位置**: `src/koch_control/koch_control/moveit_bridge.py`
    *   **邏輯**: 
        *   將真實機器人的長名稱 (如 `right_follower_shoulder_pan`) 轉換為 URDF 的短名稱 (如 `joint1`)。
        *   **執行座標轉換**: 應用偏移 (Offset) 與方向 (Direction) 修正，使真實機器人的 0 度與 MoveIt URDF 的 0 度對齊。
    *   **輸入**: `/right_follower/joint_states` (來自機器人) + `/fake_joint_states` (來自 MoveIt)
    *   **輸出**: `/joint_states` (給 MoveIt 看) + `/right_follower/joint_states_control` (給機器人執行)

3.  **ROS2 Control (Mock Hardware)**
    *   由於我們使用自定義的 Python Driver，我們在 MoveIt 端使用 `MockHardware` (FakeSystem) 來欺騙 MoveIt，讓它以為自己在控制一個標準的 ros2_control 硬體。
    *   橋接器負責將 MoveIt 發送給 Mock Hardware 的命令，攔截並轉發給真實機器人。

4.  **MoveIt 2 & RViz**
    *   **Move Group**: 核心規劃節點，負責計算路徑。
    *   **RViz**: 可視化介面，讓使用者拖拉末端點進行控制。

---

## 🔗 座標系與轉換邏輯 (Mapping Logic)

為了讓真實機器人 (Real) 與模擬模型 (Sim/MoveIt) 完美同步，`moveit_bridge.py` 實作了以下轉換邏輯：

1.  **Real -> MoveIt (Feedback)**:
    $MoveIt\_Angle = (Real\_Angle \times Direction) + Offset$

2.  **MoveIt -> Real (Command)**:
    $Real\_Command = (MoveIt\_Command - Offset) \times Direction$

**目前的轉換參數表** (已寫死在 `moveit_bridge.py`):

| 關節 (MoveIt/URDF) | 對應真實馬達名稱 (Real) | 方向 (Dir) | 偏移 (Offset) | 備註 |
| :--- | :--- | :---: | :---: | :--- |
| `joint1` | `shoulder_pan` | -1 | 0.0 | 反向 |
| `joint2` | `shoulder_lift` | -1 | +1.57 | 反向 + 補償 90 度 |
| `joint3` | `elbow_flex` | 1 | +1.57 | 補償 90 度 |
| `joint4` | `wrist_flex` | -1 | 0.0 | 反向 |
| `joint5` | `wrist_roll` | 1 | 0.0 | 同向 |
| `joint_gripper` | `gripper` | 1 | +0.5 | 夾爪偏移 |

> **注意**: 這些參數與 `koch_simulation` 套件中的 `real_robot_remapper.py` 邏輯完全一致，確保了 teleop 模式與 MoveIt 模式下的姿態表現相同。

---

## 🛠️ 常見問題排除

### 1. 啟動時報錯 `calibration directory not found`
*   **原因**: Launch file 中的路徑參數設定錯誤。
*   **解法**: 確保 `real_robot_visualization.launch.py` 中的 `calibration_dir` 指向 `/home/hrc/Lerobot_system/calibration` (不包含 `koch` 或 `follower` 子目錄)。

### 2. RViz 中的機器人是散架的 / 姿態很奇怪
*   **原因**: `koch_description` 中的 URDF 定義與真實機器人不匹配，或者 Bridge 沒有正確執行座標轉換。
*   **解法**:
    *   檢查 `koch.urdf` 中的 `joint` 類型是否為 `revolute` (而非 continuous)。
    *   確保 `moveit_bridge` 節點正在運行，且使用了正確的 mapping 參數。

### 3. MoveIt 規劃成功但機器人不動
*   **原因**: Bridge 沒有正確轉發 `/fake_joint_states` 到 `/right_follower/joint_states_control`。
*   **解法**: 使用 `ros2 topic echo /right_follower/joint_states_control` 檢查是否有命令送出。

---

## 📂 其他 Launch 檔案說明

除了核心的 `real_robot_visualization.launch.py` 外，本套件也提供了一些標準的 MoveIt 輔助工具：

### 1. 純模擬測試 (`demo.launch.py`)
*   **指令**: `ros2 launch koch_moveit_config demo.launch.py`
*   **用途**: 在不連接真實機器人的情況下，啟動一個完全虛擬的控制環境。
*   **特點**: 使用 MoveIt 內建的 Fake Controller，適合用來測試路徑規劃演算法或 UI 設定，而不用擔心硬體問題。

### 2. MoveIt 設定助理 (`setup_assistant.launch.py`)
*   **指令**: `ros2 launch koch_moveit_config setup_assistant.launch.py`
*   **用途**: 開啟圖形化介面，用來重新產生或修改 MoveIt 設定檔 (SRDF)。
*   **適用情境**:
    *   增加新的 Planning Group。
    *   定義新的 Robot Poses (如 Home, Sleep)。
    *   重新生成 Self-Collision Matrix。

### 3. Move Group 核心 (`move_group.launch.py`)
*   **指令**: `ros2 launch koch_moveit_config move_group.launch.py`
*   **用途**: 只啟動 MoveIt 的核心規劃服務 (Move Group Node)。
*   **適用情境**: 當您已有其他的 RViz 視窗或只需要後台規劃服務 (Headless Mode) 時使用。

### 4. 僅 RViz 可視化 (`moveit_rviz.launch.py`)
*   **指令**: `ros2 launch koch_moveit_config moveit_rviz.launch.py`
*   **用途**: 只開啟 RViz 視窗並載入 MoveIt 插件。
*   **備註**: 通常需配合 `move_group.launch.py` 使用，否則 RViz 內無法進行規劃。

---

## 📝 開發者筆記

若您修改了 URDF (例如調整了 Link 長度)，請務必：
1.  修改 `src/koch_MoveIt/koch_description/urdf/koch.urdf`。
2.  重新編譯: `colcon build --packages-select moveit_resources_koch_description`。
3.  若關節零點發生變化，請同步更新 `moveit_bridge.py` 中的 offset 參數。
