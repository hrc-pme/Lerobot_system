# Koch Robot Control System 使用手冊

本文件說明 Koch 機器人 ROS2 控制系統的核心程式、運作邏輯與使用方式。本系統基於 LeRobot API 構建，透過 ROS2 節點進行通訊與控制。

## 系統架構概觀

系統主要由以下三個部分組成：
1. **硬體驅動層 (Driver)**: `koch_leader_follower_control`
2. **邏輯控制層 (Bridge)**: `koch_teleop_bridge`
3. **工具與腳本 (Tools)**: `koch_calibration`, `leader_follower-teleop.sh`

---

## 1. 啟動腳本 (Launcher)

### 檔案: `leader_follower-teleop.sh`
* **位置**: `/home/hrc/Lerobot_system/entrypoint/leader_follower-teleop.sh`

#### 功能
這是系統的主進入點，用於一鍵啟動整個遙操作 (Teleoperation) 系統。它整合了環境變數設定、ROS2 環境載入，並提供互動式選單讓使用者選擇操作模式。

#### 邏輯
1. **環境設定**: 自動 `source` ROS2 (Humble) 與 workspace 的 setup script，並設定 `PYTHONPATH` 以支援 LeRobot venv。
2. **模式選擇**: 提供選單讓使用者選擇：
   - `1. Single Right Arm`: 僅啟動右臂。
   - `2. Single Left Arm`: 僅啟動左臂。
   - `3. Dual Arms`: 啟動雙臂模式。
3. **參數配置**: 根據選擇的模式，自動決定要載入哪一個 YAML 設定檔 (如 `two_leader_follower.yaml`) 以及傳遞給 Bridge 的配對參數 (Pairs)。
4. **行程管理**: 在背景同時啟動 Driver 與 Bridge 節點，並在腳本結束 (Ctrl+C) 時自動關閉所有相關行程。

#### 如何使用
在終端機執行：
```bash
./entrypoint/leader_follower-teleop.sh
```
依照螢幕指示輸入 `1`、`2` 或 `3` 即可。

---

## 2. 統一硬體驅動節點 (Unified Driver)

### 檔案: `koch_leader_follower_control.py`
* **位置**: `src/koch_control/koch_control/koch_leader_follower_control.py`
* **ROS Node**: `leader_follower_unified_node`

#### 功能
負責與實體 Dynamixel 馬達進行通訊。它同時管理 Leader (主導端) 與 Follower (跟隨端) 的硬體介面。

#### 邏輯
1. **設定讀取**: 讀取 YAML 設定檔 (定義了手臂名稱、USB Port、馬達 ID)。
2. **Leader 模式**:
   - 關閉 Torque (被動模式)，讓使用者可以手動移動手臂。
   - 持續讀取馬達角度，發佈到 `/<name>/joint_states` topic。
3. **Follower 模式**:
   - 開啟 Torque (主動模式)，維持位置。
   - 訂閱 `/<name>/joint_states_control` topic，接收目標角度並寫入馬達。
4. **校正載入**: 根據手臂名稱自動尋找位於 `/home/hrc/Lerobot_system/calibration/koch/...` 的 JSON 校正檔並套用。

#### 如何使用
通常由啟動腳本呼叫，若需單獨測試：
```bash
ros2 run koch_control koch_leader_follower_control --ros-args -p config_file:=/path/to/config.yaml
```

---

## 3. 遙操作橋接節點 (Teleop Bridge)

### 檔案: `koch_teleop_bridge.py`
* **位置**: `src/koch_control/koch_control/koch_teleop_bridge.py`
* **ROS Node**: `koch_teleop_bridge`

#### 功能
這是遙操作的「大腦」，負責將 Leader 的動作傳遞給 Follower。它將 Leader 的狀態訊息 (Sensor Data) 轉換為 Follower 的控制命令 (Control Command)。

#### 邏輯
1. **動態配對**: 透過 `pairs` 參數接收配對清單 (例如 `["right_leader:right_follower"]`)。
2. **狀態轉發**:
   - 訂閱 Leader 的 `/<leader>/joint_states`。
   - 當收到訊息時，將關節名稱中的 `leader` 替換為 `follower` (例如 `right_leader_shoulder` -> `right_follower_shoulder`)。
   - 將修改後的訊息發佈到 Follower 的控制接口 `/<follower>/joint_states_control`。

#### 如何使用
通常由啟動腳本呼叫，若需手動執行：
```bash
ros2 run koch_control koch_teleop_bridge --ros-args -p pairs:="['right_leader:right_follower']"
```

---

## 4. 校正工具 (Calibration Tool)

### 檔案: `koch_calibration.py`
* **位置**: `src/koch_control/koch_control/koch_calibration.py`

#### 功能
提供互動式介面，協助使用者對機器手臂進行物理校正 (紀錄馬達範圍與零點)。這是 LeRobot 系統運作準確的前提。

#### 邏輯
1. **讀取硬體表**: 解析 YAML 設定檔，列出所有可用的手臂。
2. **手臂選擇**: 如果偵測到多隻手臂 (例如同時有左右手)，會跳出選單讓使用者指定要校正哪一隻 (透過 `select_arm` 函式)。
3. **調用 LeRobot API**: 使用 `lerobot.robot.KochLeader` 或 `KochFollower` 類別執行標準校正程序。
4. **檔案儲存**: 校正結果會自動儲存至 `/home/hrc/Lerobot_system/calibration/koch/{role}/` 目錄下。

#### 如何使用
```bash
ros2 run koch_control koch_calibration
```
執行後：
1. 選擇要校正的角色 (1. Follower, 2. Leader, 3. Both)。
2. 若該角色有多隻手臂，選擇要校正哪一隻 (例如 1. right_leader)。
3. 依照指示手動移動手臂進行校正。

---

## 設定檔說明

位於 `src/koch_control/config/`：
- **`two_leader_follower.yaml`**: 定義雙手系統 (Right Leader/Follower + Left Leader/Follower)。
- **`single_leader_follower.yaml`**: 定義單右手系統。

每份設定檔包含：
- **`name`**: 手臂名稱 (如 `right_leader`)。
- **`port`**: USB 裝置路徑 (如 `/dev/ttykoch_leader_right`)。
- **`motors`**: 馬達名稱與 ID 的對應表。
