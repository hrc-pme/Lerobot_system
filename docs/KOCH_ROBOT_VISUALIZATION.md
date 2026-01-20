# Koch Robot 可視化系統說明手冊

本文件說明如何將真實世界中的 Koch 機器手臂動作，即時投影到 ROS2 的虛擬環境 (RViz) 中。

此系統解決了以下挑戰：
1.  **名稱不匹配**: 真實馬達回傳的名稱 (如 `right_follower_shoulder`) 與 URDF 模型名稱 (`joint1`) 不同。
2.  **零點與方向差異**: 真實組裝角度與模擬模型的零點定義通常有誤差。
3.  **多手臂呈現**: 在同一個視窗中同時顯示左手與右手，且不發生衝突。

---

## 核心程式邏輯

### 1. 關節訊號轉換器 (`real_robot_remapper.py`)
*   **位置**: `src/koch_simulation/koch_simulation/real_robot_remapper.py`
*   **功能**: 這是視覺化的核心中樞。它負責：
    *   **Remapping**: 將 LeRobot 的長名稱轉換為 URDF 的短名稱。
    *   **Calibration**: 修正馬達方向 (`Direction`) 與 物理零點偏移 (`Offset`)。
    *   **Prefixing**: 為轉換後的名稱加上前綴 (如 `left_joint1`) 以支援多手臂。

#### 如何校正角度？
若您發現虛擬手臂的動作與真實手臂不一致，請修改此檔案中的 `self.mapping` 表格：

```python
self.mapping = {
    # 格式: '真實後綴': ('URDF名稱', 方向, 偏移量弧度)
    # 方向: 1 (正常) 或 -1 (反轉)
    # 偏移量: 90度 約為 1.57, 45度 約為 0.785
    'shoulder_pan':  ('joint1', -1, 0.0),
    'shoulder_lift': ('joint2', -1, 1.57), 
    # ...
}
```
*修改後需重新編譯生效 (`colcon build`)。*

### 2. 支援前綴的 URDF 模型 (`low_cost_robot.xacro`)
*   **位置**: `src/koch_simulation/urdf/low_cost_robot.xacro`
*   **功能**: 我們修改了原始模型，增加了 `prefix` 參數。這允許我們產生 `left_base_link` 和 `right_base_link`，避免兩隻手臂的座標系名稱衝突。

---

## 使用教學

在使用視覺化功能前，請務必先啟動底層驅動（`leader_follower-teleop.sh`）。

### 情境 A：顯示單一手臂
適用於只測試單手，或快速除錯時。

```bash
# 顯示 Follower 左手 (或其他 topic)
ros2 launch koch_simulation real_robot_display.launch.py source_topic:=/left_follower/joint_states
```

### 情境 B：顯示雙手 (Dual Arms)
這是最完整的展示模式，會同時顯示左手與右手，並將其分開擺放。

```bash
ros2 launch koch_simulation dual_real_robot_display.launch.py
```

#### 💡 RViz 設定指南 (重要)
第一次啟動雙手模式時，RViz 可能只會顯示一隻手臂。請依照以下步驟設定：

1.  **設定 Fixed Frame**: 將 Global Options 中的 Fixed Frame 改為 `world`。
2.  **新增左手模型**:
    *   點擊 `Add` -> 選擇 `RobotModel`。
    *   將其 `Description Topic` 改為 `/left/robot_description`。
3.  **新增右手模型**:
    *   再次點擊 `Add` -> 選擇 `RobotModel`。
    *   將其 `Description Topic` 改為 `/right/robot_description`。
4.  **調整位置 (選用)**:
    *   若覺得兩隻手臂太近或太遠，可修改 `dual_real_robot_display.launch.py` 中的 `static_transform_publisher` 參數 (X/Y/Z)。

---

## 檔案結構概覽

```text
src/koch_simulation/
├── launch/
│   ├── display.launch.py            # (舊) 僅顯示純模擬模型
│   ├── real_robot_display.launch.py # 單臂實時顯示
│   └── dual_real_robot_display.launch.py # 雙臂實時顯示 (推薦)
├── koch_simulation/
│   └── real_robot_remapper.py       # 轉換節點原始碼
└── urdf/
    └── low_cost_robot.xacro         # 機器人 3D 模型描述
```
