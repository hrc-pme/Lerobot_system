# End-Effector Control & Consistency System (Pinocchio-based)

本目錄包含一套基於 **Pinocchio** 動力學庫的「閉迴路一致性系統」，用於驗證機器人從 **Joint Space** 到 **Task Space (End-Effector)** 的轉換與逆向還原能力。

## 🎯 核心目標
確保 LeRobot 訓練出的 End-Effector (EE) 軌跡，能夠透過逆向運動學 (IK) 精準還原為機器人可執行的關節角度 (Joint Angles)，解決 5-DOF 機械臂的奇異點與多解問題。

---

## 📂 檔案總覽

### 1. `robust_pino_kinematics.py` (核心函式庫)
- **功能**: 封裝 Pinocchio 運算邏輯，提供統一的 FK (正向運動學) 與 IK (逆向運動學) 介面。
- **邏輯**:
  - **FK**: 計算 6D Pose (Position + Quaternion)。
  - **IK**: 使用 **Damped Least Squares (DLS)**演算法。
    - 公式: $dq = J^T (J J^T + \lambda^2 I)^{-1} \cdot err$
    - 參數: Damping ($\lambda=1.0$), Step Limit (0.1 rad), Max Iterations (20)。
    - 特點: 針對 5-DOF 結構優化，犧牲部分旋轉自由度以確保位置精確度。

### 2. `convert_jt_to_ee.py` (數據預處理)
- **功能**: 將原始錄製的 Rosbag (含 `/joint_states`) 轉換為包含 EE Pose 的新 Rosbag。
- **邏輯**:
  - 讀取輸入 Bag 的 Joint States。
  - 使用 `robust_pino_kinematics`計算對應的 EE Pose。
  - **關鍵**: 確保訓練數據 (EE Pose) 與執行時使用的 IK 模型完全一致 (同一個 Pinocchio 模型)，消除模型誤差。
- **用法**:
  ```bash
  python3 scripts/end-effector/convert_jt_to_ee.py <input_bag> <output_bag>
  # 範例
  python3 scripts/end-effector/convert_jt_to_ee.py Dataset/bags/001/0.db3 Dataset/converted_001
  ```

### 3. `replay_ee_ik_robust.py` (執行與驗證)
- **功能**:讀取轉換後的 EE Pose Bag，實時解算 IK 並控制機器人 (或模擬驗證)。
- **邏輯**:
  - 訂閱/讀取 `/left/ee_pose`, `/right/ee_pose`。
  - 使用 `robust_pino_kinematics` 的 `inverse_kinematics_5dof` 解算目標關節角。
  - 發布 `/left_follower/joint_states_control` 控制機器人。
  - 包含 "Warm Start" 機制：使用上一幀的解作為下一幀的初始猜測，確保動作連續性。
- **用法**:
  ```bash
  python3 scripts/end-effector/replay_ee_ik_robust.py <ee_pose_bag>
  # 範例
  python3 scripts/end-effector/replay_ee_ik_robust.py Dataset/converted_001
  ```

### 4. `verify_consistency.py` (數據分析與驗證)
- **功能**: 量化分析 "FK -> IK" 的轉換誤差，驗證系統一致性。
- **邏輯**:
  - 讀取 Bag 中的 Joint States (Ground Truth)。
  - 計算 FK 得到 Pose。
  - 立即計算 IK 還原為 Joint Angles (Predicted)。
  - 比較 GT 與 Predicted 的誤差 (RMSE)。
- **用法**:
  ```bash
  python3 scripts/end-effector/verify_consistency.py <converted_bag>
  ```
- **輸出**:
  - `consistency_results.csv`: 詳細誤差報告。
  - 終端機顯示平均 RMSE (應 < 0.05 rad 為佳)。

---

## 🛠️ 標準工作流程 (Pipeline)


0. 

  ```
  source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && xacro ros2_ws/src/koch_simulation/urdf/low_cost_robot.xacro prefix:=left_ > /tmp/koch_left.urdf && xacro ros2_ws/src/koch_simulation/urdf/low_cost_robot.xacro prefix:=right_ > /tmp/koch_right.urdf
  ```
1.  **數據轉換**: 將原始數據轉為一致的 EE 格式。
    ```bash
    python3 scripts/end-effector/convert_jt_to_ee.py Dataset/bags/original Dataset/converted_ee
    ```

    ```
    python3 scripts/end-effector/convert_jt_to_ee.py Dataset/bags/0303_clearwater_yellowtable/0001 tmp/0303_clearwater_yellowtable/0001
    ```

2.  **一致性檢查**: 確認轉換後的數據在數學上是可逆的。
    ```bash
    python3 scripts/end-effector/verify_consistency.py Dataset/converted_ee
    ```

    ```bash
    python3 scripts/end-effector/verify_consistency.py tmp/0303_clearwater_yellowtable/0001
    ```

3.  **實機/模擬重播**: 執行動作還原。
    ```bash
    python3 scripts/end-effector/replay_ee_ik_robust.py Dataset/converted_ee
    ```

    ```bash
    python3 scripts/end-effector/replay_ee_ik_robust.py tmp/0303_clearwater_yellowtable/0001
    ```

## ⚠️ 常見問題
- **ImportError: pinocchio**: 請確認已安裝 `pin` 套件 (`pip install pin`) 且 PYTHONPATH 包含 `cmeel` 路徑。
- **URDF 錯誤**: 確保 `/tmp/koch_left.urdf` 和 `/tmp/koch_right.urdf` 存在 (由 `start_fusion_system.sh` 或相關 launch 檔生成)。
