# Cross-Robot EE Bridge (Koch <-> OpenArm)

這包工具的核心目的是為了解決「跨機器人操作 (Cross-Robot)」以及「末端夾爪模型 (End-Effector Model) 訓練不穩定」的問題。

## 為什麼需要這包工具？

原本如果直接讓 AI Policy 輸出末端夾爪的六維座標 (XYZ + 旋轉)，在訓練常常會發散抓不到規律。
但如果是使用「Joint State (關節角度)」下去訓練，模型卻很好收斂。

所以我們的策略是：
1. **內部訓練與推理 (Policy 端)**：統一使用「虛擬 Koch 機器人」的 Joint State。
2. **外部實體執行 (真機端)**：透過這包 Bridge，把 Koch 的 Joint State 轉成世界座標 (End-Effector Pose)，再投影到不同構型的機器人 (例如 OpenArm) 身上，用它的 Inverse Kinematics (IK) 去追蹤同一點。

這樣一來，不論你換成什麼款式的機器手臂，只要在共用工作區域內，AI 認為它在操控 Koch，但實際的 OpenArm 也會跑到同樣的絕對空間位置。夾爪也會等比例映射！

---

## 檔案架構與工作邏輯

### 1. `joint_to_ee_converter.py`
- **角色**：Koch 的替身。
- **去程 (Action)**：監聽 AI Policy 發出的 Koch Joint 控制訊號，用正向運動學 (Forward Kinematics, FK) 算出夾爪在空間中的 `Pose`，並發佈出去。
- **回程 (Observation)**：接收實體 OpenArm 當下的真實 `Pose`，用逆向運動學 (IK) 反推出如果這是 Koch，它的 Joint 角度會是多少，然後發佈回給 Policy 當成下一禎的 Observation。

### 2. `openarm_ee_bridge.py`
- **角色**：OpenArm 的大腦。
- **去程 (Action)**：接收到轉換器發出的 `Pose` 後，自動加上兩隻手臂的空間平移 (Y軸 $\pm$ 0.2m) 與旋轉 ($Z$ 軸 $-90^{\circ}$)，藉此**對齊雙方的正前方與絕對工作空間**。接著跑 Pinocchio IK，算出 OpenArm 全身 18 軸所需的目標角度，發送給底層。
- **末端姿態校正 (End-Effector Local Correction)**：Koch 與 OpenArm 由於夾爪構造方向不同 (Koch 預設水平、OpenArm 預設垂直)，此橋接器內建末端旋轉補償 (例如 $Z$ 軸旋轉 $90^{\circ}$)，確保在真實抓取時夾爪維持一致的相對角度。
- **針對樞紐點對齊 (Tip Link Pivot Point)**：在此橋接器中，設定 `tip_link="openarm_left_hand"` 與 Koch 的 `left_gripper_static_1` 手腕關節對齊，而非使用指尖 (`openarm_left_hand_tcp`)。因為保證手腕的中心點與旋轉角度重合，長度相仿的指尖在空間中才能以同個弧線完美疊合。
- **夾爪控制**：將 Koch 輸出的 $0 \sim 1$ 夾爪連續值，自動等比例轉換成 OpenArm 的 $0.0 \sim 0.044$ 公尺開合距離。

### 3. `openarm_real_hardware_bridge.py`
- **角色**：連接真實世界的實體手臂控制器。
- **作動原理**：它會接手由 `openarm_ee_bridge.py` 算完、原本要在 RViz 模擬中顯示的 `/openarm/joint_commands`，將其以 $50$ Hz 的頻率進行平滑處理 (alpha 濾波)，再分別封裝成 `Float64MultiArray`，推播給真實雙臂和真實夾爪的 Position Forward Controller。
- **安全機制**：內建速度限制與斷線超時保護（超過 2 秒未收到指令即停機）；按下 `Ctrl+C` 中斷時，會強制鎖定當下手臂位置取代無預警斷電，避免實體手臂暴走。

### 4. `mock_cross_robot.py`
- **角色**：發號施令與虛構硬體 (測試用)。
- 產生平滑的 Sine 波段 Joint 與 Gripper 訊號，模擬 LeRobot Policy 正在持續輸出動作，用來驗證整個轉譯管線是否流暢。

### 5. `replay_dataset_cross_robot.py` (新增功能)
- **角色**：資料集回放與驗證工具。
- **作動原理**：負責將錄製好的 Koch 真機 Dataset (Rosbag) 自動依序播放。不僅能夠重現錄製時的角度，還內建 **硬體特徵反轉與偏移量對齊 (Hardware mapping)**。
- **Topic 攔截與重導向機制**：
  由於錄製到的原始數據包含硬體機構的正負反轉，此腳本會將 `ros2 bag play` 的原始指令重新導向隱藏的 `/raw_joint_states` topic。在 Python 內經過精確校正重組後，才兵分兩路發送：一條送給 RViz 更新純淨模型，另一條送給 `joint_to_ee_converter` 去作正確的跨機器人 Forward Kinematics 解算，徹底解決了直接拿裸數據去跑 FK 所造成的顛倒與「鏡像錯位」問題。

### 6. `launch_rviz_everything.sh`
- **角色**：視覺化驗證工具。
- 把 Koch 跟 OpenArm 的 URDF 一起載入進 RViz。利用 `tf2_ros static_transform_publisher` 把兩個不同基座的機器人「重疊」在同一個桌面上，讓你直觀看見末端夾爪是否有在空中相遇。

---

## 如何執行測試？

請開啟三個不同的終端機 (Terminal)，並都在 `~/Lerobot_system/` 目錄下執行。

### Terminal 1：啟動 RViz 視覺化與 TF
```bash
./scripts/end-effector/cross_openarm/launch_rviz_everything.sh
```

### Terminal 2：啟動座標橋接計算節點
```bash
# 啟動虛擬替身與解算器
python3 scripts/end-effector/cross_openarm/joint_to_ee_converter.py &
python3 scripts/end-effector/cross_openarm/openarm_ee_bridge.py
```

### Terminal 3：啟動模擬控制器 (發送波形) 或 回放真實資料集 或 即時收發
如果你想測試自定義的弦波連貫動作：
```bash
python3 scripts/end-effector/cross_openarm/mock_cross_robot.py
```
如果你想測試錄製好的真實 Koch Bag (並將它完美轉移重現到 OpenArm 上)：
```bash
python3 scripts/end-effector/cross_openarm/replay_dataset_cross_robot.py /path/to/your/bag
```
如果你正在操作**真實的 Koch 主從手臂 (Live Teleop)**，想讓 OpenArm 同步即時跟隨你的揮舞動作，請執行：
```bash
python3 scripts/end-effector/cross_openarm/live_teleop_cross_robot.py
```

### Terminal 4 (非必選)：橋接至真實手臂硬體
如果你的網路內已經透過 launch 檔啟動了實體 OpenArm 以及對應的 `position_controller`，則可以加上負責將虛擬指令傳給真實硬體的驅動橋接器：
```bash
./scripts/end-effector/cross_openarm/openarm_real_hardware_bridge.py
```

執行後，看著 RViz 的畫面，你就會發現：雖然兩者的基座完全長得不一樣 (一個由低往上、一個由高往下)，但因為空間座標的對齊，在中間共用工作區域時，OpenArm 的兩隻夾爪會精準地咬合在 Koch 虛擬夾爪的位置上連續揮舞！並且如果有連實體硬體，其也能做到平滑的零延遲跟隨！


ros2 daemon stop

ros2 daemon start