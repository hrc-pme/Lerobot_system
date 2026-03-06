# Koch Robot LeRobot System

> **視覺-觸覺融合機器人學習系統**  
> Visual-Tactile Robot Learning System with Koch Arms

基於 LeRobot 框架的雙臂 Koch 機器人控制與感知系統，整合 RealSense 視覺與 WowSkin 觸覺感測器，支援遙操作、數據收集與 imitation learning。

---

## 📋 目錄

- [系統概覽](#系統概覽)
- [專案結構](#專案結構)
- [硬體需求](#硬體需求)
- [快速開始 (Docker)](#快速開始-docker-推薦)
- [系統啟動](#系統啟動)
- [數據收集與訓練](#數據收集與訓練)
- [系統架構](#系統架構)
- [詳細文檔](#詳細文檔)
- [常見問題](#常見問題)

---

## 系統概覽

### 核心功能

- ✅ **雙臂遙操作控制** - Leader-Follower 模式，支援左右手獨立或雙臂協同
- ✅ **多模態感知融合** - 2× RealSense D435i 視覺 + 5× WowSkin 觸覺感測器
- ✅ **即時 3D 可視化** - RViz2 整合，顯示機器人模型、點雲與觸覺熱圖
- ✅ **數據收集系統** - 標準化 rosbag 格式，支援 LeRobot 訓練
- ✅ **MoveIt 路徑規劃** - 末端點控制與軌跡規劃
- ✅ **觸覺熱圖渲染** - Multi-Gaussian Splatting (MGS) 64×64 RGB 圖像

### 技術棧

- **ROS2**: Humble (Ubuntu 22.04)
- **LeRobot**: v0.4.3 (Hugging Face Robotics Framework)
- **VT-Refine**: CoRL 2025 (Visuo-Tactile Learning)
- **Python**: 3.10
- **硬體**: Dynamixel XL330-M288 馬達
- **容器化**: Docker (GPU/CPU 支援)

---

## 📁 專案結構

本儲存庫整合了多個子系統，主要目錄結構如下：

- **`docker/`**: 容器化環境建置腳本 (CPU/GPU/Jetson)。
- **`ros2_ws/`**: ROS2 工作空間，包含核心控制與感知套件。
  - `koch_control`: 機器人底層控制。
  - `koch_tactile_ros2`: 觸覺與視覺融合感知系統。
  - `realsense_ros`: RealSense 相機驅動。
  - `robopoint`: 點雲處理相關工具。
- **`repos/`**: 外部依賴與核心演算法庫。
  - **`koch_tactile/`**: WowSkin 觸覺感測器的原始資料處理、MGS 渲染與資料集工具。
  - **`lerobot/`**: Hugging Face LeRobot 機器人學習框架核心。
  - **`vt-refine/`**: 視覺-觸覺強化學習與模擬環境相關模組。
- **`entrypoint/`**: 系統啟動快捷腳本 (Camera, Teleop 等)。
- **`Dataset/`**: 存放收集的 Rosbag 與處理後的數據集。
- **`calibration/`**: 存放 Koch 手臂的校正參數檔。

---

## 硬體需求

### 機械手臂
- 2× Koch 低成本機器人手臂 (Leader + Follower)
- 可選：額外一對左手臂（雙臂模式）

### 感測器
- **視覺**: 2× Intel RealSense D435i 深度相機
  - camera_top: 俯視 (0, 0.16, 0.5)
  - camera_first: 側視 (0.04, -0.18, 0.22)
- **觸覺**: 5× WowSkin 磁力觸覺感測器（安裝於右手 Gripper）

### 電腦配置
- CPU: Intel i7 或以上
- RAM: 16GB+
- USB 端口: 4+ (馬達 + 感測器 + 相機)

---

## 快速開始 (Docker 推薦)

本系統推薦使用 Docker 環境進行部署，以確保環境一致性。

### 1. 建立 Docker 映像檔

根據您的硬體配置選擇 CPU 或 GPU 版本 (推薦 GPU)：

**GPU 版本 (推薦):**
```bash
cd docker/GPU
./build.sh
```

**CPU 版本:**
```bash
cd docker/CPU
./build.sh
### 2. 啟動容器

建立完成後，執行以下指令啟動容器：

```bash
# 確保位在 docker/GPU 或 docker/CPU 目錄下
./run.sh
```

容器啟動後，您將會進入 `/home/hrc/Lerobot_system` 工作目錄。

---

## 系統啟動

容器內支援兩種主要工作模式：

### 模式 1: 機器人控制與數據收集

用於遙操作機器人並錄製訓練數據。建議使用 `terminator` 或 `tmux` 開啟多個終端視窗：

#### 核心系統 (必須按順序啟動)

**Terminal 1: ROS Bridge** (Web 介面通訊)
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 2: 相機系統**
```bash
cd entrypoint && ./Camera.sh
```

**Terminal 3: TF 座標廣播**
```bash
ros2 launch koch_tactile_ros2 real_robot_tf.launch.py gripper_prefix:=right_
```

**Terminal 4: 觸覺感測 (WowSkin)**
```bash
ros2 launch koch_tactile_ros2 wowskin_complete.launch.py
```

**Terminal 5: 視覺-觸覺融合**
```bash
ros2 launch koch_tactile_ros2 vt_system.launch.py gripper_side:=right
```

#### 操作與可視化 (可選)

**Terminal 6: 機器人遙操作**
```bash
cd entrypoint && ./leader_follower-teleop.sh
# 選擇: 1=右手 / 2=左手 / 3=雙手
```

**Terminal 7: RViz 3D 可視化**
```bash
rviz2 -d ros2_ws/src/koch_tactile_ros2/config/fusion_visualization.rviz
```

#### 數據錄製 (需要時開啟)

**Terminal 8: Rosbag 錄製**
```bash
ros2 run bag_recorder recorder -c ros2_ws/src/bag_recorder/config/bag_recorder.yaml
# 或使用 GUI: ros2 run bag_recorder recorder_ui -c <config>
```

停止錄製後，資料會自動儲存至 `Dataset/bags/0001/`, `0002/` ...

> 📖 **詳細說明**: [BAG_RECORDER_GUIDE.md](docs/BAG_RECORDER_GUIDE.md)

### 模式 2: 模型訓練與推論

使用收集的數據進行 imitation learning 訓練。

```bash
# 1. 處理 rosbag 資料
cd repos/koch_tactile/Dataset_processor/Integrate
python3 pipeline.py --config config.yaml

# 2. 轉換至 LeRobot 格式
cd ~/Lerobot_system/repos/lerobot
python -m lerobot.common.datasets.push_dataset_to_hub \
    --raw-dir ~/Lerobot_system/Dataset/processed \
    --repo-id your-username/koch-tactile-dataset

# 3. 訓練策略 (ACT)
python lerobot/scripts/train.py \
    policy=act \
    env=koch \
    dataset_repo_id=your-username/koch-tactile-dataset
```

> 📖 **詳細說明**: 
> - [DATA_PROCESSING_GUIDE.md](docs/DATA_PROCESSING_GUIDE.md) - 數據處理流程
> - [LEROBOT_INTEGRATION.md](docs/LEROBOT_INTEGRATION.md) - LeRobot 訓練
> - [VT_REFINE_GUIDE.md](docs/VT_REFINE_GUIDE.md) - VT-Refine 觸覺學習

---

## 數據收集與訓練

### 完整工作流程

```
1. 遙操作錄製 → 2. 解析處理 → 3. 格式轉換 → 4. 模型訓練 → 5. 部署推論
```

#### 1. 錄製示範數據

啟動系統後（參考上方「模式 1」），執行遙操作並錄製：

```bash
# Terminal 8
ros2 run bag_recorder recorder -c ros2_ws/src/bag_recorder/config/bag_recorder.yaml
```

完成後資料夾結構：
```
Dataset/bags/
├── 0001/  # 第一次錄製
├── 0002/  # 第二次錄製
└── ...
```

#### 2. 處理與標註

使用 Koch Tactile 工具鏈自動化處理：

```bash
cd repos/koch_tactile/Dataset_processor/Integrate
python3 pipeline.py --config config.yaml
```

輸出：抽取的影像、力值 CSV、分割的 episodes。

#### 3. 轉換為訓練格式

**選項 A: LeRobot 格式（推薦用於 ACT/Diffusion Policy）**
```bash
cd repos/lerobot
python -m lerobot.common.datasets.push_dataset_to_hub \
    --raw-dir ~/Lerobot_system/Dataset/processed \
    --repo-id your-username/koch-tactile
```

**選項 B: VT-Refine 格式（用於觸覺強化學習）**
```bash
cd repos/koch_tactile/Train
python3 convert_to_vtrefine.py \
    --input ~/Lerobot_system/Dataset/processed \
    --output ~/Lerobot_system/Dataset/vt_refine
```

#### 4. 訓練模型

**LeRobot ACT:**
```bash
cd repos/lerobot
python lerobot/scripts/train.py \
    policy=act \
    env=koch \
    dataset_repo_id=your-username/koch-tactile
```

**VT-Refine (觸覺融合):**
```bash
cd repos/vt-refine
python3 dppo/script/run.py \
    --config-name=koch_tactile \
    --config-path=../cfg/koch/pretrain
```

#### 5. 部署推論

載入訓練好的模型並在真實機器人上執行：

```bash
# 啟動推論節點
ros2 run vt_refine_inference vt_refine_node

# 或使用 LeRobot 策略
python scripts/deploy_policy.py --checkpoint outputs/checkpoint_best.pth
```

---

## 本地安裝 (非 Docker)

若不使用 Docker，請參考以下步驟：

### 安裝

```bash
# 1. Clone repository
cd ~/
git clone <repository_url> Lerobot_system
cd Lerobot_system

# 2. 安裝 ROS2 Humble (若未安裝)
# 參考: https://docs.ros.org/en/humble/Installation.html

# 3. 安裝 Python 依賴
pip install lerobot pyrealsense2 opencv-python

# 4. 編譯 workspace
cd ~/Lerobot_system
colcon build --symlink-install
source install/setup.bash
```

### 系統啟動

請參考上方 "系統啟動流程"，但在執行前需確保每個終端機都已執行 `source install/setup.bash`。

### RViz 顯示設定

啟動 RViz 後，確認以下顯示項目：

1. **TF 樹** - 顯示所有座標系關係
2. **機器人模型** - 左右手臂 3D 模型
3. **視覺點雲** - `/visual_points_pc2` (白色，1024 點)
4. **觸覺點雲** - `/tactile_points_pc2` (紅色，5 點)
5. **觸覺熱圖** - `/tactile_mesh_marker` (64×64 彩色平面)

**重要設定**:
- Fixed Frame: `world`
- 新增 RobotModel: Description Topic = `/left/robot_description` 和 `/right/robot_description`
- 觸覺熱圖 Marker 自動跟隨 gripper 移動

---

## 系統架構

### 數據流程圖

```
WowSkin 感測器 (5 個)
    ↓
[wowskin_feature_node] → /wowskin/contact_features_v2 (5×5 features)
    ↓
[wowskin_mgs_renderer] → /wowskin/mgs_rgb_true_v2 (64×64 RGB 圖像)
    ↓
[tactile_mesh_visualizer] → /tactile_mesh_marker (3D Mesh)
[tactile_image_to_pointcloud2] → /tactile_image_pc2 (4096 colored points)

RealSense 相機 (2 台)
    ↓
[multi_camera_visual_processor] → /visual_points (1024×3)
    ↓
[vt_points_to_pointcloud2] → /visual_points_pc2 (PointCloud2)

觸覺特徵
    ↓
[tactile_points_generator] → /tactile_points (5×4 或 5×6)
    ↓
[vt_points_to_pointcloud2] → /tactile_points_pc2 (PointCloud2)
```

### TF 座標樹

```
map
 └─ world
     ├─ camera_top_link → camera_top_depth_optical_frame
     ├─ camera_first_link → camera_first_depth_optical_frame
     ├─ left_base_link → left_link1 → ... → left_gripper_static_1
     └─ right_base_link → right_link1 → ... → right_gripper_static_1
                                               ↓
                                        right_tactile_base_link
                                               ↓
                                        right_wowskin_sensor_0~4
```

### 關鍵 Topics

#### 觸覺相關
- `/wowskin/contact_features_v2` - 原始特徵 (5×5)
- `/wowskin/mgs_rgb_true_v2` - MGS 圖像 (固定 scale)
- `/tactile_points` - 觸覺 3D 點 (5×4)
- `/tactile_points_pc2` - PointCloud2 格式
- `/tactile_mesh_marker` - 3D Mesh 可視化
- `/tactile_image_pc2` - 64×64 彩色點雲

#### 視覺相關
- `/visual_points` - 視覺 3D 點 (1024×3)
- `/visual_points_pc2` - PointCloud2 格式
- `/camera/camera_top/aligned_depth_to_color/image_raw`
- `/camera/camera_first/aligned_depth_to_color/image_raw`

#### 機器人控制
- `/left_leader/joint_states` - 左手 Leader 狀態
- `/right_follower/joint_states_control` - 右手 Follower 控制

---

## 詳細文檔

本儲存庫提供完整的技術文檔，涵蓋系統各個層面：

### 📦 核心系統文檔

位於 `docs/` 目錄：

#### 數據收集與處理
- **[BAG_RECORDER_GUIDE.md](docs/BAG_RECORDER_GUIDE.md)** - Rosbag 錄製工具完整使用指南
  - CLI/GUI 兩種模式
  - YAML 配置詳解
  - 自動編號與壓縮設定
  
- **[DATA_PROCESSING_GUIDE.md](docs/DATA_PROCESSING_GUIDE.md)** - 數據後處理流程
  - Rosbag 解析與抽取
  - 觸覺事件標註
  - HDF5/LeRobot 格式轉換

#### 機器學習框架
- **[LEROBOT_INTEGRATION.md](docs/LEROBOT_INTEGRATION.md)** - LeRobot 框架整合
  - 資料集準備與上傳
  - ACT/Diffusion Policy 訓練
  - 模型部署與推論
  
- **[VT_REFINE_GUIDE.md](docs/VT_REFINE_GUIDE.md)** - VT-Refine 觸覺學習
  - 模擬環境配置
  - 預訓練與微調
  - Sim-to-Real 轉移

#### 機器人控制
- **[KOCH_ROBOT_MANUAL.md](docs/KOCH_ROBOT_MANUAL.md)** - Koch 機器人控制系統
  - Dynamixel 馬達配置
  - Leader-Follower 遙操作
  - 安全機制與故障排除
  
- **[KOCH_ROBOT_VISUALIZATION.md](docs/KOCH_ROBOT_VISUALIZATION.md)** - RViz 可視化設定
  - TF 樹配置
  - 點雲與 Mesh 顯示
  - 自訂 RViz 配置

- **[KOCH_MOVEIT_MANUAL.md](docs/KOCH_MOVEIT_MANUAL.md)** - MoveIt 路徑規劃
  - 運動學配置
  - 末端點控制
  - 碰撞檢測

### 🤖 觸覺系統文檔

位於 `ros2_ws/src/koch_tactile_ros2/docs/`：

- **[CODE_ARCHITECTURE.md](ros2_ws/src/koch_tactile_ros2/docs/CODE_ARCHITECTURE.md)** - ROS2 套件完整架構
  - 節點功能說明
  - Topic 與 TF 關係圖
  - Launch 檔組織

- **[MGS_GUIDE.md](ros2_ws/src/koch_tactile_ros2/docs/MGS_GUIDE.md)** - MGS 渲染器使用指南
  - Multi-Gaussian Splatting 原理
  - 實時與訓練兩種模式
  - 參數調校與視覺化

### 🧪 觸覺資料處理工具

位於 `repos/koch_tactile/`：

- **[repos/koch_tactile/README.md](repos/koch_tactile/README.md)** - WowSkin 完整工作流程
  - 感測器驅動與特徵抽取
  - 資料收集與標註工具
  - 訓練與即時推論系統

- **[repos/koch_tactile/sensor2/README.md](repos/koch_tactile/sensor2/README.md)** - 感測器底層技術文檔
  - 原始資料處理
  - MGS 渲染實現細節

- **[repos/koch_tactile/Train/README.md](repos/koch_tactile/Train/README.md)** - 觸覺模型訓練
  - 神經網路架構
  - 訓練配置與超參數

### 🌐 外部框架文檔

#### LeRobot (Hugging Face)
- **官方文檔**: https://huggingface.co/docs/lerobot
- **本地**: `repos/lerobot/README.md`
- **重點**:
  - 支援的機器人列表
  - 資料集格式標準
  - 預訓練模型庫

#### VT-Refine (NVIDIA)
- **論文**: https://arxiv.org/abs/2510.14930
- **專案網站**: https://binghao-huang.github.io/vt_refine/
- **本地**: `repos/vt-refine/README.md`
- **重點**:
  - 模擬環境設定 (Isaac Sim)
  - DPPO 訓練流程
  - 領域隨機化技巧

---

## 常見問題

### Q1: RViz 中機器人模型散架或位置錯誤
**原因**: TF 系統未正確啟動或 robot_state_publisher 缺失

**解決**:
```bash
# 檢查 TF 是否正常
ros2 run tf2_ros tf2_echo world right_base_link

# 重啟 TF 系統
ros2 launch koch_tactile_ros2 real_robot_tf.launch.py gripper_prefix:=right_
```

### Q2: 相機點雲顯示桌面或低處雜訊
**原因**: 空間邊界設定包含地面

**解決**: 編輯 `config/cameras.yaml`
```yaml
camera_first:
  bounds: [-0.6, 0.6, -0.6, 0.6, 0.15, 1.0]  # z_min 提高到 0.15m
```

### Q3: 觸覺熱圖沒有顏色
**原因**: RGB PointCloud2 格式錯誤

**解決**: 確認使用 `tactile_image_to_pointcloud2` 節點，RGB 欄位為 UINT32

### Q4: 觸覺平面位置不正確
**原因**: TF 轉換參數需要調整

**解決**: 編輯 `real_robot_tf.launch.py`
```python
tactile_base_tf_right = Node(
    arguments=['0.0', '0.0', '0.03', '1.57', '0', '0',  # [x, y, z, roll, pitch, yaw]
               'right_gripper_static_1', 'right_tactile_base_link']
)
```

### Q5: WowSkin 感測器無數據
**原因**: USB 權限或端口錯誤

**解決**:
```bash
# 檢查設備
ls /dev/ttyACM*

# 修改權限
sudo chmod 666 /dev/ttyACM0

# 檢查 topic
ros2 topic echo /wowskin/contact_features_v2 --once
```

### Q6: 按 'r' 鍵無法重置 MGS running max
**原因**: 通過 launch 啟動時終端非互動模式

**解決**: 直接執行節點
```bash
ros2 run koch_tactile_ros2 wowskin_mgs_renderer
# 然後按 'r'
```

---

## 數據收集

### 錄製 Rosbag

啟動系統後（參考上方「系統啟動」），執行錄製：

```bash
ros2 run bag_recorder recorder -c ros2_ws/src/bag_recorder/config/bag_recorder.yaml
```

資料會自動儲存至 `Dataset/bags/0001/`, `0002/` ...

> 📖 **完整指南**: [BAG_RECORDER_GUIDE.md](docs/BAG_RECORDER_GUIDE.md)

### 回放驗證

```bash
cd ~/Lerobot_system/Dataset/bags/0001
ros2 bag info 0001_0.db3
ros2 bag play 0001_0.db3
```

---

## 維護與開發

### 重新編譯

```bash
cd ~/Lerobot_system
colcon build --packages-select koch_tactile_ros2 --symlink-install
source install/setup.bash
```

### 清理節點

```bash
# 停止所有觸覺相關節點
pkill -f "tactile_points_generator"
pkill -f "multi_camera_visual_processor"
pkill -f "tactile_mesh_visualizer"
pkill -f "vt_points_to_pointcloud2"
pkill -f "tactile_image_to_pointcloud2"
```

### 查看系統狀態

```bash
# 檢查節點
ros2 node list

# 檢查 topics
ros2 topic list

# 檢查 TF 樹
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## 貢獻者

- **開發**: HRC Lab
- **框架**: LeRobot (Hugging Face)
- **硬體**: Koch Low-Cost Robot
- **感測器**: WowSkin Tactile Sensor

---

## 授權

Apache 2.0 License

---

## 更新日誌

### v2.0 (2026-01-29)
- ✅ 新增 `tactile_image_to_pointcloud2` - 64×64 彩色觸覺點雲
- ✅ 修正 RGB PointCloud2 格式 (UINT32)
- ✅ 整合 `vt_system.launch.py` 統一啟動
- ✅ 優化相機空間邊界過濾 (z_min=0.15)
- ✅ 更新 TF 系統支援機器人模型

### v1.0 (2026-01-20)
- 初始版本：5-sensor 觸覺系統 + 雙相機視覺
- MGS 渲染器 + 觸覺 Mesh 可視化




python3 /home/hrc/Lerobot_system/repos/lerobot/src/lerobot/scripts/lerobot_dataset_viz.py \
  --repo-id koch_bi_wipe_water \
  --root /home/hrc/Lerobot_system/Dataset/converted_dataset3 \
  --episode-index 0 \
  --display-compressed-images 1 \
  --save 1 \
  --output-dir /home/hrc/Lerobot_system/viz_output

python3 /home/lesterliou02/lerobot/src/lerobot/scripts/lerobot_train.py \
  --dataset.repo_id koch_bi_wipe_water \
  --dataset.root /home/lesterliou02/lerobot/Koch_IL/converted_dataset3 \
  --policy.type act \
  --policy.repo_id lesterliou02/koch_bi_wipe_water_policy \
  --output_dir /home/lesterliou02/lerobot/output \
  --wandb.enable true \
  --wandb.project lerobot_koch_test

python3 /home/hrc/Lerobot_system/repos/lerobot/src/lerobot/scripts/lerobot_train.py \
  --dataset.repo_id koch_bi_wipe_water \
  --dataset.root /home/hrc/Lerobot_system/Dataset/converted_dataset3 \
  --policy.type act \
  --policy.repo_id lesterliou02/koch_bi_wipe_water_policy \
  --output_dir /home/hrc/Lerobot_system/output2 \
  --wandb.enable true \
  --wandb.project lerobot_koch_test