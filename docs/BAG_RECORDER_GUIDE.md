# Bag Recorder 使用指南

`bag_recorder` 是一個 ROS2 套件，提供基於 YAML 配置的 rosbag2 錄製工具，支援 CLI 與 GUI 兩種模式。

---

## 📋 功能特點

- ✅ **YAML 配置驅動** - 預先定義 topics、壓縮格式、儲存路徑等參數
- ✅ **自動遞增編號** - 輸出資料夾自動命名為 `0001`, `0002`, `0003`...
- ✅ **GUI 介面** - 可視化錄製控制面板（基於 Tkinter）
- ✅ **Launch 整合** - 支援 ROS2 launch 檔啟動
- ✅ **靈活 Topic 選擇** - 支援指定 topics 或全錄製模式 + 排除規則
- ✅ **壓縮與分割** - 支援 zstd 壓縮與檔案大小/時間自動分割

---

## 📦 安裝

### 1. 編譯套件

確保您位於 ROS2 工作空間根目錄：

```bash
cd ~/Lerobot_system
colcon build --packages-select bag_recorder --symlink-install
source install/setup.bash
```

### 2. 驗證安裝

```bash
ros2 pkg list | grep bag_recorder
```

應顯示 `bag_recorder`。

---

## 🚀 使用方式

### 方法 1: CLI 直接執行

使用預設配置檔：

```bash
ros2 run bag_recorder recorder -c ~/Lerobot_system/ros2_ws/src/bag_recorder/config/bag_recorder.yaml
```

使用自訂配置檔：

```bash
ros2 run bag_recorder recorder -c /path/to/your/config.yaml
```

**停止錄製**: 按 `Ctrl+C`

---

### 方法 2: Launch 檔啟動

使用預設配置：

```bash
ros2 launch bag_recorder record.launch.py
```

使用自訂配置：

```bash
ros2 launch bag_recorder record.launch.py config_file:=/path/to/custom.yaml
```

---

### 方法 3: GUI 模式（推薦新手）

啟動圖形化介面：

```bash
ros2 run bag_recorder recorder_ui -c ~/Lerobot_system/ros2_ws/src/bag_recorder/config/bag_recorder.yaml
```

或使用 launch 檔：

```bash
ros2 launch bag_recorder record_with_ui.launch.py
```

**GUI 功能**:
- **Start Recording** - 開始錄製
- **Stop** - 停止錄製
- 顯示錄製時間與狀態

---

## ⚙️ 配置檔說明

配置檔位置：`ros2_ws/src/bag_recorder/config/bag_recorder.yaml`

### 完整範例

```yaml
# ===== Topic 選擇 =====
allow_all: false  # true=錄製所有 topics，false=只錄製指定 topics

topics:
  - /tf_static
  - /camera/camera_top/color/image_raw/compressed
  - /camera/camera_first/color/image_raw/compressed
  - /left_follower/joint_states
  - /right_follower/joint_states
  - /wowskin/contact_features_v2
  - /wowskin/mgs_rgb_true_v2

# 排除 topics（只在 allow_all: true 時有效）
exclude_topics:
  - ^/rosout.*$
  - ^/parameter_events$

# ===== 儲存設定 =====
output_dir: /home/hrc/Lerobot_system/Dataset/bags  # 輸出根目錄
storage: sqlite3           # sqlite3 或 mcap
compression: zstd          # none 或 zstd
compression_mode: file     # file 或 message

# ===== 檔案分割 =====
max_bag_size: 1073741824   # 1 GB (bytes)，超過後自動分割
max_bag_duration: 600      # 600 秒 (10 分鐘)，超過後自動分割

# ===== QoS 設定 =====
qos_profile_overrides_path: ""  # QoS 覆蓋配置檔路徑（選填）
```

### 參數說明

#### Topic 選擇
- **`allow_all`**: 
  - `false` - 只錄製 `topics` 列表中的項目（推薦）
  - `true` - 錄製所有 topics，可用 `exclude_topics` 排除

- **`topics`**: 字串列表，指定要錄製的 topic 名稱

- **`exclude_topics`**: 正則表達式列表，排除符合的 topics（只在 `allow_all: true` 時生效）

#### 儲存格式
- **`storage`**: 
  - `sqlite3` - 預設格式
  - `mcap` - 需安裝 `ros-humble-rosbag2-storage-mcap`

- **`compression`**: 
  - `none` - 不壓縮（檔案較大）
  - `zstd` - 使用 zstd 壓縮（推薦）

- **`compression_mode`**: 
  - `file` - 壓縮整個檔案（推薦）
  - `message` - 逐訊息壓縮

#### 自動分割
- **`max_bag_size`**: 檔案大小上限（bytes），超過後自動建立新檔案
  - `0` - 不限制
  - `1073741824` - 1 GB

- **`max_bag_duration`**: 錄製時間上限（秒），超過後自動建立新檔案
  - `0` - 不限制
  - `600` - 10 分鐘

#### 輸出目錄
- **`output_dir`**: 錄製檔案的根目錄，資料夾會自動建立為 `0001`, `0002`...

---

## 📂 輸出結構

執行 3 次錄製後的資料夾結構：

```
Dataset/bags/
├── 0001/
│   ├── metadata.yaml
│   └── 0001_0.db3
├── 0002/
│   ├── metadata.yaml
│   └── 0002_0.db3
└── 0003/
    ├── metadata.yaml
    └── 0003_0.db3
```

每次錄製自動產生遞增編號資料夾。

---

## 🔧 進階使用

### 1. 建立專案專用配置

複製範本並修改：

```bash
cp ~/Lerobot_system/ros2_ws/src/bag_recorder/config/bag_recorder.yaml \
   ~/Lerobot_system/my_custom_config.yaml
```

編輯 `my_custom_config.yaml` 調整 topics 與參數，然後執行：

```bash
ros2 run bag_recorder recorder -c ~/Lerobot_system/my_custom_config.yaml
```

### 2. 錄製高頻觸覺數據

針對 WowSkin 觸覺感測器（100 Hz）配置：

```yaml
topics:
  - /wowskin/contact_features_v2
  - /wowskin/mgs_rgb_true_v2
  - /right_follower/joint_states

compression: zstd          # 強烈建議壓縮
max_bag_duration: 300      # 每 5 分鐘分割檔案
```

### 3. 與機器人控制整合

在啟動機器人系統後，新開一個終端執行錄製：

```bash
# Terminal 1~5: 按照標準流程啟動系統
# (ROS Bridge, Camera, TF, WowSkin, VT System)

# Terminal 6: 啟動機器人遙操作
cd ~/Lerobot_system/entrypoint
./leader_follower-teleop.sh

# Terminal 7: 開始錄製
ros2 run bag_recorder recorder -c ~/Lerobot_system/ros2_ws/src/bag_recorder/config/bag_recorder.yaml
```

### 4. 回放驗證

錄製完成後，驗證資料完整性：

```bash
cd ~/Lerobot_system/Dataset/bags/0001
ros2 bag info 0001_0.db3

# 回放
ros2 bag play 0001_0.db3
```

在 RViz 中觀察數據是否正確。

---

## 🐛 常見問題

### Q1: 錄製失敗，提示 "No such file or directory"

**原因**: 輸出目錄不存在或權限不足

**解決**:
```bash
mkdir -p ~/Lerobot_system/Dataset/bags
chmod -R 755 ~/Lerobot_system/Dataset
```

### Q2: GUI 無法啟動，提示 "Tkinter not found"

**原因**: 缺少 Python Tkinter 套件

**解決**:
```bash
sudo apt install python3-tk
```

### Q3: 壓縮失敗，提示 "zstd plugin not found"

**原因**: 缺少 zstd 壓縮插件

**解決**:
```bash
sudo apt install ros-humble-rosbag2-compression-zstd
```

### Q4: 錄製檔案過大

**解決方案**:
1. 啟用壓縮: `compression: zstd`
2. 只錄製必要 topics，移除不需要的原始影像
3. 使用壓縮版影像: `/camera/*/image_raw/compressed`
4. 啟用自動分割: `max_bag_size: 1073741824`

### Q5: 如何修改自動編號起始值？

手動建立資料夾：

```bash
mkdir -p ~/Lerobot_system/Dataset/bags/0100
```

下次錄製會從 `0101` 開始。

---

## 📚 相關文檔

- [KOCH_ROBOT_MANUAL.md](KOCH_ROBOT_MANUAL.md) - 機器人控制系統
- [DATA_PROCESSING_GUIDE.md](DATA_PROCESSING_GUIDE.md) - 數據後處理流程
- [ROS2 Bag Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

---

## 🔗 程式碼參考

- **CLI 核心**: `ros2_ws/src/bag_recorder/bag_recorder/recorder.py`
- **GUI 介面**: `ros2_ws/src/bag_recorder/bag_recorder/recorder_ui.py`
- **Launch 檔**: `ros2_ws/src/bag_recorder/launch/record.launch.py`
- **預設配置**: `ros2_ws/src/bag_recorder/config/bag_recorder.yaml`
