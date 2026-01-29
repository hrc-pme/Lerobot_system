# 數據處理流程指南

本指南說明如何將錄製的 ROS2 bag 資料處理成可用於訓練的資料集格式。

---

## 📋 處理流程概覽

```
ROS2 Bag (錄製) 
    ↓
[1. 解析與抽取] → 影像 + 力值 CSV
    ↓
[2. 標註分割] → 標註接觸事件
    ↓
[3. 切割數據] → 按 segment 分割
    ↓
[4. 格式轉換] → HDF5 / LeRobot 格式
    ↓
模型訓練
```

---

## 🛠️ 處理工具

### 1. Koch Tactile Dataset Processor

位置：`repos/koch_tactile/Dataset_processor/`

#### 主要模組

**Integrate 整合處理流程（推薦）**
- `pipeline.py` - 自動化處理主程式
- `config.yaml` - 批次處理配置檔
- `modules/` - 抽取、標註、分割、轉換模組

**單步驟工具**
- `1_parse_rosbag.py` - 解析 rosbag
- `2_label_offscreen.py` - 離線標註工具
- `3_split_by_segments.py` - 資料切割

---

## 🚀 快速開始

### 方法 1: 自動化流程（推薦）

```bash
cd ~/Lerobot_system/repos/koch_tactile/Dataset_processor/Integrate

# 1. 編輯配置檔
nano config.yaml

# 2. 執行完整流程
python3 pipeline.py --config config.yaml
```

### 方法 2: 手動逐步處理

```bash
cd ~/Lerobot_system/repos/koch_tactile/Dataset_processor

# Step 1: 解析 rosbag
python3 1_parse_rosbag.py \
    --bag_path ~/Lerobot_system/Dataset/bags/0001 \
    --output_dir ~/Lerobot_system/Dataset/processed/0001_raw

# Step 2: 標註
python3 2_label_offscreen.py \
    --data_dir ~/Lerobot_system/Dataset/processed/0001_raw \
    --output_json segments.json

# Step 3: 分割資料
python3 3_split_by_segments.py \
    --data_dir ~/Lerobot_system/Dataset/processed/0001_raw \
    --segments segments.json \
    --output_dir ~/Lerobot_system/Dataset/processed/0001_split
```

---

## ⚙️ 配置範例

### Integrate/config.yaml

```yaml
# ===== 輸入來源 =====
sources:
  - path: /home/hrc/Lerobot_system/Dataset/bags/0001
    material_type: "Hard"
  - path: /home/hrc/Lerobot_system/Dataset/bags/0002
    material_type: "Soft"

# ===== 輸出設定 =====
output:
  extract_folder: "extracted_data"
  hdf5_dir: "/home/hrc/Lerobot_system/Dataset/hdf5"
  chart_dir: "/home/hrc/Lerobot_system/Dataset/charts"

# ===== 抽取參數 =====
extractor:
  topics:
    tactile_features: "/wowskin/contact_features_v2"
    tactile_image: "/wowskin/mgs_rgb_true_v2"
    camera_top: "/camera/camera_top/color/image_raw/compressed"
    camera_first: "/camera/camera_first/color/image_raw/compressed"
  
  image_format: "png"
  csv_output: "tactile_forces.csv"

# ===== 標註參數 =====
labeler:
  auto_label: true
  force_threshold: 0.5
  min_contact_duration: 0.2
  review_mode: false  # true=人工確認，false=自動

# ===== 分割參數 =====
splitter:
  segment_prefix: "seg"
  copy_images: true
  create_metadata: true

# ===== HDF5 轉換 =====
hdf5:
  compression: "gzip"
  chunk_size: 100
  include_images: true

# ===== 工作流程控制 =====
workflow:
  skip_existing: true   # 跳過已處理的資料夾
  verbose: true
```

---

## 📊 資料格式說明

### 1. 抽取後的原始資料

```
extracted_data/
├── images_camera_top/
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
├── images_camera_first/
│   ├── 000000.png
│   └── ...
├── images_tactile/
│   ├── 000000.png  # MGS 渲染圖 (64×64)
│   └── ...
├── tactile_forces.csv
└── metadata.json
```

### 2. tactile_forces.csv 格式

```csv
timestamp,sensor_0_x,sensor_0_y,sensor_0_z,sensor_1_x,...
1706545200.123,0.12,0.34,-0.56,0.01,...
1706545200.133,0.14,0.36,-0.58,0.02,...
```

### 3. 標註檔案格式 (segments.json)

```json
{
  "segments": [
    {
      "id": 1,
      "start_frame": 0,
      "end_frame": 150,
      "contact_type": "press",
      "material": "Hard"
    },
    {
      "id": 2,
      "start_frame": 151,
      "end_frame": 300,
      "contact_type": "slide",
      "material": "Hard"
    }
  ]
}
```

### 4. HDF5 訓練格式

```
dataset.hdf5
├── /episode_0000/
│   ├── /observation/
│   │   ├── camera_top (N, H, W, 3)
│   │   ├── camera_first (N, H, W, 3)
│   │   ├── tactile_image (N, 64, 64, 3)
│   │   └── tactile_forces (N, 15)  # 5 sensors × 3 axes
│   ├── /action/
│   │   └── joint_positions (N, 6)
│   └── /metadata/
│       ├── material_type: "Hard"
│       └── contact_type: "press"
└── /episode_0001/
    └── ...
```

---

## 🔍 進階功能

### 1. 視覺化接觸事件時間軸

```bash
cd ~/Lerobot_system/repos/koch_tactile/Data_recoder
python3 3_visualize_contact_feature_timeline.py \
    --csv_file ~/Lerobot_system/Dataset/processed/0001_raw/tactile_forces.csv \
    --output_image timeline.png
```

### 2. 批次處理多個 bag

編輯 `Integrate/config.yaml`，新增多個 sources：

```yaml
sources:
  - path: /home/hrc/Lerobot_system/Dataset/bags/0001
    material_type: "Hard"
  - path: /home/hrc/Lerobot_system/Dataset/bags/0002
    material_type: "Hard"
  - path: /home/hrc/Lerobot_system/Dataset/bags/0003
    material_type: "Soft"
```

### 3. 轉換至 LeRobot 格式

```bash
cd ~/Lerobot_system/repos/lerobot
python -m lerobot.common.datasets.push_dataset_to_hub \
    --raw-dir ~/Lerobot_system/Dataset/hdf5 \
    --repo-id your-username/koch-tactile-dataset \
    --raw-format koch_v1
```

---

## 📚 工具詳細說明

### repos/koch_tactile/sensor2/

WowSkin 觸覺感測器底層驅動與即時視覺化：

- `sensor_rawdata_v2.py` - ROS2 節點，發布原始特徵與 MGS 圖像
- `sensor_viz_MGS_v2.py` - 標準解析度 MGS 渲染 (64×64)
- `sensor_viz_MGS_v2_big.py` - 高解析度 MGS 渲染

### repos/koch_tactile/Train/

觸覺模型訓練工具：

- `train.py` - 訓練主程式
- `model.py` - 神經網路模型定義
- `dataset.py` - PyTorch Dataset 載入器
- `config.yaml` - 訓練超參數配置

詳見：`repos/koch_tactile/Train/README.md`

### repos/koch_tactile/Inference/

即時推論系統：

- `realtime_inference.py` - ROS2 推論節點
- `config_realtime.yaml` - 推論配置
- `launch_realtime.sh` - 快速啟動腳本

---

## 🐛 常見問題

### Q1: rosbag 解析失敗，提示 "No messages found"

**原因**: Topic 名稱不匹配

**解決**: 先檢查 bag 內容
```bash
ros2 bag info ~/Lerobot_system/Dataset/bags/0001/0001_0.db3
```

然後修改 `config.yaml` 中的 topic 名稱。

### Q2: 影像數量與 CSV 行數不一致

**原因**: 相機與觸覺感測器頻率不同

**解決**: 使用時間戳對齊
```python
# 在 extractor 配置中啟用
extractor:
  sync_by_timestamp: true
  max_time_diff: 0.05  # 50ms 容差
```

### Q3: HDF5 檔案過大

**解決**:
1. 啟用壓縮: `compression: "gzip"`
2. 降低影像解析度
3. 只包含必要的 camera view

### Q4: 標註工具無法顯示圖像

**原因**: 缺少 OpenCV GUI 支援

**解決**:
```bash
pip install opencv-python-headless  # 移除舊版
pip install opencv-python            # 安裝完整版
```

---

## 📖 相關文檔

- [BAG_RECORDER_GUIDE.md](BAG_RECORDER_GUIDE.md) - 數據錄製
- [MGS_GUIDE.md](../ros2_ws/src/koch_tactile_ros2/docs/MGS_GUIDE.md) - MGS 渲染器
- [Koch Tactile README](../repos/koch_tactile/README.md) - 完整工作流程

---

## 🔗 參考資料

- **LeRobot Dataset Format**: https://huggingface.co/docs/lerobot/dataset
- **ROS2 Bag API**: https://docs.ros.org/en/humble/p/rosbag2_py/
- **HDF5 Best Practices**: https://www.hdfgroup.org/solutions/hdf5/
