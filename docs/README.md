# 文檔索引

本目錄包含 Koch Robot LeRobot System 的完整技術文檔。

---

## 📚 文檔結構

### 🚀 快速開始

| 文檔 | 說明 | 適用對象 |
|------|------|----------|
| [../README.md](../README.md) | 系統總覽與快速開始指南 | 所有使用者 |

---

## 📦 數據收集與處理

### 錄製數據

| 文檔 | 行數 | 內容 |
|------|------|------|
| [BAG_RECORDER_GUIDE.md](BAG_RECORDER_GUIDE.md) | 320 行 | **Rosbag 錄製工具完整指南**<br>• CLI/GUI 兩種操作模式<br>• YAML 配置檔詳解<br>• 自動編號與壓縮設定<br>• Topic 選擇與 QoS 配置 |

### 數據處理

| 文檔 | 行數 | 內容 |
|------|------|------|
| [DATA_PROCESSING_GUIDE.md](DATA_PROCESSING_GUIDE.md) | 335 行 | **數據後處理流程**<br>• Rosbag 解析與影像抽取<br>• 觸覺事件自動標註<br>• 資料切割與分段<br>• HDF5/LeRobot 格式轉換 |

---

## 🧠 機器學習與訓練

### LeRobot 框架

| 文檔 | 行數 | 內容 |
|------|------|------|
| [LEROBOT_INTEGRATION.md](LEROBOT_INTEGRATION.md) | 393 行 | **LeRobot 框架整合**<br>• 資料集準備與上傳至 HuggingFace Hub<br>• ACT/Diffusion Policy 訓練<br>• 多模態輸入配置<br>• 模型部署與推論 |

### VT-Refine 觸覺學習

| 文檔 | 行數 | 內容 |
|------|------|------|
| [VT_REFINE_GUIDE.md](VT_REFINE_GUIDE.md) | 457 行 | **視覺-觸覺融合學習**<br>• Isaac Sim 模擬環境配置<br>• DPPO 預訓練與微調<br>• Sim-to-Real 轉移技巧<br>• 領域隨機化設定 |

---

## 🤖 機器人控制系統

### 控制與操作

| 文檔 | 行數 | 內容 |
|------|------|------|
| [KOCH_ROBOT_MANUAL.md](KOCH_ROBOT_MANUAL.md) | 125 行 | **Koch 機器人控制系統**<br>• Dynamixel 馬達初始化<br>• Leader-Follower 遙操作<br>• 安全機制與緊急停止<br>• 故障排除指南 |

### 可視化

| 文檔 | 行數 | 內容 |
|------|------|------|
| [KOCH_ROBOT_VISUALIZATION.md](KOCH_ROBOT_VISUALIZATION.md) | 92 行 | **RViz 可視化設定**<br>• TF 座標樹配置<br>• 點雲與 Mesh 顯示<br>• 自訂 RViz 配置檔<br>• 即時數據監控 |

### 路徑規劃

| 文檔 | 行數 | 內容 |
|------|------|------|
| [KOCH_MOVEIT_MANUAL.md](KOCH_MOVEIT_MANUAL.md) | 138 行 | **MoveIt 運動規劃**<br>• 運動學配置<br>• 末端點位置控制<br>• 碰撞檢測與避障<br>• 軌跡規劃 API |

---

## 🔧 ROS2 套件文檔

位於 `../ros2_ws/src/` 各子套件目錄下：

### koch_tactile_ros2

| 文檔路徑 | 內容 |
|----------|------|
| `koch_tactile_ros2/docs/CODE_ARCHITECTURE.md` | ROS2 節點架構與 Topic 關係圖 |
| `koch_tactile_ros2/docs/MGS_GUIDE.md` | MGS 渲染器原理與使用 |

### bag_recorder

| 文檔路徑 | 內容 |
|----------|------|
| `bag_recorder/README.md` | (本文檔已取代，請參考 [BAG_RECORDER_GUIDE.md](BAG_RECORDER_GUIDE.md)) |

---

## 🧪 觸覺處理工具

位於 `../repos/koch_tactile/` 目錄：

| 文檔路徑 | 內容 |
|----------|------|
| `koch_tactile/README.md` | WowSkin 完整工作流程：資料收集、訓練、推論 |
| `koch_tactile/sensor2/README.md` | 感測器底層驅動與 MGS 實現細節 |
| `koch_tactile/Train/README.md` | 觸覺神經網路訓練配置 |

---

## 🌐 外部框架

### LeRobot (Hugging Face)

| 資源 | 連結 |
|------|------|
| 官方文檔 | https://huggingface.co/docs/lerobot |
| GitHub | https://github.com/huggingface/lerobot |
| 本地 README | `../repos/lerobot/README.md` |

**重點章節**:
- [Installation](https://huggingface.co/docs/lerobot/installation) - 安裝指南
- [Datasets](https://huggingface.co/docs/lerobot/dataset) - 資料集格式
- [Policies](https://huggingface.co/docs/lerobot/policies) - 支援的策略
- [Robots](https://huggingface.co/docs/lerobot/robots) - 機器人控制介面

### VT-Refine (NVIDIA)

| 資源 | 連結 |
|------|------|
| 論文 | https://arxiv.org/abs/2510.14930 |
| 專案網站 | https://binghao-huang.github.io/vt_refine/ |
| GitHub | https://github.com/NVlabs/vt-refine |
| 本地 README | `../repos/vt-refine/README.md` |

**重點章節**:
- Pre-training - 預訓練資料集與配置
- Fine-tuning - 模擬環境微調
- Evaluation - 模型評估指標

---

## 📖 閱讀路徑建議

### 初次使用者

1. [../README.md](../README.md) - 系統總覽
2. [KOCH_ROBOT_MANUAL.md](KOCH_ROBOT_MANUAL.md) - 機器人操作基礎
3. [BAG_RECORDER_GUIDE.md](BAG_RECORDER_GUIDE.md) - 開始錄製數據

### 數據科學家

1. [DATA_PROCESSING_GUIDE.md](DATA_PROCESSING_GUIDE.md) - 數據處理流程
2. [LEROBOT_INTEGRATION.md](LEROBOT_INTEGRATION.md) - LeRobot 訓練
3. `../repos/lerobot/README.md` - LeRobot 進階功能

### 研究人員（觸覺學習）

1. [VT_REFINE_GUIDE.md](VT_REFINE_GUIDE.md) - VT-Refine 整合
2. `../repos/koch_tactile/README.md` - 觸覺工具鏈
3. `../ros2_ws/src/koch_tactile_ros2/docs/MGS_GUIDE.md` - MGS 渲染原理

### 系統開發者

1. `../ros2_ws/src/koch_tactile_ros2/docs/CODE_ARCHITECTURE.md` - ROS2 架構
2. [KOCH_ROBOT_VISUALIZATION.md](KOCH_ROBOT_VISUALIZATION.md) - 可視化系統
3. [KOCH_MOVEIT_MANUAL.md](KOCH_MOVEIT_MANUAL.md) - 運動規劃

---

## 🔍 快速查詢

### 按主題分類

**硬體相關**
- [KOCH_ROBOT_MANUAL.md](KOCH_ROBOT_MANUAL.md) - 馬達控制
- [KOCH_ROBOT_VISUALIZATION.md](KOCH_ROBOT_VISUALIZATION.md) - 感測器可視化

**軟體相關**
- [BAG_RECORDER_GUIDE.md](BAG_RECORDER_GUIDE.md) - 資料錄製
- [DATA_PROCESSING_GUIDE.md](DATA_PROCESSING_GUIDE.md) - 資料處理
- `../ros2_ws/src/koch_tactile_ros2/docs/CODE_ARCHITECTURE.md` - 程式架構

**機器學習**
- [LEROBOT_INTEGRATION.md](LEROBOT_INTEGRATION.md) - 一般 Imitation Learning
- [VT_REFINE_GUIDE.md](VT_REFINE_GUIDE.md) - 觸覺強化學習

---

## 📝 文檔貢獻

若發現文檔錯誤或需要補充，請：

1. Fork 本專案
2. 修改對應的 Markdown 檔案
3. 提交 Pull Request

**撰寫規範**:
- 使用繁體中文撰寫，技術術語保留英文
- 程式碼區塊標註語言類型 (bash, python, yaml 等)
- 提供具體範例與指令
- 包含故障排除章節

---

## 📊 文檔統計

| 類別 | 文檔數量 | 總行數 |
|------|----------|--------|
| 系統總覽 | 1 | 653 |
| 數據收集 | 2 | 655 |
| 機器學習 | 2 | 850 |
| 機器人控制 | 3 | 355 |
| **總計** | **8** | **2513** |

---

**最後更新**: 2026-01-29  
**版本**: v2.0
