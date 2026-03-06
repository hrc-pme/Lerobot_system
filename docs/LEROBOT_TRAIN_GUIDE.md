# LeRobot 訓練腳本完整指南 (`lerobot_train.py`)

本文件詳細說明 `lerobot/scripts/lerobot_train.py` 的使用方式、配置參數以及針對不同硬體環境的建議設定。

## 🚀 快速開始

### 基礎指令格式

LeRobot 使用階層式的配置系統。所有的參數都可以透過命令行以 `.` (點號) 分隔的階層方式進行覆蓋。

```bash
python3 lerobot/scripts/lerobot_train.py \
  --policy.type <模型類型> \
  --dataset.repo_id <資料集名稱> \
  --dataset.root <資料集路徑> \
  --output_dir <輸出路徑>
```

### 🔥 針對 RTX 4070 (12GB VRAM) 的推薦指令

若您的顯存有限 (12GB)，請務必使用以下設定以避免 OOM (Out Of Memory)：

```bash
python3 /home/hrc/Lerobot_system/repos/lerobot/src/lerobot/scripts/lerobot_train.py \
  --dataset.repo_id koch_bi_wipe_water \
  --dataset.root /home/hrc/Lerobot_system/Dataset/converted_dataset3 \
  --policy.type act \
  --policy.repo_id lesterliou02/koch_bi_wipe_water_policy \
  --output_dir /home/hrc/Lerobot_system/output2 \
  --batch_size 4 \
  --policy.use_amp true \
  --num_workers 4 \
  --wandb.enable true \
  --wandb.mode offline
```

---

## 📚 完整參數說明

### 1. 核心訓練參數 (Top-level)

這些參數直接控制訓練流程，無需前綴。

| 參數 | 說明 | 預設值 | 建議值 |
| :--- | :--- | :--- | :--- |
| `--output_dir` | 模型 Checkpoint 與 Log 的輸出目錄。 | `outputs/train/...` | 絕對路徑 |
| `--batch_size` | **全域批次大小**。若顯存不足請調小此值。 | `8` | `4` 或 `2` (12GB VRAM) |
| `--steps` | 總訓練步數。 | `100000` | `100000`+ |
| `--save_freq` | 儲存 Checkpoint 的頻率 (步數)。 | `10000` | `10000` |
| `--eval_freq` | 執行評估的頻率 (若有設定 env)。 | `20000` | - |
| `--log_freq` | 更新 Log (WandB/Terminal) 的頻率。 | `200` | `200` |
| `--seed` | 隨機種子，確保結果可重現。 | `1000` | - |
| `--num_workers` | 資料讀取 (DataLoader) 的執行緒數量。 | `4` | `4` (過高可能導致 RAM 不足) |
| `--resume` | 是否從 `output_dir` 中最新的 checkpoint 繼續訓練。 | `false` | `true` (中斷後) |

### 2. 資料集設定 (`--dataset.*`)

控制資料的來源與預處理。

| 參數 | 說明 | 範例 / 備註 |
| :--- | :--- | :--- |
| `--dataset.repo_id` | **必要**。資料集名稱 (Hugging Face ID 或本地資料夾名)。 | `koch_bi_wipe_water` |
| `--dataset.root` | **必要** (本地訓練)。包含資料集的根目錄路徑。 | `/home/hrc/.../Dataset/converted_dataset3` |
| `--dataset.episodes` | 指定要使用的 Episode 索引列表 (用於測試)。 | `[0, 1, 2]` |
| `--dataset.image_transforms.enable` | 是否啟用影像增強 (Augmentation)。 | `true` / `false` |
| `--dataset.use_imagenet_stats` | 是否使用 ImageNet 的 Mean/Std 進行正規化。 | `true` (通常建議開啟) |
| `--dataset.video_backend` | 影片解碼後端。 | `pyav` (預設) 或 `torchcodec` |

### 3. 策略 (模型) 設定 (`--policy.*`)

根據 `--policy.type` 不同，可用的參數會有所變化。以下以 `act` 為主。

| 參數 | 說明 | 範例 / 備註 |
| :--- | :--- | :--- |
| `--policy.type` | **必要**。模型架構。 | `act`, `diffusion`, `vqbet` |
| `--policy.repo_id` | 上傳到 Hugging Face Hub 的目標 ID。 | `User/PolicyName` |
| `--policy.use_amp` | **關鍵**。啟用混合精度 (Automatic Mixed Precision)。 | `true` (省顯存必開) |
| `--policy.n_action_steps` | 模型預測未來的動作步數 (Chunk Size)。 | `100` (ACT 標準) |
| `--policy.dim_model` | Transformer 模型維度。 | `512` |
| `--policy.n_heads` | Attention Heads 數量。 | `8` |
| `--policy.n_encoder_layers` | Encoder 層數。 | `4` |
| `--policy.vision_backbone` | 影像特徵提取網路。 | `resnet18`, `resnet34` |
| `--policy.device` | 運算裝置。 | `cuda`, `cpu` |

### 4. Weights & Biases 紀錄 (`--wandb.*`)

LeRobot 深度整合 WandB 進行實驗追蹤。

| 參數 | 說明 | 選項 |
| :--- | :--- | :--- |
| `--wandb.enable` | 是否啟用 WandB。 | `true` / `false` |
| `--wandb.project` | WandB 上的專案名稱。 | `lerobot_koch` |
| `--wandb.mode` | 連線模式。 | `online` (上傳), `offline` (存本地) |
| `--wandb.notes` | 實驗備註。 | "Test run with AMP" |

### 5. 優化器與排程器 (`--optimizer.*`, `--scheduler.*`)

通常使用預設值即可，除非需要微調收斂速度。

| 參數 | 說明 | 預設值 |
| :--- | :--- | :--- |
| `--optimizer.type` | 優化器類型。 | `adamw` |
| `--optimizer.lr` | 學習率。 | `1e-5` (ACT 預設較低) |
| `--optimizer.weight_decay` | 權重衰減。 | `1e-4` |
| `--optimizer.grad_clip_norm` | 梯度裁剪閾值。 | `10.0` |

---

## 🛠️ 常見問題排解

### 1. CUDA out of memory (OOM)
這表示顯存不足。解決方案 (按順序嘗試)：
1.  **啟用 AMP**: 加入 `--policy.use_amp true`。
2.  **降低 Batch Size**: 將 `--batch_size` 降為 `4` 或 `2`。
3.  **減少 Workers**: 將 `--num_workers` 降為 `2`。
4.  **更換 Backbone**: 若使用 ResNet50，改為 `--policy.vision_backbone resnet18`。

### 2. FileNotFoundError / Dataset not found
LeRobot 找不到資料集。
*   檢查 `--dataset.root` 路徑是否正確。
*   確認目錄結構：`{root}/{repo_id}/meta/...`
*   如果是本地資料，確保不要包含 `huggingface.co` 相關的網址錯誤。

### 3. Training Loss 是 NaN
*   學習率 (`lr`) 太高，嘗試降低。
*   資料正規化有問題 (檢查 `dataset_stats` 是否正確生成)。
*   `grad_clip_norm` 設得太寬鬆。
