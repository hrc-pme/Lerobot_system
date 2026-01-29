# LeRobot 框架整合指南

本指南說明如何將 Koch Robot 系統與 LeRobot 框架整合，進行 imitation learning 訓練。

---

## 📋 LeRobot 簡介

[LeRobot](https://github.com/huggingface/lerobot) 是 Hugging Face 開發的機器人學習框架，提供：

- 🤖 **統一的機器人控制介面** - 支援多種硬體平台
- 📊 **標準化資料集格式** - Parquet + MP4/Images
- 🧠 **State-of-the-art 策略** - ACT, Diffusion Policy, VQ-BeT 等
- 🌐 **Hugging Face Hub 整合** - 資料集與模型分享

---

## 🗂️ 專案結構

LeRobot 位於 `repos/lerobot/`，包含：

```
repos/lerobot/
├── src/lerobot/           # 核心程式碼
│   ├── common/            # 資料集、環境、策略
│   ├── robots/            # 機器人控制介面
│   └── scripts/           # 訓練與評估腳本
├── examples/              # 使用範例
├── docs/                  # 完整文檔
└── requirements.txt       # Python 依賴
```

---

## 🚀 安裝與設定

### 1. 安裝 LeRobot

```bash
cd ~/Lerobot_system/repos/lerobot

# 開發模式安裝
pip install -e .

# 驗證安裝
lerobot-info
```

### 2. 安裝額外依賴（針對訓練）

```bash
# PyTorch with CUDA (根據您的 CUDA 版本)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# 訓練相關
pip install wandb tensorboard
wandb login
```

---

## 📊 資料集準備

### 1. 資料集格式轉換

LeRobot 使用 **Parquet + MP4** 格式，需將 ROS2 bag 轉換：

```bash
cd ~/Lerobot_system/repos/lerobot

python -m lerobot.common.datasets.push_dataset_to_hub \
    --raw-dir ~/Lerobot_system/Dataset/processed \
    --repo-id your-username/koch-tactile-demo \
    --raw-format koch_v1 \
    --local-dir ~/Lerobot_system/Dataset/lerobot_format
```

### 2. 上傳至 Hugging Face Hub（選填）

```bash
# 登入
huggingface-cli login

# 上傳
python -m lerobot.common.datasets.push_dataset_to_hub \
    --raw-dir ~/Lerobot_system/Dataset/processed \
    --repo-id your-username/koch-tactile-demo \
    --push-to-hub
```

### 3. 本地載入資料集

```python
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(
    "your-username/koch-tactile-demo",
    root="~/Lerobot_system/Dataset/lerobot_format"
)

print(f"Dataset size: {len(dataset)}")
print(f"Keys: {dataset[0].keys()}")
```

---

## 🧠 模型訓練

### 1. ACT (Action Chunking Transformer)

LeRobot 預設推薦的策略，適合雙臂操作任務：

```bash
cd ~/Lerobot_system/repos/lerobot

python lerobot/scripts/train.py \
    policy=act \
    env=koch \
    dataset_repo_id=your-username/koch-tactile-demo \
    training.offline_steps=50000 \
    training.batch_size=8 \
    training.save_checkpoint=true \
    wandb.enable=true \
    wandb.project=koch-tactile
```

### 2. Diffusion Policy

適合需要精細操作的任務：

```bash
python lerobot/scripts/train.py \
    policy=diffusion \
    env=koch \
    dataset_repo_id=your-username/koch-tactile-demo \
    training.offline_steps=100000 \
    training.batch_size=16
```

### 3. 自訂配置

建立配置檔 `configs/koch_tactile.yaml`：

```yaml
env:
  name: koch
  fps: 30
  state_dim: 12  # 左右手各 6 個關節
  action_dim: 12

policy:
  name: act
  n_obs_steps: 1
  chunk_size: 100
  hidden_dim: 512
  dim_feedforward: 3200
  n_heads: 8
  n_encoder_layers: 4
  n_decoder_layers: 7

training:
  offline_steps: 50000
  batch_size: 8
  lr: 1e-4
  device: cuda
  save_freq: 5000
  eval_freq: 5000

dataset:
  image_transforms:
    enable: true
    brightness: [0.9, 1.1]
    contrast: [0.9, 1.1]
```

執行：

```bash
python lerobot/scripts/train.py --config-name koch_tactile
```

---

## 🤖 機器人控制整合

### 1. 建立 Koch Robot 控制器

在 `repos/lerobot/src/lerobot/robots/` 建立 `koch.py`：

```python
from lerobot.common.robot_devices.robots.utils import RobotDeviceAlreadyConnectedError
from lerobot.common.robot_devices.robots.base import Robot

class KochRobot(Robot):
    def __init__(self, config):
        super().__init__(config)
        self.left_arm = None
        self.right_arm = None
    
    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError("Koch Robot already connected")
        # 初始化 Dynamixel 控制器
        # ...
        self.is_connected = True
    
    def get_observation(self):
        obs = {
            "observation.state": self._get_joint_positions(),
            "observation.images.camera_top": self._get_camera_image("top"),
            "observation.images.camera_first": self._get_camera_image("first"),
            "observation.tactile.forces": self._get_tactile_forces(),
        }
        return obs
    
    def send_action(self, action):
        # 發送關節位置指令
        # ...
        pass
```

### 2. 註冊機器人

在 `repos/lerobot/src/lerobot/robots/__init__.py` 加入：

```python
from lerobot.robots.koch import KochRobot

ROBOT_REGISTRY = {
    "koch": KochRobot,
    # ... 其他機器人
}
```

### 3. 執行推論

```python
from lerobot.robots import make_robot
from lerobot.common.policies.act.modeling_act import ACTPolicy

# 載入訓練好的模型
policy = ACTPolicy.from_pretrained("your-username/koch-act-policy")

# 連接機器人
robot = make_robot("koch", config=robot_config)
robot.connect()

# 執行策略
for _ in range(1000):
    obs = robot.get_observation()
    action = policy.select_action(obs)
    robot.send_action(action)
```

---

## 📈 訓練監控

### 使用 Weights & Biases

1. 登入：
```bash
wandb login
```

2. 訓練時啟用 wandb：
```bash
python lerobot/scripts/train.py \
    ... \
    wandb.enable=true \
    wandb.project=koch-tactile \
    wandb.entity=your-team
```

3. 查看訓練曲線：https://wandb.ai/your-team/koch-tactile

### 使用 TensorBoard

```bash
# 訓練時自動記錄
tensorboard --logdir outputs/train/

# 瀏覽器開啟 http://localhost:6006
```

---

## 🔧 進階配置

### 1. 多模態輸入（視覺 + 觸覺）

修改策略配置以包含觸覺輸入：

```yaml
policy:
  input_shapes:
    observation.images.camera_top: [3, 480, 640]
    observation.images.camera_first: [3, 480, 640]
    observation.tactile.image: [3, 64, 64]      # MGS 圖像
    observation.tactile.forces: [15]             # 5 sensors × 3 axes
    observation.state: [12]                      # 關節位置
```

### 2. 資料增強

```yaml
dataset:
  image_transforms:
    enable: true
    brightness: [0.8, 1.2]
    contrast: [0.8, 1.2]
    saturation: [0.8, 1.2]
    hue: [-0.05, 0.05]
    crop_size: [224, 224]
```

### 3. 分散式訓練

```bash
torchrun --nproc_per_node=4 lerobot/scripts/train.py \
    policy=act \
    env=koch \
    dataset_repo_id=your-username/koch-tactile-demo \
    training.batch_size=32
```

---

## 📚 LeRobot 核心文檔

官方文檔位於 `repos/lerobot/docs/`：

- **Installation**: https://huggingface.co/docs/lerobot/installation
- **Datasets**: https://huggingface.co/docs/lerobot/dataset
- **Policies**: https://huggingface.co/docs/lerobot/policies
- **Robots**: https://huggingface.co/docs/lerobot/robots

---

## 🔗 相關資源

### 範例資料集
- [lerobot/pusht](https://huggingface.co/datasets/lerobot/pusht)
- [lerobot/aloha_sim_insertion_human](https://huggingface.co/datasets/lerobot/aloha_sim_insertion_human)

### 預訓練模型
- [lerobot/act_pusht](https://huggingface.co/lerobot/act_pusht)
- [lerobot/diffusion_pusht](https://huggingface.co/lerobot/diffusion_pusht)

### 教學影片
- [LeRobot Tutorial Series](https://www.youtube.com/playlist?list=PLo2EIpI_JMQtNtKNFFSMNIZwspj8H7-sL)

---

## 🐛 常見問題

### Q1: 訓練時 GPU 記憶體不足

**解決**:
```yaml
training:
  batch_size: 4           # 減少 batch size
  gradient_accumulation: 2  # 使用梯度累積
```

### Q2: 資料集載入過慢

**解決**: 啟用快取
```python
dataset = LeRobotDataset(
    "your-username/koch-tactile-demo",
    cache_dir="~/.cache/lerobot"
)
```

### Q3: 如何 Fine-tune 預訓練模型？

```bash
python lerobot/scripts/train.py \
    policy=act \
    env=koch \
    dataset_repo_id=your-username/koch-tactile-demo \
    policy.pretrained_model_path=lerobot/act_pusht \
    training.freeze_encoder=true
```

---

## 📖 下一步

1. 閱讀 [DATA_PROCESSING_GUIDE.md](DATA_PROCESSING_GUIDE.md) 準備訓練資料
2. 執行 [LeRobot Tutorial](https://github.com/huggingface/lerobot/tree/main/examples)
3. 參考 [VT-Refine Integration](VT_REFINE_GUIDE.md) 整合觸覺強化學習
