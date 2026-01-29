# VT-Refine 整合指南

VT-Refine (Visuo-Tactile Refinement) 是 NVIDIA 開發的雙臂裝配學習框架，透過模擬微調提升視覺-觸覺融合策略。

---

## 📋 VT-Refine 簡介

**論文**: [VT-Refine: Learning Bimanual Assembly with Visuo-Tactile Feedback via Simulation Fine-Tuning](https://arxiv.org/abs/2510.14930)  
**會議**: CoRL 2025  
**專案網站**: https://binghao-huang.github.io/vt_refine/

### 核心特點

- ✅ **視覺-觸覺融合** - 結合 RGB-D 與觸覺力回饋
- ✅ **Sim-to-Real** - 模擬環境預訓練 + 真實環境微調
- ✅ **雙臂協同** - 支援複雜的雙手操作任務
- ✅ **DPPO 訓練** - Distributed Proximal Policy Optimization

---

## 🗂️ 專案結構

VT-Refine 位於 `repos/vt-refine/`：

```
repos/vt-refine/
├── dppo/                  # 分散式 PPO 訓練框架
│   ├── cfg/               # 訓練配置檔
│   ├── script/            # 訓練腳本
│   └── README.md
├── easysim/               # Isaac Sim 模擬環境
│   ├── envs/              # 環境定義
│   └── utils/
├── easysim-envs/          # 環境資產
├── data/                  # 預訓練資料集
└── docker/                # 容器化環境
```

---

## 🚀 安裝與設定

### 1. Docker 環境（推薦）

VT-Refine 提供完整的 Docker 環境：

```bash
cd ~/Lerobot_system/repos/vt-refine/docker

# 建置映像檔
./build.sh

# 啟動容器（需設定 wandb entity）
DPPO_WANDB_ENTITY=your-team ./run.sh
```

### 2. 下載預訓練資料

```bash
cd ~/Lerobot_system/repos/vt-refine
./data/fetch_vt-refine_data.sh
```

資料集包含：
- **Aloha 雙臂操作資料** - 插入、組裝等任務
- **觸覺接觸標註** - Ground truth force measurements
- **視覺點雲** - RGB-D 處理後的 3D 點

---

## 🧠 預訓練（Pre-training）

### 1. 使用官方資料集

```bash
cd ~/Lerobot_system/repos/vt-refine

# Aloha 插入任務（預訓練配置 00186）
python3 dppo/script/run.py \
    --config-name=pre_tactile \
    --config-path=../cfg/aloha/pretrain/00186
```

### 2. 使用 Koch 資料集

建立 Koch 專用配置 `dppo/cfg/koch/pretrain/koch_tactile.yaml`：

```yaml
defaults:
  - _self_
  - override /policy: tactile_policy
  - override /env: koch_env

experiment_name: koch_tactile_pretrain
seed: 42

# ===== 環境設定 =====
env:
  name: koch_bimanual
  obs_mode: rgbd_tactile
  
  visual:
    camera_names: ["camera_top", "camera_first"]
    image_size: [224, 224]
    points_per_camera: 512
  
  tactile:
    num_sensors: 5
    feature_dim: 15  # 5 sensors × 3 axes
    image_size: [64, 64]

# ===== 策略網路 =====
policy:
  name: transformer_actor_critic
  
  visual_encoder:
    type: resnet18
    pretrained: true
  
  tactile_encoder:
    type: cnn
    channels: [32, 64, 128]
  
  fusion:
    type: cross_attention
    hidden_dim: 512
  
  actor:
    hidden_dims: [512, 512, 256]
    action_dim: 12  # 雙臂各 6 關節
  
  critic:
    hidden_dims: [512, 512, 256]

# ===== 訓練參數 =====
training:
  algorithm: ppo
  num_epochs: 1000
  batch_size: 256
  learning_rate: 3e-4
  gamma: 0.99
  gae_lambda: 0.95
  clip_range: 0.2
  
  # 分散式訓練
  num_workers: 8
  rollout_steps: 2048

# ===== 資料集 =====
dataset:
  train_path: /home/hrc/Lerobot_system/Dataset/vt_refine/koch_pretrain
  val_path: /home/hrc/Lerobot_system/Dataset/vt_refine/koch_val
  
  # 資料增強
  augmentation:
    color_jitter: true
    random_crop: true
    tactile_noise: 0.01

# ===== Logging =====
wandb:
  project: koch-vt-refine
  entity: your-team
  name: ${experiment_name}
```

執行訓練：

```bash
python3 dppo/script/run.py \
    --config-name=koch_tactile \
    --config-path=../cfg/koch/pretrain
```

---

## 🎮 模擬環境微調（Fine-tuning）

### 1. Isaac Sim 設定

VT-Refine 使用 NVIDIA Isaac Sim 進行模擬微調：

```bash
# 在 Docker 容器內
cd ~/Lerobot_system/repos/vt-refine

# 啟動微調訓練
python3 dppo/script/run.py \
    --config-name=finetune_tactile \
    --config-path=../cfg/aloha/finetune/sim_00186 \
    policy.pretrained_path=outputs/pretrain/checkpoint_best.pth
```

### 2. 自訂模擬環境

建立 Koch 模擬環境 `easysim/envs/koch_insertion.py`：

```python
import numpy as np
from easysim.envs.base_env import BaseEnv

class KochInsertionEnv(BaseEnv):
    def __init__(self, config):
        super().__init__(config)
        self.peg_pos = None
        self.hole_pos = None
    
    def reset(self):
        # 重置雙臂與物體位置
        self.robot.reset_joints()
        self.peg_pos = self._sample_peg_position()
        self.hole_pos = self._sample_hole_position()
        return self.get_observation()
    
    def step(self, action):
        # 執行動作
        self.robot.set_joint_positions(action)
        self.simulator.step()
        
        # 計算獎勵
        obs = self.get_observation()
        reward = self._compute_reward()
        done = self._check_termination()
        
        return obs, reward, done, {}
    
    def get_observation(self):
        return {
            "visual_points": self._get_visual_pointcloud(),
            "tactile_features": self._get_tactile_features(),
            "tactile_image": self._get_tactile_image(),
            "state": self.robot.get_joint_positions()
        }
    
    def _compute_reward(self):
        # 距離獎勵
        dist = np.linalg.norm(self.peg_pos - self.hole_pos)
        reward = -dist
        
        # 接觸獎勵（觸覺回饋）
        tactile = self._get_tactile_features()
        contact_reward = np.sum(np.abs(tactile)) * 0.1
        
        # 成功獎勵
        if dist < 0.01:
            reward += 100.0
        
        return reward + contact_reward
```

---

## 🤖 真實機器人部署

### 1. 載入預訓練模型

```python
import torch
from dppo.models.policy import TactilePolicy

# 載入 checkpoint
checkpoint = torch.load("outputs/finetune/checkpoint_best.pth")
policy = TactilePolicy(config)
policy.load_state_dict(checkpoint['policy_state_dict'])
policy.eval()

# 推論
with torch.no_grad():
    obs = {
        "visual_points": visual_points,      # (1024, 3)
        "tactile_features": tactile_forces,  # (15,)
        "tactile_image": tactile_image,      # (64, 64, 3)
        "state": joint_positions             # (12,)
    }
    action, _ = policy(obs)
```

### 2. ROS2 整合

建立 ROS2 推論節點 `ros2_ws/src/vt_refine_inference/`：

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import torch

class VTRefineInferenceNode(Node):
    def __init__(self):
        super().__init__('vt_refine_inference')
        
        # 載入模型
        self.policy = self.load_policy()
        
        # 訂閱感測器資料
        self.create_subscription(
            PointCloud2, '/visual_points_pc2', self.visual_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/tactile_points', self.tactile_callback, 10)
        
        # 發布控制指令
        self.pub = self.create_publisher(
            JointState, '/right_follower/joint_states_control', 10)
        
        self.timer = self.create_timer(0.033, self.inference_loop)  # 30Hz
    
    def inference_loop(self):
        if not self.is_ready():
            return
        
        obs = self.collect_observation()
        action = self.policy_inference(obs)
        self.publish_action(action)
```

啟動：

```bash
ros2 run vt_refine_inference vt_refine_node
```

---

## 📊 訓練監控

### WandB 整合

VT-Refine 預設使用 Weights & Biases：

```bash
# 設定 entity
export DPPO_WANDB_ENTITY=your-team

# 訓練時自動記錄
python3 dppo/script/run.py ...
```

查看訓練曲線：https://wandb.ai/your-team/koch-vt-refine

### 關鍵指標

- **Episode Return** - 累積獎勵
- **Success Rate** - 任務成功率
- **Contact Accuracy** - 觸覺預測準確度
- **Policy Loss** - Actor loss
- **Value Loss** - Critic loss

---

## 🔧 進階配置

### 1. 領域隨機化（Domain Randomization）

增強 Sim-to-Real 轉移：

```yaml
env:
  domain_randomization:
    enable: true
    
    visual:
      lighting: [0.5, 1.5]
      texture: true
      camera_noise: 0.01
    
    tactile:
      sensor_noise: 0.02
      offset_range: [-0.001, 0.001]
    
    physics:
      friction: [0.5, 1.5]
      mass: [0.8, 1.2]
```

### 2. 課程學習（Curriculum Learning）

```yaml
training:
  curriculum:
    enable: true
    stages:
      - name: easy
        epochs: 200
        task_difficulty: 0.3
      
      - name: medium
        epochs: 400
        task_difficulty: 0.6
      
      - name: hard
        epochs: 400
        task_difficulty: 1.0
```

---

## 📚 相關文檔

### 官方資源
- **論文**: https://arxiv.org/abs/2510.14930
- **專案網站**: https://binghao-huang.github.io/vt_refine/
- **GitHub**: https://github.com/NVlabs/vt-refine
- **影片示範**: https://youtu.be/AEt30ttJ9A8

### 本專案相關
- [LEROBOT_INTEGRATION.md](LEROBOT_INTEGRATION.md) - LeRobot 框架整合
- [DATA_PROCESSING_GUIDE.md](DATA_PROCESSING_GUIDE.md) - 資料集準備
- [MGS_GUIDE.md](../ros2_ws/src/koch_tactile_ros2/docs/MGS_GUIDE.md) - MGS 觸覺渲染

---

## 🐛 常見問題

### Q1: Isaac Sim 無法啟動

**原因**: GPU 驅動版本不符

**解決**: 使用 VT-Refine 提供的 Docker 環境
```bash
cd ~/Lerobot_system/repos/vt-refine/docker
./run.sh
```

### Q2: 訓練時觸覺特徵未使用

**檢查**: 確認配置檔啟用觸覺輸入
```yaml
policy:
  use_tactile: true
  tactile_fusion_layer: 3  # 在第 3 層融合
```

### Q3: Sim-to-Real Gap 過大

**解決方案**:
1. 增加領域隨機化強度
2. 收集更多真實世界資料進行微調
3. 使用 System Identification 校正模擬參數

---

## 📖 引用

若使用 VT-Refine，請引用原論文：

```bibtex
@INPROCEEDINGS{huang:corl2025,
  author    = {Binghao Huang and Jie Xu and Iretiayo Akinola and Wei Yang and 
               Balakumar Sundaralingam and Rowland O'Flaherty and Dieter Fox and 
               Xiaolong Wang and Arsalan Mousavian and Yu-Wei Chao and Yunzhu Li},
  booktitle = {Conference on Robot Learning (CoRL)},
  title     = {{VT-Refine}: Learning Bimanual Assembly with Visuo-Tactile Feedback 
               via Simulation Fine-Tuning},
  year      = {2025},
}
```
