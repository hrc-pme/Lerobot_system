import numpy as np
import cv2
import sys
import matplotlib.pyplot as plt
from tqdm import tqdm
import os

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError as e:
    print(f"請在您的 LeRobot 虛擬環境中執行此腳本。錯誤: {e}")
    sys.exit(1)

# Dataset path
root_dir = "/home/hrc/Lerobot_system/Dataset/Converted_dataset/merged_wipewuthTissue"

print(f"正在載入資料集: {root_dir}")
dataset = LeRobotDataset("local/merged", root=root_dir)

# 提取 Episode 0 (例如:第一筆資料) 第一個 frame
idx_a = 0

# 提取 Episode 30 (例如:第 31 筆資料) 第一個 frame
# 我們可以藉由 hf_dataset 來找到 Episode 30 開頭的 index
hf_ep_indices = dataset.hf_dataset['episode_index']
try:
    idx_b = hf_ep_indices.index(55)
except ValueError:
    idx_b = hf_ep_indices.index(dataset.meta.episodes[-1]['episode_index']) # 或取最後一筆


frame_a = dataset[idx_a]
frame_b = dataset[idx_b]

# 找出所有相機畫面的 Key
image_keys = [k for k in frame_a.keys() if "images" in k]
print(f"找到以下相機畫面: {image_keys}")

fig, axes = plt.subplots(len(image_keys), 3, figsize=(15, 5 * len(image_keys)))
if len(image_keys) == 1:
    axes = np.expand_dims(axes, axis=0)

for row, cam_key in enumerate(image_keys):
    img_a_tensor = frame_a[cam_key]
    img_b_tensor = frame_b[cam_key]
    
    # 轉換 Tensor [C, H, W] 到 Numpy [H, W, C] 並統一轉成 uint8
    if img_a_tensor.dtype.is_floating_point:
        img_a = (img_a_tensor.permute(1, 2, 0).numpy() * 255.0).astype(np.uint8)
        img_b = (img_b_tensor.permute(1, 2, 0).numpy() * 255.0).astype(np.uint8)
    else:
        img_a = img_a_tensor.permute(1, 2, 0).numpy().astype(np.uint8)
        img_b = img_b_tensor.permute(1, 2, 0).numpy().astype(np.uint8)

    # 計算兩張圖片（不同情境）的絕對像素差異
    diff = cv2.absdiff(img_a, img_b)
    
    # 轉灰階並增強對比 (找出哪裡有差異)
    diff_gray = cv2.cvtColor(diff, cv2.COLOR_RGB2GRAY)
    
    # 計算平均差異分數 (評估兩張圖片在機器學習眼裡到底有多像)
    mean_diff = np.mean(diff_gray)
    
    # 將差異上色 (紅色代表差異很大，藍色代表沒有差異)
    heatmap = cv2.applyColorMap(diff_gray, cv2.COLORMAP_JET)
    
    # 畫圖
    axes[row, 0].imshow(img_a)
    axes[row, 0].set_title(f"Episode 0 (Setup A)\n{cam_key}")
    axes[row, 0].axis('off')
    
    axes[row, 1].imshow(img_b)
    axes[row, 1].set_title(f"Episode 55 (Setup B)\n{cam_key}")
    axes[row, 1].axis('off')
    
    axes[row, 2].imshow(cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB))
    axes[row, 2].set_title(f"Pixel Difference Heatmap\n(Mean Diff: {mean_diff:.2f})\nRed=High Diff")
    axes[row, 2].axis('off')

plt.tight_layout()
save_path = "/home/hrc/Lerobot_system/visual_cue_analysis.png"
plt.savefig(save_path, bbox_inches='tight')
print(f"視覺分析結果已儲存至: {save_path}")
print("請點開這個圖表，觀察『Heatmap (熱力圖)』！")
