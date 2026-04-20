import os
import json
import numpy as np
import pyarrow.parquet as pq

DATASET_PATH = "/home/hrc/Lerobot_system/Dataset/Converted_dataset/merged_wipewuthTissue"

def analyze_dataset():
    episodes_file = os.path.join(DATASET_PATH, "meta", "episodes", "chunk-000", "file-000.parquet")
    if not os.path.exists(episodes_file):
        print(f"Error: {episodes_file} not found.")
        return

    print("Loading episodes metadata...")
    df_ep = pq.read_table(episodes_file).to_pandas()
    episodes = df_ep.to_dict('records')
            
    print(f"Total episodes found: {len(episodes)}")

    # 根據您的說明：前30筆=右手擦，後30筆(30-59)=左手擦
    group_right = episodes[:30]
    group_left = episodes[30:60]

    # 1. 比較總 Frame 數
    frames_right = sum([ep['length'] for ep in group_right])
    frames_left = sum([ep['length'] for ep in group_left])
    print(f"\n--- [1] Frame Count Comparison (影格數比較) ---")
    print(f"右手任務 (0-29): {frames_right} frames (平均每集: {frames_right/len(group_right):.1f} frames)")
    print(f"左手任務 (30-59): {frames_left} frames (平均每集: {frames_left/len(group_left):.1f} frames)")
    
    if frames_left > frames_right * 1.2:
        print(">> [警告] 左手的資料量明顯多於右手，模型訓練時摸到左手影格的機率高很多！")
    elif frames_right > frames_left * 1.2:
        print(">> [警告] 右手的資料量明顯多於左手。")
    else:
        print(">> 總資料量大致平衡。")

    # 2. 分析 Action 變異度
    parquet_dir = os.path.join(DATASET_PATH, "data")
    if not os.path.exists(parquet_dir):
        print(f"Error: Cannot find data directory: {parquet_dir}")
        return

    print("\n--- [2] Computing Action Variance (計算動作變異度) ---")
    actions = []
    episode_indices = []

    for root, dirs, files in os.walk(parquet_dir):
        for file in files:
            if file.endswith('.parquet'):
                try:
                    df = pq.read_table(os.path.join(root, file)).to_pandas()
                    if 'action' in df.columns and 'episode_index' in df.columns:
                        for act, ep_idx in zip(df['action'], df['episode_index']):
                            actions.append(act)
                            episode_indices.append(ep_idx)
                except Exception as e:
                    print(f"Error reading {file}: {e}")

    if not actions:
        print("No action data found in parquet files.")
        return

    actions = np.array(actions)
    episode_indices = np.array(episode_indices)

    mask_right = episode_indices < 30
    mask_left = (episode_indices >= 30) & (episode_indices < 60)

    act_right = actions[mask_right]
    act_left = actions[mask_left]

    if len(act_right) == 0 or len(act_left) == 0:
        print("Error: Missing data for left or right episodes.")
        return

    var_right = np.var(act_right, axis=0)
    var_left = np.var(act_left, axis=0)

    mean_var_right = np.mean(var_right)
    mean_var_left = np.mean(var_left)

    print(f"右手任務 (0-29) 的動作平均空間變異度: {mean_var_right:.6f}")
    print(f"左手任務 (30-59) 的動作平均空間變異度: {mean_var_left:.6f}")

    if mean_var_right > mean_var_left * 1.5:
        print(">> [大警告] ✋ 右手動作的變異度比左手大很多 (較不穩定/軌跡變化大)！這會導致模型為了降低誤差，選擇放棄學習右手、只保留學習穩定的左手！")
    elif mean_var_left > mean_var_right * 1.5:
        print(">> [大警告] ✋ 左手動作的變異度比右手大很多！")
    else:
        print(">> 雙手動作的變異度差異在合理範圍內。")

if __name__ == "__main__":
    analyze_dataset()
