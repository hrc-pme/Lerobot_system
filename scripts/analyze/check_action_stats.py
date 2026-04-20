import pyarrow.parquet as pq
import numpy as np

DATASET_PATH = "/home/hrc/Lerobot_system/Dataset/Converted_dataset/merged_wipewuthTissue"

def check_constant_actions():
    df = pq.read_table(f"{DATASET_PATH}/data/chunk-000/file-000.parquet").to_pandas()
    
    right_mask = df['episode_index'] < 30
    left_mask = (df['episode_index'] >= 30) & (df['episode_index'] < 60)
    
    right_actions = np.vstack(df[right_mask]['action'].values)
    left_actions = np.vstack(df[left_mask]['action'].values)
    
    print("Action shape:", right_actions.shape)
    
    print("\n--- Right Arm Episodes (右手任務) ---")
    print("Action array min values per joint:", np.min(right_actions, axis=0))
    print("Action array max values per joint:", np.max(right_actions, axis=0))
    print("Variance per joint:", np.var(right_actions, axis=0))
    
    print("\n--- Left Arm Episodes (左手任務) ---")
    print("Action array min values per joint:", np.min(left_actions, axis=0))
    print("Action array max values per joint:", np.max(left_actions, axis=0))
    print("Variance per joint:", np.var(left_actions, axis=0))

if __name__ == "__main__":
    check_constant_actions()