import sys
import os
import json
import torch
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import safetensors.torch as safetensors

# Add the lerobot repository to the Python path
sys.path.append("/home/hrc/Lerobot_system/repos/lerobot/src")
from lerobot.policies.act.modeling_act import ACTPolicy

# Replace with your actual checkpoint path
CHECKPOINT_DIR = "/home/hrc/Lerobot_system/outputs_nano/merged_wipewuthTissue_hyp2/checkpoints/060000/pretrained_model"
OUTPUT_DIR = "/home/hrc/Lerobot_system/outputs_nano/merged_wipewuthTissue_hyp2"

def analyze_normalizations():
    print("\n--- [1] Analyzing Normalization Statistics ---")
    norm_file = None
    for f in os.listdir(CHECKPOINT_DIR):
        if "normalizer_processor.safetensors" in f:
            norm_file = os.path.join(CHECKPOINT_DIR, f)
            break
            
    if norm_file and os.path.exists(norm_file):
        try:
            norms = safetensors.load_file(norm_file)
            action_std = None
            for key in norms:
                if 'action' in key and 'std' in key:
                    action_std = norms[key].numpy()
                    break
                    
            if action_std is not None:
                if len(action_std.shape) > 1:
                    action_std = action_std[0]
                
                plt.figure(figsize=(10, 4))
                plt.bar(range(len(action_std)), action_std)
                plt.title("Action Standard Deviation (Normalization Scale)")
                plt.xlabel("Joint Index (0-5 Right Arm, 6-11 Left Arm)")
                plt.ylabel("Standard Deviation")
                plt.axvline(x=5.5, color='r', linestyle='--', label='Right / Left boundary')
                plt.legend()
                
                save_path = os.path.join(OUTPUT_DIR, "norm_std_plot.png")
                plt.savefig(save_path)
                print(f"✅ Normalization plot saved to: {save_path}")
                plt.close()
            else:
                print("Action std not found in normalizer.")
        except Exception as e:
            print("Could not load normalizers:", e)
    else:
        print("Could not find normalizer file.")

def analyze_inference():
    print("\n--- [2] Evaluating Model Inference (Action Chunks) ---")
    def get_dummy_batch(batch_size=1, config=None):
        obs = {}
        n_obs_steps = getattr(config, "n_obs_steps", 1)
        input_features = getattr(config, "input_features", {})
        
        for k, v in input_features.items():
            shape = getattr(v, "shape", [])
            v_type = str(getattr(v, "type", ""))
            
            if "VISUAL" in v_type:
                # Remove n_obs_steps from image initialization to match what ACT expects for batched Conv2D
                obs[k] = torch.zeros((batch_size, *shape), dtype=torch.float32).cuda()
            elif "STATE" in v_type:
                # ACT expects state to have n_obs_steps or not depending on context, keeping simple
                obs[k] = torch.zeros((batch_size, *shape), dtype=torch.float32).cuda()
                
        if "observation.state" not in obs:
            obs["observation.state"] = torch.zeros((batch_size, 12), dtype=torch.float32).cuda()
        return obs

    try:
        policy = ACTPolicy.from_pretrained(str(CHECKPOINT_DIR)).cuda()
        policy.eval()
        obs_batch = get_dummy_batch(1, policy.config)
        
        with torch.no_grad():
            actions = policy.select_action(obs_batch)
            
        print(f"Action Prediction Shape: {actions.shape}")
        actions_np = actions.squeeze(0).cpu().numpy()
        
        # In case action shape is (12,) vs (chunk_size, 12)
        if len(actions_np.shape) == 1:
            actions_np = actions_np[np.newaxis, :]
            
        fig, axes = plt.subplots(1, 2, figsize=(15, 4))
        # Ploting initial 6 actions (Right arm)
        # Using [0, :6] to just plot a single step if shape is 1D or all steps if 2D
        if len(actions_np.shape) == 2:
            axes[0].plot(actions_np[:, :6])
            axes[1].plot(actions_np[:, 6:])
        else:
            axes[0].plot(actions_np[:6], marker='o')
            axes[1].plot(actions_np[6:], marker='o')
            
        axes[0].set_title('Predicted Action (Right Arm) using Zeros')
        axes[1].set_xlabel('Steps ahead')
        
        save_path = os.path.join(OUTPUT_DIR, "inference_bias_plot.png")
        plt.savefig(save_path)
        print(f"✅ Inference plot saved to: {save_path}")
        plt.close()
        
    except Exception as e:
        print(f"Failed to infer from policy. Details: {e}")

if __name__ == "__main__":
    print(f"Diagnostics starting for checkpoint: {CHECKPOINT_DIR}")
    analyze_normalizations()
    analyze_inference()
    print("\nDiagnostics complete!")