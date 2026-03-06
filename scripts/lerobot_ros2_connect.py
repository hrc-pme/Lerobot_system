#!/usr/bin/env python3
import time
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, CompressedImage, Image
from cv_bridge import CvBridge
import cv2  # For decoding compressed images
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.processor import PolicyProcessorPipeline, PolicyAction
from lerobot.utils.constants import ACTION
from pathlib import Path

# Adjust this path to your checkpoint
CHECKPOINT_PATH = "/home/hrc/Lerobot_system/outputs_nano/output/checkpoints/100000/pretrained_model"

class LeRobotROS2Bridge(Node):
    def __init__(self):
        super().__init__('lerobot_policy_node')
        
        # --- 1. Load Policy ---
        self.get_logger().info(f"Loading policy from {CHECKPOINT_PATH}...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Load the policy from the checkpoint folder
        # pretrained_model usually contains config.json and model.safetensors
        self.policy = ACTPolicy.from_pretrained(CHECKPOINT_PATH).to(self.device)

        # Load Preprocessor (Normalization)
        self.preprocessor = PolicyProcessorPipeline.from_pretrained(
            CHECKPOINT_PATH, 
            config_filename="policy_preprocessor.json"
        )

        # Load Postprocessor (Un-normalization)
        self.postprocessor = PolicyProcessorPipeline.from_pretrained(
            CHECKPOINT_PATH, 
            config_filename="policy_postprocessor.json"
        )
        
        # --- Policy Inference Settings ---
        # 1. Temporal Ensembling (Optional)
        # Enable this to smooth out actions, but requires higher compute.
        # If enabled (coeff is not None), n_action_steps MUST be 1.
        
        # Enable Temporal Ensembling manully:
        self.policy.config.temporal_ensemble_coeff = 0.04 
        self.policy.config.n_action_steps = 1
        
        # ACT uses chunk_size - 1 for ensembling usually? or 
        # Check chunk size. If chunk size is 100, and n_action_steps is 11, it might cause mismatch if not divisible?
        # ACT Policy implementation usually expects n_action_steps to be 1 when temporal_ensemble is used?
        # Ah wait, for TemporalEnsembler, the buffer size must match the policy output chunk size.
        
        # If the attribute 'temporal_ensembler' is missing, we must initialize it manually
        if not hasattr(self.policy, 'temporal_ensembler') or self.policy.temporal_ensembler is None:
             self.get_logger().info("Initializing Temporal Ensembler manually...")
             # In newer LeRobot versions, the class name might be ACTTemporalEnsembler
             try:
                 from lerobot.policies.act.modeling_act import ACTTemporalEnsembler as TemporalEnsembler
             except ImportError:
                 from lerobot.policies.act.modeling_act import TemporalEnsembler
             
             # ACTConfig usually has 'dim_model' or check output_shapes
             # The action dimension is in config.output_features["action"]["shape"][0] 
             if hasattr(self.policy.config, 'output_features'):
                 # It's likely a PolicyFeature object here, not dict.
                 # Let's try to access .shape directly if it's an object
                 feat = self.policy.config.output_features["action"]
                 if hasattr(feat, 'shape'):
                     action_dim = feat.shape[0]
                 else:
                     action_dim = feat.shape[0] 
             elif hasattr(self.policy.config, 'action_dim'):
                 action_dim = self.policy.config.action_dim
             else:
                 # Fallback, assuming 12 based on known setup if all else fails
                 action_dim = 12 
                 self.get_logger().warning(f"Could not find action_dim in config, using default: {action_dim}")

             # CORRECT INIT: ACTTemporalEnsembler(coeff, chunk_size)
             # We should NOT use action_dim as the second argument if the class is ACTTemporalEnsembler
             try:
                 from lerobot.policies.act.modeling_act import ACTTemporalEnsembler as TemporalEnsembler
                 self.policy.temporal_ensembler = TemporalEnsembler(self.policy.config.temporal_ensemble_coeff, self.policy.config.chunk_size)
             except ImportError:
                 from lerobot.policies.act.modeling_act import TemporalEnsembler
                 # Old TemporalEnsembler might have different signature, but let's assume valid one
                 # If it takes (chunk_size, dim), we need to be careful.
                 # But in this workspace, we saw ACTTemporalEnsembler.
                 # Let's check signatures (we can't easily at runtime, but we read the file earlier).
                 # The file had class ACTTemporalEnsembler: def __init__(self, temporal_ensemble_coeff: float, chunk_size: int)
                 self.policy.temporal_ensembler = TemporalEnsembler(self.policy.config.temporal_ensemble_coeff, self.policy.config.chunk_size)
        
        # 2. Action Steps
        # ...
        
        self.policy.eval()
        
        # --- Validate Model Output Shape & Fix Ensembler ---
        try:
            self.get_logger().info("Starting Model Output Shape Validation...")
            
            # Prepare dummy batch based on config inputs
            dummy_batch = {}
            for key, feat in self.policy.config.input_features.items():
                if isinstance(feat, dict):
                    f_shape = feat.get("shape")
                elif hasattr(feat, 'shape'):
                     f_shape = feat.shape
                else: 
                     f_shape = getattr(feat, 'shape', [3, 480, 640])
                
                # Check shape dimensions to decide tensor type
                if len(f_shape) == 3: # Image [C, H, W]
                    # Ensure float32 for images
                    dummy_batch[key] = torch.zeros([1] + list(f_shape), dtype=torch.float32, device=self.device)
                elif len(f_shape) == 1: # State [D]
                    dummy_batch[key] = torch.zeros([1] + list(f_shape), dtype=torch.float32, device=self.device)
            
            # Run dummy inference
            self.get_logger().info("Running dummy inference...")
            with torch.no_grad():
                dummy_out = self.policy.predict_action_chunk(dummy_batch)
            
            real_chunk_size = dummy_out.shape[1]
            real_action_dim = dummy_out.shape[2]
            
            self.get_logger().info(f"Model Inference Output: {dummy_out.shape} (Chunk: {real_chunk_size}, Dim: {real_action_dim})")
            
            # Check if Ensembler matches
            ensembler_chunk = self.policy.config.chunk_size
            if hasattr(self.policy, 'temporal_ensembler') and self.policy.temporal_ensembler is not None:
                ensembler_chunk = self.policy.temporal_ensembler.chunk_size
                self.get_logger().info(f"Existing Ensembler Chunk Size: {ensembler_chunk}")
            
            # Manually ensure correct initialization if mismatch
            if real_chunk_size != ensembler_chunk:
                self.get_logger().warning(f"Length Mismatch! Model outputs {real_chunk_size}, Ensembler expects {ensembler_chunk}")
                self.get_logger().info(f"Ensuring Temporal Ensembler is initialized with Chunk={real_chunk_size}")

                try:
                    from lerobot.policies.act.modeling_act import ACTTemporalEnsembler as TemporalEnsemblerType
                except ImportError:
                     from lerobot.policies.act.modeling_act import TemporalEnsembler as TemporalEnsemblerType
                
                # Use correct arguments: (coeff, chunk_size)
                # NOT (chunk_size, action_dim)
                coeff = self.policy.config.temporal_ensemble_coeff if self.policy.config.temporal_ensemble_coeff else 0.01
                self.policy.temporal_ensembler = TemporalEnsemblerType(coeff, real_chunk_size)
                self.policy.config.chunk_size = real_chunk_size
                self.get_logger().info("TemporalEnsembler reset done with CORRECT arguments.")
            else:
                self.get_logger().info("Validation Passed: Chunk sizes match.")

        except Exception as e:
            self.get_logger().error(f"Validation CRASHED: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

        self.get_logger().info("Policy loaded successfully!")
        
        # --- DEBUG: Verify Config and Temporal Ensembler ---
        self.get_logger().info(f"DEBUG: Config Chunk Size: {self.policy.config.chunk_size}")
        self.get_logger().info(f"DEBUG: Config Action Dim: {self.policy.config.action_dim if hasattr(self.policy.config, 'action_dim') else 'Unknown'}")
        if hasattr(self.policy, 'temporal_ensembler') and self.policy.temporal_ensembler is not None:
             self.get_logger().info(f"DEBUG: Ensembler Chunk Size: {self.policy.temporal_ensembler.chunk_size}")
             # Check if ensemble weights match chunk size
             self.get_logger().info(f"DEBUG: Ensembler Weights Shape: {self.policy.temporal_ensembler.ensemble_weights.shape}")

        # --- 2. ROS 2 Setup ---
        self.bridge = CvBridge()
        
        # Observation Buffers
        self.latest_images = {} 
        self.latest_state = {} # Should store positions for 'left' and 'right'
        
        # Expected keys from your training config
        # You might need to adjust these keys based on your training config (check info.json in dataset)
        self.camera_keys = {
            "/camera/camera_top/color/image_raw/compressed": "observation.images.top",
            "/camera/camera_far/color/image_raw/compressed": "observation.images.far",
            "/camera/camera_first/color/image_raw/compressed": "observation.images.first"
        }
        
        # Subscribers
        self.create_subscription(CompressedImage, "/camera/camera_top/color/image_raw/compressed", 
                                 lambda msg: self.image_cb(msg, "/camera/camera_top/color/image_raw/compressed"), 1)
        self.create_subscription(CompressedImage, "/camera/camera_far/color/image_raw/compressed", 
                                 lambda msg: self.image_cb(msg, "/camera/camera_far/color/image_raw/compressed"), 1)
        self.create_subscription(CompressedImage, "/camera/camera_first/color/image_raw/compressed", 
                                 lambda msg: self.image_cb(msg, "/camera/camera_first/color/image_raw/compressed"), 1)

        self.create_subscription(JointState, "/left_follower/joint_states", lambda msg: self.joint_cb(msg, "left"), 1)
        self.create_subscription(JointState, "/right_follower/joint_states", lambda msg: self.joint_cb(msg, "right"), 1)

        # Publisher (Action)
        # We control the 'Follower' arms directly based on policy output
        self.action_pub_left = self.create_publisher(JointState, "/left_follower/joint_states_control", 10)
        self.action_pub_right = self.create_publisher(JointState, "/right_follower/joint_states_control", 10)
        
        # Initial Homing Configuration
        self.is_homed = False
        self.homing_start_time = None
        self.homing_complete_time = None
        self.homing_duration = 2.0  # seconds to move to home
        self.homing_wait = 1.0      # seconds to wait after reaching home

        # [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
        self.initial_left = np.array([0.054470, -1.117778, 1.605703, -0.240894, -0.082088, 0.524750], dtype=np.float32)
        self.initial_right = np.array([-0.052935, -1.186057, 1.646363, -0.204069, 0.187959, 0.522448], dtype=np.float32)

        # Control Loop (e.g., 30Hz to match training FPS)
        self.dt = 1.0 / 30.0
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("LeRobot Bridge Node Started.")

    def image_cb(self, msg, topic):
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # BGR
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)   # RGB
        
        # LeRobot expects [C, H, W] tensors, normalized [0,1] floating point usually handled by policy preprocessor?
        # Actually, the policy preprocessor usually handles resizing and normalization if configured.
        # But we need to pass torch tensors.
        
        # Check if resize is needed (based on your training config)
        # If you trained with default act, it might expect specific resolution like 480x640 or what you set in config.
        # For now, we assume the preprocessor handles it, but we need to provide the dict.
        
        key = self.camera_keys[topic]
        self.latest_images[key] = img

    def joint_cb(self, msg, side):
        # Assuming 6 joints per arm
        # We need to concatenate left and right to match "observation.state"
        if len(msg.position) >= 6:
            self.latest_state[side] = np.array(msg.position[:6], dtype=np.float32)

    def control_loop(self):
        # 1. Check if we have all data
        if "left" not in self.latest_state or "right" not in self.latest_state:
            return # Waiting for joints

        # --- Homing Logic ---
        if not self.is_homed:
            current_left = self.latest_state["left"]
            current_right = self.latest_state["right"]
            
            # Start Homing Timer
            if self.homing_start_time is None:
                self.homing_start_time = time.time()
                self.start_left = current_left.copy()
                self.start_right = current_right.copy()
                self.get_logger().info(f"Starting homing sequence over {self.homing_duration} seconds...")

            # Calculate Progress
            elapsed = time.time() - self.homing_start_time
            alpha = np.clip(elapsed / self.homing_duration, 0.0, 1.0)
            
            # Interpolate Position
            cmd_left = self.start_left * (1 - alpha) + self.initial_left * alpha
            cmd_right = self.start_right * (1 - alpha) + self.initial_right * alpha
            
            self.publish_action(cmd_left, self.action_pub_left)
            self.publish_action(cmd_right, self.action_pub_right)

            # Check Completion
            if alpha >= 1.0:
                if self.homing_complete_time is None:
                    self.homing_complete_time = time.time()
                    self.get_logger().info(f"Reached Home. Waiting {self.homing_wait} seconds before inference...")
                
                wait_elapsed = time.time() - self.homing_complete_time
                if wait_elapsed >= self.homing_wait:
                    self.get_logger().info("Homing and Wait Complete. Starting Policy Inference.")
                    self.is_homed = True
            
            return

        for key in self.camera_keys.values():
            if key not in self.latest_images:
                return # Waiting for images

        # 2. Prepare Observation Dictionary
        # Concatenate state: [left_0...left_5, right_0...right_5] -> 12 dims
        try:
            state_vec = np.concatenate([self.latest_state["left"], self.latest_state["right"]])
            
            # Construct observation dictionary
            # Convert to Torch Tensors and move to device
            state_tensor = torch.from_numpy(state_vec).float().to(self.device).unsqueeze(0) # [B, D]
            
            # Images need to be [B, C, H, W] and float normalized (usually) 
            # or at least [B, C, H, W] if the model handles normalization.
            # However, looking at LeRobot datasets, they often store uint8. 
            # Let's assume the policy expects float [0,1] or uint8 depending on config.
            # BUT standard PyTorch convention is Channel First.
            
            def to_tensor(img_np):
                # H, W, C -> B, C, H, W
                tensor = torch.from_numpy(img_np).permute(2, 0, 1).float() / 255.0  # [0, 1] range float32
                return tensor.unsqueeze(0).to(self.device)

            # Debug: Print raw stats of input images
            if not hasattr(self, '_img_stats_printed'):
                top_img = to_tensor(self.latest_images["observation.images.top"])
                self.get_logger().info(f"Top Image Stats | Min: {top_img.min():.3f}, Max: {top_img.max():.3f}, Mean: {top_img.mean():.3f}")
                self._img_stats_printed = True
            
            observation = {
                "observation.state": state_tensor,
                "observation.images.top": to_tensor(self.latest_images["observation.images.top"]),
                "observation.images.far": to_tensor(self.latest_images["observation.images.far"]),
                "observation.images.first": to_tensor(self.latest_images["observation.images.first"]),
            }
            
            # Apply Normalization
            observation = self.preprocessor(observation)
            
            # 3. Inference
            with torch.no_grad():
                # Debugging shapes before inference
                # self.get_logger().info(f"Obs State: {observation['observation.state'].shape}")
                
                # We need to manually call predict_action_chunk first to debug size
                actions = self.policy.predict_action_chunk(observation)
                
                # Check for inconsistent size (e.g. 12 instead of 100)
                if not hasattr(self, '_loop_debug_printed'):
                   block_size = actions.shape[1]
                   self.get_logger().info(f"Loop Inference Output Shape: {actions.shape} (Chunk: {block_size})")
                   
                   ens_chunk = self.policy.temporal_ensembler.chunk_size
                   self.get_logger().info(f"Ensembler Chunk Size is: {ens_chunk}")
                   
                   if block_size != ens_chunk:
                       self.get_logger().error(f"FATAL: Model output {block_size} != Ensembler {ens_chunk}")
                       # Force re-init ONLY if we are sure
                       # Fix arguments here too (coeff, chunk)
                       try:
                           from lerobot.policies.act.modeling_act import ACTTemporalEnsembler as TE
                       except:
                           from lerobot.policies.act.modeling_act import TemporalEnsembler as TE
                       self.policy.temporal_ensembler = TE(0.01, block_size)
                       self.policy.config.chunk_size = block_size
                       self.get_logger().info("Runtime Fix Applied!")
                   self._loop_debug_printed = True

                # Proceed with Ensembler
                if self.policy.temporal_ensembler is not None:
                     action_dict = self.policy.temporal_ensembler.update(actions)
                else:
                     action_dict = actions[:, 0] # Fallback if no ensembler
                
            # 4. Extract Action
            # The output is usually unnormalized action
            # Shape is [B, Action_Dim] -> [1, 12]
            action_raw = action_dict.squeeze(0).cpu().numpy()
            
            # 5. Publish
            # Split into left (0-5) and right (6-11)
            # Apply Un-normalization
            action_dict = {"action": action_dict} # Wrap for postprocessor
            action_dict = self.postprocessor(action_dict)
            action_raw = action_dict["action"].squeeze(0).cpu().numpy()
            
            # Debug: Print raw action values to see if they change
            # self.get_logger().info(f"Action: {action_raw[:3]}") # Print first 3 joints
            
            # Debug: Compare current state vs action
            current_left = self.latest_state["left"]
            diff = np.abs(current_left - action_raw[:6])
            # self.get_logger().info(f"\nCurrent Left: {current_left[:3]}")  
            # self.get_logger().info(f"Target Left : {action_raw[:3]}")
            # self.get_logger().info(f"Max Diff    : {diff.max():.4f}")

            left_action = action_raw[:6]
            right_action = action_raw[6:]
            
            self.publish_action(left_action, self.action_pub_left)
            self.publish_action(right_action, self.action_pub_right)
            
        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")

    def publish_action(self, joints, publisher):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # You might need to fill in joint names if your controller requires them
        # msg.name = [...] 
        msg.position = joints.tolist()
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LeRobotROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
