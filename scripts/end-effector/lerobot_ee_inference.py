#!/usr/bin/env python3
"""
ROS2 Inference Script for End-Effector Space Policy
1. Subscribes to JointStates + Images
2. Computes FK (Joints -> EE Pose)
3. Runs Policy (EE Pose + Images -> Target EE Pose)
4. Computes IK (Target EE -> Target Joints)
5. Publishes Target Joints
"""

import time
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, CompressedImage
import cv2
import sys
import os

# --- Configuration ---
# Update this with your actual EE-trained model path
CHECKPOINT_PATH = "/home/hrc/Lerobot_system/outputs_nano/koch_bi_wipe_water_tissue_ee/checkpoints/040000/pretrained_model"

# Add paths for robust_pino_kinematics
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "end-effector"))

try:
    from robust_pino_kinematics import RobotKinematics
except ImportError:
    print("Error: Could not import robust_pino_kinematics. Make sure scripts/end-effector/robust_pino_kinematics.py exists.")
    sys.exit(1)

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor import PolicyProcessorPipeline

class LeRobotEEInferenceNode(Node):
    def __init__(self):
        super().__init__('lerobot_ee_inference_node')
        
        # --- 1. Load Policy ---
        self.get_logger().info(f"Loading EE policy from {CHECKPOINT_PATH}...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.policy = ACTPolicy.from_pretrained(CHECKPOINT_PATH).to(self.device)
        self.preprocessor = PolicyProcessorPipeline.from_pretrained(CHECKPOINT_PATH, config_filename="policy_preprocessor.json")
        self.postprocessor = PolicyProcessorPipeline.from_pretrained(CHECKPOINT_PATH, config_filename="policy_postprocessor.json")
        
        # Configure Temporal Ensembling
        self.policy.config.temporal_ensemble_coeff = 0.01 
        self.policy.config.n_action_steps = 1
        
        # Initialize Temporal Ensembler if missing
        if not hasattr(self.policy, 'temporal_ensembler') or self.policy.temporal_ensembler is None:
             chunk_size = self.policy.config.chunk_size
             try:
                 from lerobot.policies.act.modeling_act import ACTTemporalEnsembler as TemporalEnsembler
             except ImportError:
                 from lerobot.policies.act.modeling_act import TemporalEnsembler
             self.policy.temporal_ensembler = TemporalEnsembler(self.policy.config.temporal_ensemble_coeff, chunk_size)
        
        self.policy.eval()
        self.get_logger().info("Policy loaded successfully!")

        # --- 2. Initialize Kinematics ---
        urdf_l = "/tmp/koch_left.urdf"
        urdf_r = "/tmp/koch_right.urdf"
        
        if not os.path.exists(urdf_l) or not os.path.exists(urdf_r):
            self.get_logger().error("URDF files not found in /tmp. Please run xacro commands first.")
            # sys.exit(1) # Don't crash, let user fix it

        self.kin_left = RobotKinematics(urdf_l)
        self.kin_right = RobotKinematics(urdf_r)
        
        # URDF Joint Mapping (ROS -> URDF)
        self.map_left = {
            'left_follower_shoulder_pan': 'left_joint1',
            'left_follower_shoulder_lift': 'left_joint2',
            'left_follower_elbow_flex': 'left_joint3',
            'left_follower_wrist_flex': 'left_joint4',
            'left_follower_wrist_roll': 'left_joint5',
        }
        self.map_right = {
            'right_follower_shoulder_pan': 'right_joint1',
            'right_follower_shoulder_lift': 'right_joint2',
            'right_follower_elbow_flex': 'right_joint3',
            'right_follower_wrist_flex': 'right_joint4',
            'right_follower_wrist_roll': 'right_joint5',
        }

        # Keep track of current joints as seed for IK
        self.last_q_left = None 
        self.last_q_right = None
        self.last_gripper_left = 0.0
        self.last_gripper_right = 0.0
        
        # --- 3. ROS Setup ---
        self.latest_images = {}
        self.camera_keys = {
            "/camera/camera_top/color/image_raw/compressed": "observation.images.top",
            "/camera/camera_far/color/image_raw/compressed": "observation.images.far",
            "/camera/camera_first/color/image_raw/compressed": "observation.images.first"
        }
        
        # Subscribers
        for topic in self.camera_keys:
            self.create_subscription(CompressedImage, topic, lambda msg, t=topic: self.image_cb(msg, t), 1)

        self.create_subscription(JointState, "/left_follower/joint_states", lambda msg: self.joint_cb(msg, "left"), 1)
        self.create_subscription(JointState, "/right_follower/joint_states", lambda msg: self.joint_cb(msg, "right"), 1)

        # Publishers
        self.pub_left = self.create_publisher(JointState, "/left_follower/joint_states_control", 10)
        self.pub_right = self.create_publisher(JointState, "/right_follower/joint_states_control", 10)
        
        # Control Loop
        self.create_timer(1.0/30.0, self.control_loop)
        
    def get_q_for_model(self, name_map, mapping, kin):
        """Map ROS joint dictionary to Pinocchio q vector"""
        q = np.zeros(kin.model.nq)
        for name in kin.model.names:
            if name == 'universe': continue
            
            # Find ROS name for this URDF joint
            ros_name = None
            for r_name, u_name in mapping.items():
                if u_name == name:
                    ros_name = r_name
                    break
            
            if ros_name and ros_name in name_map:
                joint_id = kin.model.getJointId(name)
                idx_q = kin.model.joints[joint_id].idx_q
                if 0 <= idx_q < len(q):
                    q[idx_q] = name_map[ros_name]
        return q

    def joint_cb(self, msg, side):
        name_map = dict(zip(msg.name, msg.position))
        
        # Determine mapping and kin model
        mapping = self.map_left if side == 'left' else self.map_right
        kin = self.kin_left if side == 'left' else self.kin_right
        
        # Only process if we have relevant joints
        if any(k in name_map for k in mapping):
            q = self.get_q_for_model(name_map, mapping, kin)
            
            # Extract gripper
            gripper_val = 0.0
            prefix = f"{side}_follower"
            possible_names = [f"{prefix}_gripper", "gripper"]
            for gn in possible_names:
                if gn in name_map:
                    gripper_val = name_map[gn]
                    break
            
            # Update internal state
            if side == 'left':
                self.last_q_left = q
                self.last_gripper_left = gripper_val
            else:
                self.last_q_right = q
                self.last_gripper_right = gripper_val

    def image_cb(self, msg, topic):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        key = self.camera_keys[topic]
        self.latest_images[key] = img

    def compute_fk_state(self, q, gripper, kin):
        """Compute 8-dim state [x,y,z, qx,qy,qz,qw, gripper]"""
        pos, rot = kin.forward_kinematics(q) # rot is [x,y,z,w] or similar from pinocchio
        return np.concatenate([pos, rot, [gripper]]).astype(np.float32)

    def control_loop(self):
        # 1. Check Data Availability
        if self.last_q_left is None or self.last_q_right is None:
            return # Waiting for joints
            
        for k in self.camera_keys.values():
            if k not in self.latest_images:
                return # Waiting for images

        try:
            # 2. Compute FK (Observation)
            ee_left = self.compute_fk_state(self.last_q_left, self.last_gripper_left, self.kin_left)
            ee_right = self.compute_fk_state(self.last_q_right, self.last_gripper_right, self.kin_right)
            
            state_vec = np.concatenate([ee_left, ee_right]) # 16 dims
            
            # Prepare Tensor Inputs
            state_tensor = torch.from_numpy(state_vec).float().to(self.device).unsqueeze(0)
            
            def to_tensor(img):
                return torch.from_numpy(img).permute(2, 0, 1).float().div(255.0).unsqueeze(0).to(self.device)
            
            observation = {
                "observation.state": state_tensor,
                "observation.images.top": to_tensor(self.latest_images["observation.images.top"]),
                "observation.images.far": to_tensor(self.latest_images["observation.images.far"]),
                "observation.images.first": to_tensor(self.latest_images["observation.images.first"]), # Matches 'wrist' logic from config
            }
            
            # 3. Policy Inference
            # Normalize
            observation = self.preprocessor(observation)
            
            with torch.no_grad():
                actions = self.policy.predict_action_chunk(observation)
                action_dict = self.policy.temporal_ensembler.update(actions)
            
            # Un-normalize & Extract
            action_dict = self.postprocessor({"action": action_dict})
            # Check dimensions before squeeze
            # If shape is [1, 16], squeeze(0) -> [16]
            # If shape is [1, 100, 16] (horizon), squeeze(0) -> [100, 16]
            action_raw_tensor = action_dict["action"].squeeze(0)
            if action_raw_tensor.dim() > 1:
                # Take the first action in the chunk for immediate execution
                action_raw = action_raw_tensor[0].cpu().numpy()
            else:
                action_raw = action_raw_tensor.cpu().numpy()
            
            # 4. Process Actions (IK)
            # Left Action
            action_l = action_raw[:8]
            target_pos_l = action_l[:3]
            target_quat_l = action_l[3:7] # [x, y, z, w]
            # Normalize quaternion to ensure valid rotation
            norm_l = np.linalg.norm(target_quat_l)
            if norm_l > 1e-6:
                target_quat_l /= norm_l
            else:
                target_quat_l = np.array([0.0, 0.0, 0.0, 1.0]) # Fallback identity
            
            # Additional Check: Dot product with previous quaternion to prevent flipping
            # oMf = kin.data.oMf[kin.tip_frame_id] -> current quat
            # But calculating FK every time is expensive, let's trust IK logic or re-compute current
            
            target_grip_l = action_l[7]
            
            # Solve IK Left
            q_sol_l = self.kin_left.inverse_kinematics_5dof(target_pos_l, target_quat_l, self.last_q_left)
            
            # Right Action
            action_r = action_raw[8:]
            target_pos_r = action_r[:3]
            target_quat_r = action_r[3:7]
            # Normalize right quaternion
            norm_r = np.linalg.norm(target_quat_r)
            if norm_r > 1e-6:
                target_quat_r /= norm_r
            else:
                target_quat_r = np.array([0.0, 0.0, 0.0, 1.0])
                
            target_grip_r = action_r[7]
            
            # Solve IK Right
            q_sol_r = self.kin_right.inverse_kinematics_5dof(target_pos_r, target_quat_r, self.last_q_right)
            
            # 5. Publish
            self.publish_joints(self.pub_left, q_sol_l, target_grip_l, self.map_left, self.kin_left, "left_follower")
            self.publish_joints(self.pub_right, q_sol_r, target_grip_r, self.map_right, self.kin_right, "right_follower")
            
        except Exception as e:
            self.get_logger().error(f"Inference Loop Error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def publish_joints(self, pub, q_sol, gripper_val, mapping, kin, prefix):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Reconstruct ROS message from Pinocchio q
        # Iterate over mapping to find values
        # This order should match what the driver expects. 
        # Usually drivers need specific order or named joints.
        # Let's use the keys from mapping as names, and find values from q_sol
        
        ordered_names = [
            f"{prefix}_shoulder_pan",
            f"{prefix}_shoulder_lift",
            f"{prefix}_elbow_flex",
            f"{prefix}_wrist_flex",
            f"{prefix}_wrist_roll",
            f"{prefix}_gripper"
        ]
        
        msg.name = ordered_names
        msg.position = []
        
        for name in ordered_names:
            if 'gripper' in name:
                msg.position.append(float(gripper_val))
            else:
                # Find which URDF joint this corresponds to
                urdf_name = mapping.get(name)
                if urdf_name:
                    joint_id = kin.model.getJointId(urdf_name)
                    idx_q = kin.model.joints[joint_id].idx_q
                    msg.position.append(float(q_sol[idx_q]))
                else:
                    self.get_logger().warn(f"Joint {name} not found in mapping, appending 0.0")
                    msg.position.append(0.0)
                    
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LeRobotEEInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
