#!/usr/bin/env python3
import sys
import os
import argparse
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import rosbag2_py
from rclpy.serialization import deserialize_message

# Add script directory to python path to find robust_pino_kinematics
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from robust_pino_kinematics import RobotKinematics

def get_urdf_path(side):
    path = f"/tmp/koch_{side}.urdf"
    if not os.path.exists(path):
        # Fallback or error
        pass 
    return path

class RobustIKReplayer(Node):
    def __init__(self, bag_path):
        super().__init__('robust_ik_replayer')
        self.bag_path = bag_path
        
        # Initialize Kinematics
        print("Initializing Robust Pinocchio Kinematics...")
        self.kin_left = RobotKinematics(get_urdf_path("left"))
        self.kin_right = RobotKinematics(get_urdf_path("right"))
        
        # Publishers
        # MUST publish to _control topic to drive the robot, otherwise we are just spoofing state
        self.pub_left = self.create_publisher(JointState, '/left_follower/joint_states_control', 10)
        self.pub_right = self.create_publisher(JointState, '/right_follower/joint_states_control', 10)
        
        # Initial State (Warm Start)
        self.q_left = np.zeros(self.kin_left.dof)
        self.q_right = np.zeros(self.kin_right.dof)
        
        # Flags for initialization
        self.left_initialized = False
        self.right_initialized = False
        
        # Gripper states
        self.gripper_left = 0.0
        self.gripper_right = 0.0
        
        # Subscribers for current state
        self.create_subscription(JointState, '/left_follower/joint_states', self.left_state_cb, 10)
        self.create_subscription(JointState, '/right_follower/joint_states', self.right_state_cb, 10)
        
        # Mappings (URDF -> Robot Topic Names)
        # We need to map the output of IK (which corresponds to URDF joints) to the names expected by the robot driver
        # The URDF usually has: joint1, joint2... joint_gripper
        # The driver expects: shoulder_pan, shoulder_lift... gripper
        
        # Note: Pinocchio model.names gives the joint names in order.
        # We should use that to build the message.
        self.joint_names_left = [n for n in self.kin_left.model.names if n != 'universe']
        self.joint_names_right = [n for n in self.kin_right.model.names if n != 'universe']
        
        # We need a map from Pinocchio names (URDF names) to Robot Driver names
        self.remap_left = {
            'left_joint1': 'left_follower_shoulder_pan',
            'left_joint2': 'left_follower_shoulder_lift',
            'left_joint3': 'left_follower_elbow_flex',
            'left_joint4': 'left_follower_wrist_flex',
            'left_joint5': 'left_follower_wrist_roll',
            'left_joint_gripper': 'left_follower_gripper'
        }
        self.remap_right = {
            'right_joint1': 'right_follower_shoulder_pan',
            'right_joint2': 'right_follower_shoulder_lift',
            'right_joint3': 'right_follower_elbow_flex',
            'right_joint4': 'right_follower_wrist_flex',
            'right_joint5': 'right_follower_wrist_roll',
            'right_joint_gripper': 'right_follower_gripper'
        }

    def left_state_cb(self, msg):
        if not self.left_initialized:
            self.q_left = self.extract_joints(msg, self.remap_left, self.kin_left.model)
            self.left_initialized = True
            self.get_logger().info(f"Initialized Left Arm: {self.q_left}")

    def right_state_cb(self, msg):
        if not self.right_initialized:
            self.q_right = self.extract_joints(msg, self.remap_right, self.kin_right.model)
            self.right_initialized = True
            self.get_logger().info(f"Initialized Right Arm: {self.q_right}")

    def extract_joints(self, msg, remap, model):
        # Reverse map: Robot Name -> URDF Name
        # Actually remap is URDF -> Robot.
        # msg has Robot Names.
        # model has URDF names.
        
        msg_map = dict(zip(msg.name, msg.position))
        q = []
        
        # Iterate over model joints (skipping universe)
        # model.names includes 'universe' at index 0 probably? 
        # pinocchio q vector corresponds to model.joints.
        # model.names is list of joint names.
        
        for name in model.names:
            if name == 'universe': continue
            
            robot_name = remap.get(name, name)
            val = msg_map.get(robot_name, 0.0)
            q.append(val)
            
        return np.array(q)

    def publish_command(self, publisher, q, remap, model, gripper_val):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        names = []
        positions = []
        
        # Iterate model joints to preserve order of q
        idx = 0
        for name in model.names:
            if name == 'universe': continue
            
            robot_name = remap.get(name, name)
            names.append(robot_name)
            
            # If it is gripper, use the recorded one
            if 'gripper' in name:
                positions.append(gripper_val)
            else:
                positions.append(q[idx])
            
            idx += 1
            
        msg.name = names
        msg.position = positions
        publisher.publish(msg)

    def run(self):
        # Wait for initialization
        print("Waiting for robot state...")
        while rclpy.ok() and (not self.left_initialized or not self.right_initialized):
            rclpy.spin_once(self, timeout_sec=0.1)
            
        print("Robot initialized. Starting replay...")
        
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)
        
        topics_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
        
        start_time_bag = None
        start_time_real = time.time()
        
        while reader.has_next() and rclpy.ok():
            (topic, data, t_ns) = reader.read_next()
            t_sec = t_ns / 1e9
            
            if start_time_bag is None:
                start_time_bag = t_sec
                start_time_real = time.time()
                
            # Sync
            elapsed_bag = t_sec - start_time_bag
            elapsed_real = time.time() - start_time_real
            
            if elapsed_bag > elapsed_real:
                time.sleep(elapsed_bag - elapsed_real)
                
            msg_type = topics_map.get(topic)
            
            if msg_type == 'geometry_msgs/msg/PoseStamped':
                msg = deserialize_message(data, PoseStamped)
                pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
                # ROS Quaternion is x,y,z,w
                quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
                
                if '/left/' in topic:
                    # Solve IK
                    # Use previous q as seed
                    q_new = self.kin_left.inverse_kinematics_5dof(pos, quat, self.q_left)
                    self.q_left = q_new
                    self.publish_command(self.pub_left, q_new, self.remap_left, self.kin_left.model, self.gripper_left)
                    # Simple progress logging
                    print(f"L: Replayed {topic}", end='\r')
                    
                elif '/right/' in topic:
                    q_new = self.kin_right.inverse_kinematics_5dof(pos, quat, self.q_right)
                    self.q_right = q_new
                    self.publish_command(self.pub_right, q_new, self.remap_right, self.kin_right.model, self.gripper_right)
                    # print(f"R: Replayed {topic}", end='\r')
            
            elif msg_type == 'sensor_msgs/msg/JointState':
                msg = deserialize_message(data, JointState)
                msg_map = dict(zip(msg.name, msg.position))
                
                # Extract Gripper Data from original JointStates
                # Check Left Gripper
                if 'left_follower_gripper' in msg_map:
                    self.gripper_left = msg_map['left_follower_gripper']
                    
                # Check Right Gripper
                if 'right_follower_gripper' in msg_map:
                    self.gripper_right = msg_map['right_follower_gripper']

            elif msg_type == 'std_msgs/msg/Float32':
                msg = deserialize_message(data, Float32)
                if '/left/' in topic:
                    self.gripper_left = msg.data
                elif '/right/' in topic:
                    self.gripper_right = msg.data

def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", help="Path to bag file with EE poses")
    args = parser.parse_args()
    
    node = RobustIKReplayer(args.bag_path)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
