#!/usr/bin/env python3
import sys
import os
import argparse
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
import rosbag2_py
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

# Add script directory to python path to find robust_pino_kinematics
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from robust_pino_kinematics import RobotKinematics
except ImportError:
    print("Error: Could not import robust_pino_kinematics. Make sure it is in the same directory.")
    sys.exit(1)

def get_urdf_path(side):
    # Depending on where we run this, paths might differ.
    # We assume standard install location or local file.
    # For now, let's look for it in /tmp/ or expected locations
    possible_paths = [
        f"/tmp/koch_{side}.urdf",
        f"koch_{side}.urdf",
        f"../koch_{side}.urdf"
    ]
    for p in possible_paths:
        if os.path.exists(p):
            return p
    print(f"Error: URDF file for {side} arm not found. Please run xacro first or provide path.")
    sys.exit(1)

class BagConverter:
    def __init__(self, input_bag, output_bag):
        self.input_bag = input_bag
        self.output_bag = output_bag
        
        # Load Kinematics (Pinocchio)
        print("Loading Pinocchio Kinematics...")
        try:
            self.kin_left = RobotKinematics(get_urdf_path("left"))
            self.kin_right = RobotKinematics(get_urdf_path("right"))
        except Exception as e:
            print(f"Failed to load Kinematics: {e}")
            sys.exit(1)

        # Mapping for bag joints (from topic) to URDF joints
        # The bag contains 'left_follower_shoulder_pan' etc.
        # The URDF contains 'left_joint1', 'left_joint2' etc.
        # We need to build the vector q in the order Pinocchio expects.
        
        # Map: BagName -> URDFName
        self.bag_to_urdf_map_left = {
            'left_follower_shoulder_pan': 'left_joint1',
            'left_follower_shoulder_lift': 'left_joint2',
            'left_follower_elbow_flex': 'left_joint3',
            'left_follower_wrist_flex': 'left_joint4',
            'left_follower_wrist_roll': 'left_joint5',
            # Gripper is separate usually, but if in chain, ignore or handle
        }
        
        self.bag_to_urdf_map_right = {
            'right_follower_shoulder_pan': 'right_joint1',
            'right_follower_shoulder_lift': 'right_joint2',
            'right_follower_elbow_flex': 'right_joint3',
            'right_follower_wrist_flex': 'right_joint4',
            'right_follower_wrist_roll': 'right_joint5',
        }

    def get_q_for_model(self, joint_map_msg, bag_to_urdf_map, kin_solver):
        # We need to construct q vector of size nq
        # model.names gives the joint names in order
        
        q = np.zeros(kin_solver.model.nq)
        
        # Iterate over all joints in the model
        # model.names includes 'universe' at 0, usually we skip it or handle it
        # But actually Pinocchio expects q to match configuration.
        # Let's see how RobotKinematics does it? 
        # It takes q. If dimensions match.
        
        # We need to map: index in q -> joint name in model -> joint name in bag -> value
        
        for i, name in enumerate(kin_solver.model.names):
            if name == 'universe': continue
            
            # Find which bag key maps to this model name
            bag_key = None
            for b_key, u_name in bag_to_urdf_map.items():
                if u_name == name:
                    bag_key = b_key
                    break
            
            if bag_key and bag_key in joint_map_msg:
                # The index in q is determined by joint_id.
                # joint_id = model.getJointId(name)
                # idx_q = model.joints[joint_id].idx_q
                # But for simple revolute/prismatic, idx_q aligns mostly 
                # Let's rely on robust_pino_kinematics logic if possible?
                # robust_pino doesn't have a name->q mapper. We build it here.
                
                joint_id = kin_solver.model.getJointId(name)
                idx_q = kin_solver.model.joints[joint_id].idx_q
                if 0 <= idx_q < len(q):
                    q[idx_q] = joint_map_msg[bag_key]
                
        return q

    def process_bag(self):
        reader = SequentialReader()
        storage_opts = StorageOptions(uri=self.input_bag, storage_id='sqlite3')
        converter_opts = ConverterOptions('', '')
        reader.open(storage_opts, converter_opts)
        
        writer = SequentialWriter()
        out_storage_opts = StorageOptions(uri=self.output_bag, storage_id='sqlite3')
        writer.open(out_storage_opts, converter_opts)

        # Create custom topics
        topic_l_ee = TopicMetadata(name='/left/ee_pose', type='geometry_msgs/msg/PoseStamped', serialization_format='cdr')
        topic_r_ee = TopicMetadata(name='/right/ee_pose', type='geometry_msgs/msg/PoseStamped', serialization_format='cdr')
        writer.create_topic(topic_l_ee)
        writer.create_topic(topic_r_ee)
        
        # Copy all input topics definitions
        typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}
        for t in reader.get_all_topics_and_types():
            writer.create_topic(t)
            
        count = 0
        
        while reader.has_next():
            (topic, data, t_stamp) = reader.read_next()
            
            # Write original data
            writer.write(topic, data, t_stamp)
            
            # Process for EE Pose generation if it's a JointState
            if 'joint_states' in topic and typemap[topic] == 'sensor_msgs/msg/JointState':
                msg = deserialize_message(data, JointState)
                name_map = dict(zip(msg.name, msg.position))
                
                # Check for Left Arm Joints
                if any(k in name_map for k in self.bag_to_urdf_map_left.keys()):
                    q_left = self.get_q_for_model(name_map, self.bag_to_urdf_map_left, self.kin_left)
                    pos, rot = self.kin_left.forward_kinematics(q_left)
                    
                    p_msg = PoseStamped()
                    p_msg.header.stamp = msg.header.stamp
                    p_msg.header.frame_id = "world" # Pinocchio root
                    p_msg.pose.position.x = float(pos[0])
                    p_msg.pose.position.y = float(pos[1])
                    p_msg.pose.position.z = float(pos[2])
                    p_msg.pose.orientation.x = float(rot[0])
                    p_msg.pose.orientation.y = float(rot[1])
                    p_msg.pose.orientation.z = float(rot[2])
                    p_msg.pose.orientation.w = float(rot[3])
                    
                    writer.write('/left/ee_pose', serialize_message(p_msg), t_stamp)

                # Check for Right Arm Joints
                if any(k in name_map for k in self.bag_to_urdf_map_right.keys()):
                    q_right = self.get_q_for_model(name_map, self.bag_to_urdf_map_right, self.kin_right)
                    pos, rot = self.kin_right.forward_kinematics(q_right)
                    
                    p_msg = PoseStamped()
                    p_msg.header.stamp = msg.header.stamp
                    p_msg.header.frame_id = "world"
                    p_msg.pose.position.x = float(pos[0])
                    p_msg.pose.position.y = float(pos[1])
                    p_msg.pose.position.z = float(pos[2])
                    p_msg.pose.orientation.x = float(rot[0])
                    p_msg.pose.orientation.y = float(rot[1])
                    p_msg.pose.orientation.z = float(rot[2])
                    p_msg.pose.orientation.w = float(rot[3])
                    
                    writer.write('/right/ee_pose', serialize_message(p_msg), t_stamp)
            
            count += 1
            if count % 1000 == 0:
                print(f"Processed {count} messages...", end='\r')
                
        print(f"\nDone. Output bag: {self.output_bag}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    args = parser.parse_args()
    
    if os.path.exists(args.output_bag):
        import shutil
        shutil.rmtree(args.output_bag)
        
    converter = BagConverter(args.input_bag, args.output_bag)
    converter.process_bag()
