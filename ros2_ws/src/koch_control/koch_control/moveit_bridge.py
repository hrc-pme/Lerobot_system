#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class MoveItBridge(Node):
    def __init__(self):
        super().__init__('moveit_bridge')
        
        # Mapping: Real Robot (Follower) -> MoveIt (URDF)
        # Key: Real Robot Joint Name (from single_follower.yaml + prefix)
        # Value: (URDF Joint Name, Direction, Offset)
        # Match real_robot_remapper.py logic
        self.joint_map = {
            'right_follower_shoulder_pan':  ('joint1', -1, 0.0),
            'right_follower_shoulder_lift': ('joint2', -1, 1.05), # Adjusted by -30 deg (1.57 - 0.52)
            'right_follower_elbow_flex':    ('joint3', 1, 1.57),
            'right_follower_wrist_flex':    ('joint4', -1, 0.0),
            'right_follower_wrist_roll':    ('joint5', 1, 0.0),
            'right_follower_gripper':       ('joint_gripper', 1, 0.5)
        }
        # Reverse map for command forwarding: URDF Name -> (Real Name, Direction, Offset)
        self.urdf_to_real_map = {v[0]: (k, v[1], v[2]) for k, v in self.joint_map.items()}
        
        # 最後一次收到的真實機器人狀態 (用於驗證)
        self.last_real_state = None

        # 1. Subscribe to Real Robot States
        self.sub_real_states = self.create_subscription(
            JointState,
            '/right_follower/joint_states',
            self.cb_real_states,
            10
        )

        # 2. Publish to MoveIt (Standard /joint_states)
        self.pub_moveit_states = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # 3. Subscribe to Fake Joint States (from Mock Hardware / MoveIt)
        # We listen to the fake robot states and mirror them to the real robot.
        self.sub_moveit_cmd = self.create_subscription(
            JointState,
            '/fake_joint_states', 
            self.cb_fake_states,
            10
        )
        
        # 4. Publish Command to Real Robot
        self.pub_real_cmd = self.create_publisher(
            JointState,
            '/right_follower/joint_states_control',
            10
        )

        self.get_logger().info('🌉 MoveIt Bridge Initialized')
        self.get_logger().info(f'   Mapping with Transforms: {self.joint_map}')

    def cb_real_states(self, msg):
        """Forward real robot states to MoveIt with renamed joints AND transforms"""
        self.last_real_state = msg  # 保存最新狀態
        
        new_msg = JointState()
        new_msg.header = msg.header
        
        for i, name in enumerate(msg.name):
            if name in self.joint_map:
                urdf_name, direction, offset = self.joint_map[name]
                new_msg.name.append(urdf_name)
                
                # Apply Transform: MoveIt = Real * Dir + Offset
                raw_pos = msg.position[i]
                new_pos = (raw_pos * direction) + offset
                new_msg.position.append(new_pos)
                
                if msg.velocity and i < len(msg.velocity):
                    new_msg.velocity.append(msg.velocity[i] * direction)
                if msg.effort and i < len(msg.effort):
                    new_msg.effort.append(msg.effort[i] * direction)
        
        if new_msg.name:
            self.pub_moveit_states.publish(new_msg)

    def cb_fake_states(self, msg):
        """Forward Fake Robot states (MoveIt Plan) to Real Robot with Inverse Transform"""
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, name in enumerate(msg.name):
            if name in self.urdf_to_real_map:
                real_full_name, direction, offset = self.urdf_to_real_map[name]
                
                # 移除前綴 (koch_follower_control 期望不帶前綴的名稱)
                clean_name = real_full_name.replace('right_follower_', '')
                cmd_msg.name.append(clean_name)
                
                if i < len(msg.position):
                    # Apply Inverse Transform: Real = (MoveIt - Offset) * Dir
                    moveit_pos = msg.position[i]
                    real_pos = (moveit_pos - offset) * direction
                    cmd_msg.position.append(real_pos)
        
        if cmd_msg.name:
            self.pub_real_cmd.publish(cmd_msg)
            # 只在調試時打印
            # self.get_logger().debug(f'→ Real Robot CMD: {dict(zip(cmd_msg.name, cmd_msg.position))}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveItBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
