#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import math

class MockCrossRobot(Node):
    def __init__(self):
        super().__init__('mock_cross_robot')
        
        # --- 1. Koch 虛擬指令發布者 (模擬你的 LeRobot Model 輸出) ---
        self.koch_left_pub = self.create_publisher(JointState, "/left_follower/joint_states_control", 10)
        self.koch_right_pub = self.create_publisher(JointState, "/right_follower/joint_states_control", 10)
        
        # --- 2. OpenArm 虛擬硬體回放 (模擬真實 OpenArm 接到指令後到達，回傳狀態) ---
        self.create_subscription(JointState, "/openarm/joint_commands", self.openarm_cmd_cb, 10)
        self.openarm_state_pub = self.create_publisher(JointState, "/openarm/joint_states", 10)
        # 多加一個發布給 /joint_states 讓 robot_state_publisher 和 RViz 能夠更新畫面
        self.rviz_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        # 計時器：產生連續的正弦波動作 (Sine wave)
        self.timer = self.create_timer(0.05, self.publish_koch_dummy_cmd) # 20Hz
        self.t = 0.0

        self.get_logger().info("Mock Cross-Robot Simulator Started.")
        self.get_logger().info("1. Generating sine-wave movements for Koch...")
        self.get_logger().info("2. Looping back OpenArm commands to states...")

    def publish_koch_dummy_cmd(self):
        """產生左右手平滑的假訊號 (例如：抬手、轉向、揮動)"""
        self.t += 0.05
        
        # 假設 Koch 是 6 個自由度 (5旋轉 + 1夾爪)
        # 用正弦波產生擺動的 Joint 角度 [-1.0 到 1.0 之間]
        wave1 = math.sin(self.t) * 0.5
        wave2 = math.cos(self.t * 0.5) * 0.5
        
        # 夾爪類比開合 (0 ~ 1 之間循環)
        gripper_wave = (math.sin(self.t * 2) + 1.0) / 2.0 
        
        # Left Command
        msg_l = JointState()
        msg_l.header.stamp = self.get_clock().now().to_msg()
        msg_l.name = ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint_gripper"]
        msg_l.position = [wave1, wave2, -1.0 + wave1*0.5, 0.0, 0.0, gripper_wave]
        
        # Right Command (可以故意做稍微不同的動作)
        msg_r = JointState()
        msg_r.header.stamp = self.get_clock().now().to_msg()
        msg_r.name = ["right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint_gripper"]
        msg_r.position = [-wave1, wave2, -1.0 + wave2*0.5, 0.0, 0.0, 1.0 - gripper_wave]
        
        self.koch_left_pub.publish(msg_l)
        self.koch_right_pub.publish(msg_r)

        # 把左右手的虛擬角度也順便發佈給 RViz，讓 RViz 裡的 Koch 模型也能動起來
        combined_viz_msg = JointState()
        combined_viz_msg.header.stamp = msg_l.header.stamp
        combined_viz_msg.name = msg_l.name + msg_r.name
        combined_viz_msg.position = msg_l.position + msg_r.position
        self.rviz_state_pub.publish(combined_viz_msg)

    def openarm_cmd_cb(self, msg):
        """假裝：OpenArm 真機接收到指令後，馬達跑到該位置了，所以回傳給 /openarm/joint_states"""
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = msg.name
        state_msg.position = msg.position
        self.openarm_state_pub.publish(state_msg)
        self.rviz_state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockCrossRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
