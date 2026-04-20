#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys

class LiveTeleopBridge(Node):
    def __init__(self):
        super().__init__('live_teleop_bridge')
        
        # 訂閱真實 Koch 手臂發佈的原始 JointState
        # (根據你的實際環境，真實硬體可能是發在 /left_follower/joint_states)
        self.create_subscription(JointState, "/left_follower/joint_states", self.left_cb, 10)
        self.create_subscription(JointState, "/right_follower/joint_states", self.right_cb, 10)
        
        # 發布校正後的 joint state 給 FK (joint_to_ee_converter.py) 進行末端點追蹤
        self.koch_left_ctrl_pub = self.create_publisher(JointState, "/left_follower/joint_states_control", 10)
        self.koch_right_ctrl_pub = self.create_publisher(JointState, "/right_follower/joint_states_control", 10)
        
        # OpenArm 虛擬硬體回放 (假裝真機已跑到目標位置，回傳狀態給 RViz)
        self.create_subscription(JointState, "/openarm/joint_commands", self.openarm_cmd_cb, 10)
        self.openarm_state_pub = self.create_publisher(JointState, "/openarm/joint_states", 10)
        
        # 發布給 RViz 顯示 (純粹視覺化)
        self.rviz_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        self.last_left_msg = None
        self.last_right_msg = None
        
        # 真實硬體的關節可能與 URDF 模型有正負向反轉與偏移(Offset)差異
        # 這裡將實體機訊號映射到標準 URDF 模型
        self.mapping = {
            'shoulder_pan':  ('joint1', -1, 0.0),
            'shoulder_lift': ('joint2', -1, 1.05), 
            'elbow_flex':    ('joint3', 1, 1.57),
            'wrist_flex':    ('joint4', -1, 0.0),
            'wrist_roll':    ('joint5', 1, 0.0),
            'gripper':       ('joint_gripper', 1, 0.5)
        }

        self.get_logger().info("Live Teleop Cross-Robot Bridge Started!")
        self.get_logger().info("Listening to real Koch hardware and mirroring to OpenArm...")

    def process_joint_msg(self, msg, prefix):
        """將真機的裸 Joint 數值，校正為標準模型能夠運算的數值"""
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = []
        new_msg.position = []
        
        for i, name in enumerate(msg.name):
            # 遍歷 mapping 找出對應的名字
            matched = False
            for key, (target_name, direction, offset) in self.mapping.items():
                if key in name or target_name in name:
                    new_msg.name.append(f"{prefix}{target_name}")
                    if i < len(msg.position):
                        val = msg.position[i]
                        new_msg.position.append((val * direction) + offset)
                    matched = True
                    break
            
            # 若無匹配到 mapping，則直接照抄原名稱
            if not matched:
                clean_name = name.replace("left_", "").replace("right_", "")
                new_msg.name.append(f"{prefix}{clean_name}")
                if i < len(msg.position):
                    new_msg.position.append(msg.position[i])
                    
        return new_msg

    def left_cb(self, msg):
        self.last_left_msg = self.process_joint_msg(msg, "left_")
        # 1. 丟給 FK 解算 OpenArm
        self.koch_left_ctrl_pub.publish(self.last_left_msg)
        # 2. 丟給 RViz 顯示
        self.publish_koch_rviz()

    def right_cb(self, msg):
        self.last_right_msg = self.process_joint_msg(msg, "right_")
        # 1. 丟給 FK 解算 OpenArm
        self.koch_right_ctrl_pub.publish(self.last_right_msg)
        # 2. 丟給 RViz 顯示
        self.publish_koch_rviz()

    def publish_koch_rviz(self):
        if not self.last_left_msg or not self.last_right_msg:
            return
            
        combined_viz_msg = JointState()
        combined_viz_msg.header.stamp = self.get_clock().now().to_msg()
        combined_viz_msg.name = list(self.last_left_msg.name) + list(self.last_right_msg.name)
        combined_viz_msg.position = list(self.last_left_msg.position) + list(self.last_right_msg.position)
        self.rviz_state_pub.publish(combined_viz_msg)

    def openarm_cmd_cb(self, msg):
        """假裝 OpenArm 模擬機已經到達指定位址，回傳結果。把運算出的 OpenArm 發布給 RViz。"""
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = msg.name
        state_msg.position = msg.position
        self.openarm_state_pub.publish(state_msg)
        self.rviz_state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LiveTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
