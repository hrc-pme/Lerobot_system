#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch Teleop Bridge
- 訂閱 Leader JointState
- 轉換關節名稱 (Leader -> Follower)
- 發佈到 Follower Control Topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class KochTeleopBridge(Node):
    def __init__(self):
        super().__init__('koch_teleop_bridge')
        
        # 參數: 支援雙手，使用 parameter array
        # 格式: [leader_left, follower_left, leader_right, follower_right]
        # 但為了簡單，我們動態偵測 topic
        
        # 參數: 設定配對清單 (List of strings)
        # 格式: "leader_name:follower_name"
        # 預設: ["left_leader:left_follower", "right_leader:right_follower"]
        self.declare_parameter('pairs', ["left_leader:left_follower", "right_leader:right_follower"])
        
        pairs_list = self.get_parameter('pairs').value
        self.pairs = {} # {leader_name: follower_name}
        
        for p in pairs_list:
            if ":" in p:
                l, f = p.split(":")
                self.pairs[l] = f
                
                # 建立 Subscription (Leader)
                self.create_subscription(
                    JointState,
                    f'/{l}/joint_states',
                    lambda msg, leader=l, follower=f: self.callback(msg, leader, follower),
                    10
                )
                
                # 建立 Publisher (Follower)
                # 使用 dict 儲存 publisher 以便在 callback 中使用
                if not hasattr(self, 'pubs'): self.pubs = {}
                self.pubs[f] = self.create_publisher(JointState, f'/{f}/joint_states_control', 10)
                
                self.get_logger().info(f'Bridge Pair: /{l}/joint_states -> /{f}/joint_states_control')

    def callback(self, msg, leader_name, follower_name):
        # 建立新的控制訊息
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # 轉換名稱: <leader>_xxx -> <follower>_xxx
        new_names = []
        for name in msg.name:
            if name.startswith(leader_name):
                new_name = name.replace(leader_name, follower_name)
                new_names.append(new_name)
            else:
                new_names.append(f"{follower_name}_{name}")

        cmd.name = new_names
        cmd.position = msg.position
        cmd.velocity = msg.velocity
        cmd.effort = msg.effort
        
        # 根據 follower_name 找到對應的 publisher
        if follower_name in self.pubs:
            self.pubs[follower_name].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = KochTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
