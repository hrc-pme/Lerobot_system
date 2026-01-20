#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class RealRobotRemapper(Node):
    def __init__(self):
        super().__init__('real_robot_remapper')
        
        # 參數: 來源 topic
        self.declare_parameter('source_topic', '/right_follower/joint_states')
        self.source_topic = self.get_parameter('source_topic').value

        # 映射表: Key -> (URDF Joint Name, Direction, Offset)
        # 用來修正實體馬達零點與模擬模型零點的差異
        # 公式: Final = (Raw * Direction) + Offset
        # 常見 Offset: 90度 = 1.57, -90度 = -1.57
        
        self.mapping = {
            'shoulder_pan':  ('joint1', -1, 0.0),
            'shoulder_lift': ('joint2', -1, 1.57), 
            'elbow_flex':    ('joint3', 1, 1.57),
            'wrist_flex':    ('joint4', -1, 0.0),
            'wrist_roll':    ('joint5', 1, 0.0),
            'gripper':       ('joint_gripper', 1, 0.5)
        }

        # 參數: 輸出 topic (預設為 'joint_states'，可被 namespace 影響)
        self.declare_parameter('target_topic', 'joint_states')
        target_topic = self.get_parameter('target_topic').value
        
        # 參數: 目標前綴 (Target Prefix)
        # 如果 URDF 中的關節名稱有前綴 (如 left_joint1)，這裡就要設定 'left_'
        self.declare_parameter('target_prefix', '')
        self.target_prefix = self.get_parameter('target_prefix').value

        self.sub = self.create_subscription(
            JointState,
            self.source_topic,
            self.listener_callback,
            10
        )
        self.pub = self.create_publisher(JointState, target_topic, 10)
        self.get_logger().info(f"Remapper started: {self.source_topic} -> {target_topic} (Prefix: {self.target_prefix})")

    def listener_callback(self, msg):
        new_msg = JointState()
        new_msg.header = msg.header
        
        # 除錯: 確認是否有收到訊息 (只顯示一次)
        if not hasattr(self, '_logged_once'):
            self.get_logger().info(f"Received data from {self.source_topic}. Joints: {msg.name}")
            self._logged_once = True

        # 因為 msg.velocity 與 msg.effort 可能是空的，直接 zip 會導致 loop 不執行
        # 改用 enumerate msg.name
        for i, name in enumerate(msg.name):
            urdf_name = None
            direction = 1
            offset = 0.0
            
            # 比對結尾 (Suffix Matching)
            for key, (target, sign, off) in self.mapping.items():
                if name.endswith(key):
                    urdf_name = self.target_prefix + target
                    direction = sign
                    offset = off
                    break
            
            if urdf_name:
                new_msg.name.append(urdf_name)
                
                # 安全地添加數值並應用方向與偏移
                if i < len(msg.position):
                    val = msg.position[i]
                    new_msg.position.append((val * direction) + offset)
                
                if msg.velocity and i < len(msg.velocity):
                    new_msg.velocity.append(msg.velocity[i] * direction)
                
                if msg.effort and i < len(msg.effort):
                    new_msg.effort.append(msg.effort[i] * direction)
        
        if new_msg.name:
            self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealRobotRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
