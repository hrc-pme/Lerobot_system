#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import cv2
import time

# --- 模擬 LLM ---
# 在實際應用中，這裡會 Call OpenAI API 或本地 VLM
def mock_vlm_inference(cv_image):
    """
    模擬 VLM 行為。
    這裡我們用一個簡單的色彩遮罩來找 '紅色' 物體代替 LLM。
    回傳: (u, v) 像素座標
    """
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # 紅色範圍
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    
    # 找最大的紅色區塊
    contours, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy
    return None, None

class AutonomousGraspNode(Node):
    def __init__(self):
        super().__init__('autonomous_grasp_node')
        
        # 1. 訂閱相機資訊 (Top Camera)
        # 確保您的 launch 檔有開啟 align_depth=true
        self.camera_ns = '/camera/camera_top'
        self.bridge = CvBridge()
        
        # 訂閱影像
        self.create_subscription(Image, f'{self.camera_ns}/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, f'{self.camera_ns}/aligned_depth_to_color/image_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, f'{self.camera_ns}/color/camera_info', self.info_cb, 10)
        
        # 發布目標點 (給 RViz 看)
        self.target_pub = self.create_publisher(PointStamped, '/grasp_target_point', 10)
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 狀態變數
        self.cv_image = None
        self.depth_image = None
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.get_logger().info("Grasp Node Started. Waiting for images...")

    def info_cb(self, msg):
        self.camera_info = msg
        K = np.array(msg.k).reshape(3, 3)
        self.fx = K[0, 0]
        self.fy = K[1, 1]
        self.cx = K[0, 2]
        self.cy = K[1, 2]

    def color_cb(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='unchanged')

    def trigger_pipeline(self):
        """
        這就是主流程： LLM -> 2D -> 3D -> TF -> MoveIt
        """
        if self.cv_image is None or self.depth_image is None or self.fx is None:
            self.get_logger().warn("Waiting for camera data...")
            return

        # --- STEP 1: LLM (VLM) 識別 ---
        self.get_logger().info("Step 1: Asking VLM 'Where is the object?'...")
        u, v = mock_vlm_inference(self.cv_image)
        
        if u is None:
            self.get_logger().warn("VLM found nothing.")
            return
        
        self.get_logger().info(f"VLM says object is at Pixel: ({u}, {v})")

        # --- STEP 2: 2D 轉 3D (Camera Frame) ---
        depth_val = self.depth_image[v, u] # 注意 numpy 是 [row, col] -> [y, x]
        
        # 單位換算 (mm -> m)
        if self.depth_image.dtype == np.uint16:
            d = float(depth_val) / 1000.0
        else:
            d = float(depth_val)

        if d <= 0:
            self.get_logger().warn("Invalid depth (0) at target point.")
            return

        # De-projection algorithm
        x_cam = (u - self.cx) * d / self.fx
        y_cam = (v - self.cy) * d / self.fy
        z_cam = d
        
        self.get_logger().info(f"Camera Coordinates: x={x_cam:.3f}, y={y_cam:.3f}, z={z_cam:.3f}")

        # --- STEP 3: TF 轉換 (Camera -> World) ---
        pt_cam = PointStamped()
        pt_cam.header.stamp = self.get_clock().now().to_msg()
        # 注意：一定要用 camera info 裡面的 frame_id (通常是 optical frame)
        pt_cam.header.frame_id = self.camera_info.header.frame_id 
        pt_cam.point.x = x_cam
        pt_cam.point.y = y_cam
        pt_cam.point.z = z_cam
        
        try:
            # 查詢轉換到 world
            transform = self.tf_buffer.lookup_transform(
                'world', # Target Frame
                pt_cam.header.frame_id, # Source Frame
                rclpy.time.Time())
            
            pt_world = do_transform_point(pt_cam, transform)
            
            self.get_logger().info(f"Step 3: World Coordinates: x={pt_world.point.x:.3f}, y={pt_world.point.y:.3f}, z={pt_world.point.z:.3f}")
            self.target_pub.publish(pt_world) # 可以在 RViz 看到這個點

            # --- STEP 4: 傳給 MoveIt (示意用) ---
            self.send_to_moveit(pt_world.point.x, pt_world.point.y, pt_world.point.z)

        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")

    def send_to_moveit(self, x, y, z):
        self.get_logger().info(f"Step 4: Sending command to MoveIt -> Go to ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # 這裡需要您的 MoveIt 介面
        # 在 ROS2 Python 中，這通常透過 MoveItPy 或 Action Client 實作
        # 以下為虛擬碼範例：
        
        """
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        # 設定夾爪向下 (這需要根據您的機械手臂具體定義)
        # 例如: 旋轉 180 度繞 X 軸
        target_pose.pose.orientation.w = 0.0
        target_pose.pose.orientation.x = 1.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        
        move_group.set_pose_target(target_pose)
        move_group.plan()
        move_group.execute()
        """

def main():
    rclpy.init()
    node = AutonomousGraspNode()
    
    # 讓程式跑一下接收圖片
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # 這裡我們手動觸發一次識別 (例如每 2 秒一次)
            # 在實際應用中，可能是等待 User 輸入指令才觸發
            # node.trigger_pipeline() 
            
            # 為了測試，您可以取消上面的註解，或者用鍵盤監聽來觸發
            
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
