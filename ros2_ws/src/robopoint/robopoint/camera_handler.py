import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

class CameraHandler:
    def __init__(self, node: Node, camera_ns: str):
        self.node = node
        self.bridge = CvBridge()
        
        self.cv_image = None
        self.depth_image = None
        self.camera_info = None
        
        # Intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Topics
        color_topic = f'{camera_ns}/color/image_raw'
        depth_topic = f'{camera_ns}/aligned_depth_to_color/image_raw'
        info_topic = f'{camera_ns}/color/camera_info'

        self.node.create_subscription(Image, color_topic, self.color_cb, 10)
        self.node.create_subscription(Image, depth_topic, self.depth_cb, 10)
        self.node.create_subscription(CameraInfo, info_topic, self.info_cb, 10)
        
        self.node.get_logger().info(f"[CameraHandler] Subscribed to {camera_ns}")

    def info_cb(self, msg):
        self.camera_info = msg
        K = np.array(msg.k).reshape(3, 3)
        self.fx = K[0, 0]
        self.fy = K[1, 1]
        self.cx = K[0, 2]
        self.cy = K[1, 2]

    def color_cb(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.node.get_logger().error(f"CvBridge Color Error: {e}")

    def depth_cb(self, msg):
        try:
            # FIX: Use 'passthrough' instead of 'unchanged' for depth images (usually 16UC1)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.node.get_logger().error(f"CvBridge Depth Error: {e}")

    def is_ready(self):
        return (self.cv_image is not None) and \
               (self.depth_image is not None) and \
               (self.fx is not None)
    
    def get_rgb(self):
        return self.cv_image

    def get_depth_val(self, u, v):
        if self.depth_image is None:
            return 0.0
        
        try:
            # Check bounds
            h, w = self.depth_image.shape
            if u < 0 or u >= w or v < 0 or v >= h:
                return 0.0
            
            raw_depth = self.depth_image[v, u]
            # Convert mm to meters if needed (RealSense usually raw=mm)
            if self.depth_image.dtype == np.uint16:
                return float(raw_depth) / 1000.0
            else:
                return float(raw_depth)
        except Exception as e:
            self.node.get_logger().error(f"Get Depth Error: {e}")
            return 0.0
