#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

# Import our modules
from robopoint.camera_handler import CameraHandler
from robopoint.spatial_calculator import SpatialCalculator
from robopoint.planner import Planner

# OpenCV for VLM simulation
import cv2
import numpy as np

def mock_vlm_inference(cv_image):
    """
    Simulation of VLM: Find red object
    """
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    
    contours, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy
    return None, None

class GraspNode(Node):
    def __init__(self):
        super().__init__('grasp_node')
        
        # 1. Initialize Modules
        self.camera = CameraHandler(self, camera_ns='/camera/camera_top')
        self.calculator = SpatialCalculator(self)
        self.planner = Planner(self)
        
        # 2. Publisher for visualization
        # Change to Marker driven by topic /visualization_marker
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        
        # 3. Timer loop (Trigger logic every 2 seconds)
        self.timer = self.create_timer(2.0, self.pipeline_callback)
        
        self.get_logger().info("--- Autonomous Grasp Node Ready ---")

    def pipeline_callback(self):
        if not self.camera.is_ready():
            self.get_logger().info("Waiting for camera frames...", throttle_duration_sec=2.0)
            return

        # [Phase 1 & 2] Perception & Cognition
        rgb_img = self.camera.get_rgb()
        u, v = mock_vlm_inference(rgb_img)
        
        if u is None:
            self.get_logger().info("No red object found.", throttle_duration_sec=2.0)
            return
            
        self.get_logger().info(f"Target identified at Pixel ({u}, {v})")

        # [Phase 3] Spatial Calculation
        depth_val = self.camera.get_depth_val(u, v)
        if depth_val <= 0:
            self.get_logger().warn("Invalid depth detected.")
            return

        intrinsics = (self.camera.fx, self.camera.fy, self.camera.cx, self.camera.cy)
        frame_id = self.camera.camera_info.header.frame_id # usually camera_top_color_optical_frame

        # 2D -> 3D Camera
        pt_cam = self.calculator.pixel_to_point(u, v, depth_val, intrinsics, frame_id)
        
        # 3D Camera -> 3D World
        pt_world = self.calculator.transform_to_world(pt_cam, target_frame="world")
        
        if pt_world:
            self.get_logger().info(f"Transform SUCCESS: ({pt_world.point.x:.3f}, {pt_world.point.y:.3f}, {pt_world.point.z:.3f})")
            
            # Visualize in RViz (Marker)
            marker = Marker()
            marker.header = pt_world.header
            marker.ns = "grasp_target"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = pt_world.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)
            
            # [Phase 4] Action
            self.planner.plan_and_execute(pt_world.point)

def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
