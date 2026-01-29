import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import numpy as np

class SpatialCalculator:
    def __init__(self, node: Node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def pixel_to_point(self, u, v, depth, intrinsics, frame_id):
        """
        Convert (u, v, depth) -> PointStamped in Camera Frame
        intrinsics: tuple (fx, fy, cx, cy)
        """
        fx, fy, cx, cy = intrinsics
        
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth

        pt = PointStamped()
        pt.header.stamp = self.node.get_clock().now().to_msg()
        pt.header.frame_id = frame_id
        pt.point.x = x_cam
        pt.point.y = y_cam
        pt.point.z = z_cam
        
        return pt

    def transform_to_world(self, point_stamped: PointStamped, target_frame="world"):
        try:
            # Look up transform
            # Note: We use Time(seconds=0) to get the latest available transform
            # to be robust against slight time defs
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point_stamped.header.frame_id,
                rclpy.time.Time()) # latest
            
            # Perform transform
            world_point = do_transform_point(point_stamped, transform)
            return world_point
            
        except tf2_ros.LookupException as e:
            self.node.get_logger().error(f"TF Lookup Error: {e}")
            return None
        except tf2_ros.ExtrapolationException as e:
            self.node.get_logger().error(f"TF Extrapolation Error: {e}")
            return None
        except Exception as e:
            self.node.get_logger().error(f"TF General Error: {e}")
            return None
