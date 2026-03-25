#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np
import time
import os
import yaml

def list_to_matrix(trans, rot):
    # rot is [x, y, z, w]
    # transforms3d uses w, x, y, z order
    return transforms3d.affines.compose(
        trans, 
        transforms3d.quaternions.MatQuat2Qt(np.array(rot).reshape(3,3) if len(rot)==9 else [rot[3], rot[0], rot[1], rot[2]]),
        np.ones(3)
    )

def list_to_matrix_ros(trans, rot):
    # rot is [x, y, z, w]
    # transforms3d.quaternions.quat2mat expects [w, x, y, z]
    w, x, y, z = rot[3], rot[0], rot[1], rot[2]
    mat = transforms3d.quaternions.quat2mat([w, x, y, z])
    T = np.eye(4)
    T[:3, :3] = mat
    T[:3, 3] = trans
    return T

def matrix_to_transform(mat, parent_frame, child_frame):
    t = TransformStamped()
    t.header.stamp = rclpy.time.Time().to_msg() 
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    
    trans = mat[:3, 3]
    # mat2quat returns w, x, y, z
    w, x, y, z = transforms3d.quaternions.mat2quat(mat[:3, :3])
    
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]
    t.transform.rotation.x = x
    t.transform.rotation.y = y
    t.transform.rotation.z = z
    t.transform.rotation.w = w
    return t

class MultiCamBroadcaster(Node):
    def __init__(self):
        super().__init__('multi_cam_broadcaster')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # --- CONFIGURATION FROM CALIBRATION ---
        # Load params from yaml if possible
        config_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'config', 'calibration_params.yaml')
        
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                self.calibrations = yaml.safe_load(f)
            self.get_logger().info(f"Loaded calibration from {config_path}")
        else:
            self.get_logger().warn(f"Calibration file {config_path} not found. Using defaults.")
            self.calibrations = {
                # Defaults...
            }
        
        self.resolved_transforms = {}
        self.timer = self.create_timer(1.0, self.resolve_and_publish)
        self.get_logger().info("MultiCamBroadcaster started. Waiting for internal camera transforms...")

    def resolve_and_publish(self):
        # We need to find T_opt_to_link for each child camera to properly parent the link frame
        # T_parent_to_child_link = T_parent_to_child_opt * T_child_opt_to_child_link
        # T_child_opt_to_child_link = inv(T_child_link_to_child_opt)
        
        all_resolved = True
        
        for cam_name, calib in self.calibrations.items():
            if cam_name in self.resolved_transforms:
                continue # Already calculated
            
            try:
                # Look up T_link_to_opt (Internal RS transform)
                # Wait for transform to be available
                if self.tf_buffer.can_transform(calib['child_link_frame'], calib['child_optical_frame'], rclpy.time.Time()):
                    
                    # Get T_link_to_opt
                    tf_link_to_opt = self.tf_buffer.lookup_transform(
                        calib['child_link_frame'], 
                        calib['child_optical_frame'], 
                        rclpy.time.Time(),
                         timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    
                    T_link_to_opt = list_to_matrix_ros(
                        [tf_link_to_opt.transform.translation.x, tf_link_to_opt.transform.translation.y, tf_link_to_opt.transform.translation.z],
                        [tf_link_to_opt.transform.rotation.x, tf_link_to_opt.transform.rotation.y, tf_link_to_opt.transform.rotation.z, tf_link_to_opt.transform.rotation.w]
                    )
                    
                    # We need T_opt_to_link (Inverse)
                    T_opt_to_link = np.linalg.inv(T_link_to_opt)
                    
                    # Get Calibrated T_parent_to_opt
                    T_parent_to_opt = list_to_matrix_ros(calib['pos'], calib['quat'])
                    
                    # Calculate result: T_parent_to_link
                    T_parent_to_link = np.dot(T_parent_to_opt, T_opt_to_link)
                    
                    # Create transform message
                    ts = matrix_to_transform(T_parent_to_link, calib['parent_frame'], calib['child_link_frame'])
                    
                    self.resolved_transforms[cam_name] = ts
                    self.static_broadcaster.sendTransform(ts)
                    self.get_logger().info(f"Resolved and published transform for {cam_name}")
                    
                else:
                    self.get_logger().warn(f"Waiting for transform {calib['child_link_frame']} -> {calib['child_optical_frame']}")
                    all_resolved = False
                    
            except Exception as e:
                self.get_logger().error(f"Error resolving {cam_name}: {e}")
                all_resolved = False

        if all_resolved:
            # If all are resolved, we don't need the timer anymore, strictly speaking, 
            # but static broadcaster latches, so we can just idle or stop the timer.
            # We keep sending just in case of restart of other nodes? No, static TF is latched.
            pass

def main():
    rclpy.init()
    node = MultiCamBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
