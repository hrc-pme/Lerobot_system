#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import argparse
import sys
import os
import time
from scipy.spatial.transform import Rotation as R
import json

class MultiCamCalibrator(Node):
    def __init__(self, cam1_ns, cam2_ns, marker_size, marker_id, marker_dict_name, output_file=None):
        super().__init__('multi_cam_calibrator')
        
        self.cam1_ns = cam1_ns
        self.cam2_ns = cam2_ns
        self.marker_size = marker_size # meters
        self.marker_id = marker_id
        self.output_file = output_file
        
        # Determine dictionary
        self.aruco_dict = self.get_aruco_dict(marker_dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.bridge = CvBridge()
        
        # Camera 1 subs
        self.sub_cam1_info = self.create_subscription(
            CameraInfo, f'{cam1_ns}/color/camera_info', self.cam1_info_cb, 10
        )
        self.sub_cam1_img = self.create_subscription(
            Image, f'{cam1_ns}/color/image_raw', self.cam1_img_cb, 10
        )
        
        # Camera 2 subs
        self.sub_cam2_info = self.create_subscription(
            CameraInfo, f'{cam2_ns}/color/camera_info', self.cam2_info_cb, 10
        )
        self.sub_cam2_img = self.create_subscription(
            Image, f'{cam2_ns}/color/image_raw', self.cam2_img_cb, 10
        )
        
        self.cam1_intrinsics = None
        self.cam1_dist_coeffs = None
        self.cam2_intrinsics = None
        self.cam2_dist_coeffs = None
        
        self.cam1_pose = None # T_c1_m (Marker in Cam1)
        self.cam2_pose = None # T_c2_m (Marker in Cam2)
        
        self.cam1_frame_id = None
        self.cam2_frame_id = None
        
        self.samples = []
        self.sample_count = 0
        self.required_samples = 30
        
        self.get_logger().info(f"Waiting for images from {cam1_ns} and {cam2_ns}...")
        self.get_logger().info(f"Looking for Marker ID {marker_id} of size {marker_size}m")

    def get_aruco_dict(self, name):
        # Default to 36h11 (AprilTag) if not specified or found
        if "APRILTAG" in name:
            if "36h11" in name: return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            # Add others if needed
        elif "4X4" in name:
            return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # Fallback
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    def cam1_info_cb(self, msg):
        if self.cam1_intrinsics is None:
            self.cam1_intrinsics = np.array(msg.k).reshape((3, 3))
            self.cam1_dist_coeffs = np.array(msg.d)
            self.cam1_frame_id = msg.header.frame_id
            self.get_logger().info(f"Received Cam1 Info: {self.cam1_frame_id}")

    def cam2_info_cb(self, msg):
        if self.cam2_intrinsics is None:
            self.cam2_intrinsics = np.array(msg.k).reshape((3, 3))
            self.cam2_dist_coeffs = np.array(msg.d)
            self.cam2_frame_id = msg.header.frame_id
            self.get_logger().info(f"Received Cam2 Info: {self.cam2_frame_id}")

    def detect_marker(self, cv_image, intrinsics, dist_coeffs):
        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params
        )
        
        if ids is not None and self.marker_id in ids:
            index = np.where(ids == self.marker_id)[0][0]
            marker_corners = corners[index][0]
            
            # Solve PnP
            # Define 3D points of marker in marker frame (z=0)
            # corners are usually TopLeft, TopRight, BottomRight, BottomLeft
            half_size = self.marker_size / 2.0
            obj_points = np.array([
                [-half_size, half_size, 0],
                [half_size, half_size, 0],
                [half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners, intrinsics, dist_coeffs
            )
            
            if success:
                # Draw for debug (optional)
                # cv2.aruco.drawDetectedMarkers(cv_image, corners)
                # cv2.drawFrameAxes(cv_image, intrinsics, dist_coeffs, rvec, tvec, 0.1)
                
                # Construct 4x4 homogenous matrix T_c_m
                rmat, _ = cv2.Rodrigues(rvec)
                T = np.eye(4)
                T[:3, :3] = rmat
                T[:3, 3] = tvec.flatten()
                return T
        return None

    def cam1_img_cb(self, msg):
        if self.cam1_intrinsics is None: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cam1_pose = self.detect_marker(cv_image, self.cam1_intrinsics, self.cam1_dist_coeffs)
            self.check_calibration()
        except Exception as e:
            self.get_logger().error(f"Cam1 Error: {e}")

    def cam2_img_cb(self, msg):
        if self.cam2_intrinsics is None: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cam2_pose = self.detect_marker(cv_image, self.cam2_intrinsics, self.cam2_dist_coeffs)
            self.check_calibration()
        except Exception as e:
            self.get_logger().error(f"Cam2 Error: {e}")

    def check_calibration(self):
        # If we have both poses recently updated (in a real system we should check timestamps, 
        # but for static scene assuming roughly simultaneous is okay if simple)
        if self.cam1_pose is not None and self.cam2_pose is not None:
            # Calculate T_c1_c2 = T_c1_m * (T_c2_m)^-1
            # We want pose of c2 in c1 frame? Or parent to child transform.
            # Usually static_transform_publisher args: x y z qx qy qz qw parent_frame child_frame
            # Let's say we want to publish c2 relative to c1. So c1 is parent.
            # T_c1_c2 = T_c1_m * inv(T_c2_m)
            
            T_c2_m_inv = np.linalg.inv(self.cam2_pose)
            T_c1_c2 = np.dot(self.cam1_pose, T_c2_m_inv)
            
            self.samples.append(T_c1_c2)
            self.sample_count += 1
            
            # Reset poses to ensure we get fresh ones next time
            self.cam1_pose = None
            self.cam2_pose = None
            
            sys.stdout.write(f"\rCollecting samples: {self.sample_count}/{self.required_samples}")
            sys.stdout.flush()
            
            if self.sample_count >= self.required_samples:
                self.finalize_calibration()

    def finalize_calibration(self):
        print("\n\nComputing average transform...")
        
        # Simple averaging of translation
        trans_sum = np.zeros(3)
        quats = []
        
        for T in self.samples:
            trans_sum += T[:3, 3]
            r = R.from_matrix(T[:3, :3])
            quats.append(r.as_quat())
            
        avg_trans = trans_sum / len(self.samples)
        
        # Average quaternions (just mean normalization for simple approximation or better: chordal L2 mean)
        # For small differences, simple mean and normalize is okay.
        avg_quat = np.mean(quats, axis=0)
        avg_quat = avg_quat / np.linalg.norm(avg_quat)
        
        # Output result
        print("Done!")
        print("-" * 50)
        print(f"Parent Frame: {self.cam1_frame_id}")
        print(f"Child Frame:  {self.cam2_frame_id}")
        print("-" * 50)
        print(f"Translation (x, y, z): {avg_trans}")
        print(f"Rotation (x, y, z, w): {avg_quat}")
        print("-" * 50)
        print("Command for static_transform_publisher:")
        print(f"ros2 run tf2_ros static_transform_publisher "
              f"{avg_trans[0]:.6f} {avg_trans[1]:.6f} {avg_trans[2]:.6f} "
              f"{avg_quat[0]:.6f} {avg_quat[1]:.6f} {avg_quat[2]:.6f} {avg_quat[3]:.6f} "
              f"{self.cam1_frame_id} {self.cam2_frame_id}")
        print("-" * 50)
        
        if self.output_file:
            result = {
                "translation": avg_trans.tolist(),
                "rotation": avg_quat.tolist(),
                "parent_frame": self.cam1_frame_id,
                "child_frame": self.cam2_frame_id
            }
            with open(self.output_file, 'w') as f:
                json.dump(result, f)
            print(f"Calibration saved to {self.output_file}", flush=True)
        
        # Exit hard to avoid rclpy cleanup hang
        os._exit(0)

def main():
    parser = argparse.ArgumentParser(description="Calibrate two cameras using an ArUco/AprilTag marker.")
    parser.add_argument("--cam1", type=str, required=True, help="Namespace of camera 1 (e.g. /camera_1)")
    parser.add_argument("--cam2", type=str, required=True, help="Namespace of camera 2 (e.g. /camera_2)")
    parser.add_argument("--size", type=float, default=0.166, help="Marker size in meters (default: 0.166 for typical printout)")
    parser.add_argument("--id", type=int, default=0, help="Marker ID to detect (default: 0)")
    parser.add_argument("--dict", type=str, default="APRILTAG_36h11", help="Dictionary name (default: APRILTAG_36h11)")
    parser.add_argument("--output", type=str, default=None, help="Output JSON file for results")
    
    args = parser.parse_args()
    
    rclpy.init()
    node = MultiCamCalibrator(args.cam1, args.cam2, args.size, args.id, args.dict, args.output)
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
