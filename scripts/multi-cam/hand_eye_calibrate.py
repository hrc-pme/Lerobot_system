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
import json
import yaml
from scipy.spatial.transform import Rotation as R

class RobotHandEyeCalib(Node):
    def __init__(self, cam_ns, robot_base_frame, robot_ee_frame, marker_size, marker_id, marker_dict_name, output_file=None):
        super().__init__('robot_hand_eye_calib')
        
        self.cam_ns = cam_ns
        self.robot_base_frame = robot_base_frame
        self.robot_ee_frame = robot_ee_frame
        self.marker_size = marker_size 
        self.marker_id = marker_id
        self.output_file = output_file
        
        self.aruco_dict = self.get_aruco_dict(marker_dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera Subscribers
        # Assuming typical realsense topics
        self.sub_cam_info = self.create_subscription(
            CameraInfo, f'{cam_ns}/color/camera_info', self.cam_info_cb, 10
        )
        self.sub_cam_img = self.create_subscription(
            Image, f'{cam_ns}/color/image_raw', self.cam_img_cb, 10
        )
        
        self.intrinsics = None
        self.dist_coeffs = None
        self.tag_pose = None  # T_cam_tag (4x4)
        self.ee_pose = None   # T_base_ee (4x4)
        
        self.samples_R_gripper2base = []
        self.samples_t_gripper2base = []
        self.samples_R_target2cam = []
        self.samples_t_target2cam = []
        
        self.collected_samples = 0
        self.required_samples = 15 # Need diverse poses
        
        self.get_logger().info(f"STARTING HAND-EYE CALIBRATION")
        self.get_logger().info(f"Camera: {cam_ns}")
        self.get_logger().info(f"Robot Base: {robot_base_frame}")
        self.get_logger().info(f"Robot EE:   {robot_ee_frame}")
        self.get_logger().info("Please hold the tag with the robot/gripper or attach it rigidly.")
        self.get_logger().info("Move the robot to a NEW POS and press ENTER in the terminal to capture.")

    def get_aruco_dict(self, name):
        if "APRILTAG" in name:
            if "36h11" in name: return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        elif "4X4" in name:
            return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    def cam_info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f"Received Camera Info: {msg.header.frame_id}")

    def detect_tag(self, cv_image):
        if self.intrinsics is None: return None
        
        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params
        )
        
        if ids is not None and self.marker_id in ids:
            index = np.where(ids == self.marker_id)[0][0]
            marker_corners = corners[index][0]
            
            # Solve PnP
            half_size = self.marker_size / 2.0
            obj_points = np.array([
                [-half_size, half_size, 0],
                [half_size, half_size, 0],
                [half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners, self.intrinsics, self.dist_coeffs
            )
            
            if success:
                rmat, _ = cv2.Rodrigues(rvec)
                T = np.eye(4)
                T[:3, :3] = rmat
                T[:3, 3] = tvec.flatten()
                return T
        return None

    def cam_img_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.tag_pose = self.detect_tag(cv_image)
            
            # Optional visualization
            if self.tag_pose is not None:
                # We could publish debug image...
                pass
                
        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

    def capture_sample(self):
        # Get latest robot pose
        try:
            if not self.tf_buffer.can_transform(self.robot_base_frame, self.robot_ee_frame, rclpy.time.Time()):
                self.get_logger().warn(f"Cannot find transform {self.robot_base_frame} -> {self.robot_ee_frame}")
                return False
                
            t = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.robot_ee_frame, rclpy.time.Time()
            )
            
            # Convert to matrix
            trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            rot = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            
            r_mat = R.from_quat(rot).as_matrix()
            t_vec = np.array(trans).reshape(3,1)
            
            # Get latest Vision Pose
            if self.tag_pose is None:
                self.get_logger().warn("Tag not detected in camera view!")
                return False
                
            # Store T_base_ee
            self.samples_R_gripper2base.append(r_mat)
            self.samples_t_gripper2base.append(t_vec)
            
            # Store T_cam_target
            # cv2.calibrateHandEye expects:
            # R_gripper2base, t_gripper2base: Robot poses
            # R_target2cam, t_target2cam: Tag poses in Camera frame
            
            self.samples_R_target2cam.append(self.tag_pose[:3, :3])
            self.samples_t_target2cam.append(self.tag_pose[:3, 3].reshape(3,1))
            
            self.collected_samples += 1
            print(f"Captured Sample {self.collected_samples}/{self.required_samples}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Sample Capture Failed: {e}")
            return False

    def solve(self):
        if self.collected_samples < 3:
            print("Not enough samples.")
            return
            
        print("\nSolving Hand-Eye Calibration...")
        
        # Method: Eye-To-Hand (Camera is static)
        # We solve AX = XB
        # Returns (R_cam2base, t_cam2base)
        # Note: OpenCV docs say calibrateHandEye returns R_cam2gripper? No.
        # For EYE_TO_HAND:
        # returns R_base2cam, t_base2cam?
        # Let's check docs or standard: Usually returns T_base_cam.
        
        try:
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                self.samples_R_gripper2base,
                self.samples_t_gripper2base,
                self.samples_R_target2cam,
                self.samples_t_target2cam,
                method=cv2.CALIB_HAND_EYE_TSAI
            )
            
            print("Done!")
            print("-" * 50)
            print(f"Translation (x, y, z): {t_cam2base.flatten()}")
            # R_cam2base is 3x3
            quat = R.from_matrix(R_cam2base).as_quat() # x, y, z, w
            print(f"Rotation (x, y, z, w): {quat}")
            print("-" * 50)
            
            # For static_transform_publisher, we need parent -> child
            # Usually we want T_base_cam.
            # verify directions.
            
            result = {
                "base_frame": self.robot_base_frame,
                "camera_frame": self.cam_ns.replace("/camera/", "") + "_link", # Assuming std naming
                "pos": t_cam2base.flatten().tolist(),
                "quat": quat.tolist()
            }
            
            if self.output_file:
                # Update YAML
                base_cam_name = self.cam_ns.split('/')[-1] # camera_top
                
                # Check if file exists
                if os.path.exists(self.output_file):
                    with open(self.output_file, 'r') as f:
                        data = yaml.safe_load(f) or {}
                else:
                    data = {}
                    
                # Store
                # We need to adapt the schema.
                # Currently schema key is CHILD camera.
                # So key: camera_top
                # parent_frame: robot_base_frame
                
                data[base_cam_name] = {
                    "parent_frame": self.robot_base_frame,
                    "child_optical_frame": f"{base_cam_name}_color_optical_frame",
                    "child_link_frame": f"{base_cam_name}_link",
                    "pos": result['pos'],
                    "quat": result['quat']
                }
                
                with open(self.output_file, 'w') as f:
                    yaml.dump(data, f)
                print(f"Calibration saved to {self.output_file}")

        except cv2.error as e:
            print(f"OpenCV Error: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam", type=str, default="/camera/camera_top", help="Camera Namespace")
    parser.add_argument("--base", type=str, required=True, help="Robot Base Frame (e.g. left_follower_base)")
    parser.add_argument("--ee", type=str, required=True, help="Robot End-Effector Frame (e.g. left_follower_end_effector)")
    parser.add_argument("--size", type=float, default=0.10, help="Marker size")
    parser.add_argument("--id", type=int, default=0, help="Marker ID")
    parser.add_argument("--config", type=str, default="scripts/multi-cam/config/calibration_params.yaml")
    
    args = parser.parse_args()
    
    rclpy.init()
    calib = RobotHandEyeCalib(args.cam, args.base, args.ee, args.size, args.id, "APRILTAG_36h11", args.config)
    
    # Simple loop for interaction
    import threading
    
    def ros_spin():
        rclpy.spin(calib)
        
    thread = threading.Thread(target=ros_spin)
    thread.start()
    
    try:
        while rclpy.ok() and calib.collected_samples < calib.required_samples:
            input(f"Press ENTER to capture sample {calib.collected_samples + 1}/{calib.required_samples} (Ctrl+C to quit)...")
            if not calib.capture_sample():
                print("Retry...")
        
        calib.solve()
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        thread.join()
        os._exit(0)

if __name__ == "__main__":
    main()
