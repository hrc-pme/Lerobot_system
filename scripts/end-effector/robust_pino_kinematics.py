import sys
import os

# Fix path for 'pin' package if installed via cmeel (overrides system pinocchio)
cmeel_path = "/usr/local/lib/python3.10/dist-packages/cmeel.prefix/lib/python3.10/site-packages"
if os.path.exists(cmeel_path):
    sys.path.insert(0, cmeel_path)

import pinocchio as pin
import numpy as np

class RobotKinematics:
    def __init__(self, urdf_path, tip_link=None):
        # Load Pinocchio Model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # Determine Frame ID for Tip Link
        if tip_link:
            if not self.model.existFrame(tip_link):
                 # Try to match fuzzy or just list frames
                 print(f"Frame {tip_link} not found. Available frames:")
                 for f in self.model.frames:
                     print(f.name)
                 raise ValueError(f"Frame {tip_link} not found in model.")
            self.tip_frame_id = self.model.getFrameId(tip_link)
        else:
            # Heuristic: find frame with 'gripper' or last frame
            found = False
            for f in self.model.frames:
                if 'gripper' in f.name:
                    self.tip_frame_id = self.model.getFrameId(f.name)
                    print(f"Selected tip frame: {f.name}")
                    found = True
                    break
            if not found:
                self.tip_frame_id = len(self.model.frames) - 1 # Last frame
                print(f"Selected last frame as tip: {self.model.frames[self.tip_frame_id].name}")

        self.dof = self.model.nq
        self.q_min = self.model.lowerPositionLimit
        self.q_max = self.model.upperPositionLimit

    def forward_kinematics(self, q):
        """
        Returns:
            pos: [x, y, z]
            rot: [x, y, z, w]
        """
        q = np.array(q)
        pin.framesForwardKinematics(self.model, self.data, q)
        
        # Get frame state
        # oMf is the transformation of frame relative to universe (world)
        oMf = self.data.oMf[self.tip_frame_id]
        
        pos = oMf.translation
        # Convert rotation matrix to quaternion [x, y, z, w]
        quat = pin.Quaternion(oMf.rotation)
        # Pinocchio quaternion is [x, y, z, w] standard or [w, x, y, z]?
        # Pinocchio print shows x,y,z,w usually.
        # But wait, pinocchio.Quaternion(R) constructs (w, x, y, z)?
        # Let's check coefficients: .coeffs() returns [x, y, z, w]
        rot = quat.coeffs() 
        
        return pos, rot

    def inverse_kinematics_5dof(self, target_pos, target_quat, q_init, max_iter=20, tol=1e-3, damping=1.0):
        """
        Damped Least Squares IK using Pinocchio.
        Higher damping (1.0) helps with stability near singularity for 5DOF
        """
        q = np.array(q_init, dtype=np.float64)
        
        # Target Transform
        oMdes = pin.SE3(pin.Quaternion(np.array(target_quat)), np.array(target_pos))
        
        for i in range(max_iter):
            pin.framesForwardKinematics(self.model, self.data, q)
            pin.computeJointJacobians(self.model, self.data, q)
            
            # Current Transform
            oMf = self.data.oMf[self.tip_frame_id]
            
            # Error in Local Frame of End Effector (body frame)
            # oMf.actInv(oMdes) is T_curr_des (Position of Desired in Current frame)
            dMf = oMf.actInv(oMdes) 
            err = pin.log(dMf).vector # 6D error vector (linear + angular)
            
            # For 5-DOF, we cannot achieve arbitrary 6D pose (rotation around EE axis usually free or constrained)
            # Let's see if relaxing one rotational DOF helps.
            # But DLS should handle it naturally by minimizing overall error norm.
            # If err is too big, DLS might do weird things.
            
            # If error is small enough, we are done
            if np.linalg.norm(err) < tol:
                break
                
            # Jacobian in Local Frame
            J = pin.getFrameJacobian(self.model, self.data, self.tip_frame_id, pin.ReferenceFrame.LOCAL)
            
            # DLS: J*dq = err
            # dq = J.T * (J J.T + l^2 I)^-1 * err
            
            J_h = J.T
            A = J @ J_h + np.eye(6) * (damping**2)
            
            # Standard DLS
            x = np.linalg.solve(A, err)
            dq = J_h @ x
            
            # Clamp step magnitude to prevent jumps
            if np.linalg.norm(dq) > 0.1:
                dq = dq * (0.1 / np.linalg.norm(dq))
            
            q = pin.integrate(self.model, q, dq)
            
            # Enforce limits (simple clipping)
            # q = np.clip(q, self.q_min, self.q_max) # Be careful with continuous joints if any
            
        return q
