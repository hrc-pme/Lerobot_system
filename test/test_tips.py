import sys, os
sys.path.append(os.path.abspath('scripts/end-effector'))
from robust_pino_kinematics import RobotKinematics
import numpy as np

kin_koch = RobotKinematics('/tmp/koch_left.urdf', tip_link='left_gripper_static_1')
pos_k, rot_k = kin_koch.forward_kinematics(np.zeros(kin_koch.dof))
print("Koch zero pose rot:", rot_k)

kin_oa = RobotKinematics('repos/openarm_dev/openarm_urdf/openarm_bimanual.urdf', tip_link='openarm_left_hand')
pos_o, rot_o = kin_oa.forward_kinematics(np.zeros(kin_oa.dof))
print("OpenArm zero pose rot:", rot_o)
