from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class Planner:
    def __init__(self, node: Node):
        self.node = node
        self.group_name = "koch_arm" 
        
        # 使用 Action Client 取代 MoveItPy
        self.action_client = ActionClient(self.node, MoveGroup, 'move_action')
        self._goal_in_progress = False
        
        self.node.get_logger().info("[Planner] Initialized using ROS 2 Action Client")
        self.node.get_logger().info("[Planner] Waiting for 'move_action' server...")
        
        # 注意: 請勿使用 wait_for_server() 因為這會卡住主執行緒的回呼
        # 我們會在 plan_and_execute 中檢查連線

    def plan_and_execute(self, point_msg):
        """
        Receives a geometry_msgs/Point (x,y,z) and plans a grasp using Action Client.
        """
        if not self.action_client.server_is_ready():
            self.node.get_logger().warn("[Planner] MoveGroup Action Server not ready yet. Retrying later...")
            return False
        if self._goal_in_progress:
            self.node.get_logger().info("[Planner] Goal already in progress. Skipping new request.")
            return False

        x = point_msg.x
        y = point_msg.y
        z = point_msg.z
        
        # [動態高度調整] 根據物體高度動態計算安全 offset
        # 如果物體在桌面上 (z~0.06m)，offset 設為 5cm (不要太高，怕超過工作空間)
        if z < 0.1:  # 物體在桌面附近
            safe_offset = 0.05  # 5cm
        else:
            safe_offset = 0.08  # 8cm
        
        target_z = z + safe_offset
        self.node.get_logger().info(f"[Planner] Object Height: {z:.3f}m, Safe Offset: {safe_offset:.3f}m, Target Z: {target_z:.3f}m")
        self.node.get_logger().info(f"[Planner] Sending Goal to MoveIt: ({x:.3f}, {y:.3f}, {target_z:.3f})")

        # --- Construct Goal ---
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        goal_msg.request.start_state.is_diff = True
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 30
        goal_msg.request.allowed_planning_time = 15.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3

        # --- Position Constraint ---
        # 限制 End Effector 必须到达指定位置
        pcm = PositionConstraint()
        pcm.header.frame_id = "world"
        pcm.link_name = "gripper_static_1"  # MoveIt 的 IK solver 实际绑定的末端 link
        pcm.target_point_offset.x = 0.0
        pcm.target_point_offset.y = 0.0
        pcm.target_point_offset.z = 0.0
        
        # 定義目標區域 (放寬容忍度到 5cm，讓 IK 更容易採樣)
        bv = BoundingVolume()
        pr = SolidPrimitive()
        pr.type = SolidPrimitive.BOX
        pr.dimensions = [0.05, 0.05, 0.05]  # 5cm tolerance box
        bv.primitives.append(pr)
        
        # --- Position Constraint 使用已計算的 target_z ---
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = target_z
        # Orientation is not constrained in PositionConstraint primitives usually, 
        # but the constraint region center is defined by primitive_poses.
        bv.primitive_poses.append(target_pose)
        
        pcm.constraint_region = bv
        pcm.weight = 1.0

        # [最小化約束：只用位置約束，先確保可達性]
        # 末端位置必須到達目標 (gripper_static_1)，其他角度由 IK 自由決定
        # 一旦這個版本成功，才考慮加入 joint4 或 orientation 約束

        # Add Constraints
        goal_msg.request.goal_constraints.append(Constraints())
        goal_msg.request.goal_constraints[0].position_constraints.append(pcm)
        
        goal_msg.planning_options.plan_only = False # Execute directly
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True

        # --- Send Goal ---
        self.node.get_logger().info("[Planner] Waiting for result...")
        
        # 在 Callback 中同步呼叫 Action 是一個冒險行為，
        # 但為了簡單起見，我們使用 send_goal_async 並讓它在背景執行。
        # 因為我們不能在這裡 await (會卡住 pipeline loop)
        
        self._goal_in_progress = True
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('[Planner] Goal rejected :(')
            self._goal_in_progress = False
            return

        self.node.get_logger().info('[Planner] Goal accepted! Execution starting...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        val = result.error_code.val
        if val == 1: # SUCCESS
            self.node.get_logger().info(f"[Planner] Execution Finished Successfully! (Error Code: {val})")
        else:
            self.node.get_logger().error(f"[Planner] Execution Failed. Error Code: {val}. (1=Success)")
        self._goal_in_progress = False
