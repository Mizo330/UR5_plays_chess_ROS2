import rclpy
import asyncio
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.action._move_group import MoveGroup_FeedbackMessage
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters, JointConstraint
from geometry_msgs.msg import PoseStamped, Quaternion
from shape_msgs.msg import SolidPrimitive
from ur_chess.misc.math_helpers import make_vector3, make_quaternion, make_point
from ur_chess_msgs.srv import URChessMoveToPosition
import math
import yaml
import os
from typing import Optional

WORKSPACE_MIN_CORNER = make_vector3(x=-1.0, y=-1.0, z=-1.0)
WORKSPACE_MAX_CORNER = make_vector3(x=1.0, y=1.0, z=1.0)

class MoveItCommander(Node):

    def __init__(self):
        super().__init__('moveit_goal_sender')

        self.declare_parameter('constraints_file', '/home/appuser/ros2_ws/src/my_packages/ur_chess/config/planning_constaints.yaml')
        self.constraints_file = self.get_parameter('constraints_file').get_parameter_value().string_value

        self.constraints_yaml = self.load_constraints_from_yaml()
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveGroup action server is available.')

        success = self.send_to_start()
        if success:
            self.get_logger().info('Successfully moved to start position.')
        else:
            self.get_logger().error('Failed to move to start position.')
        
        self.create_service(URChessMoveToPosition, 'ur_chess/move_to_position', self.move_to_position_callback)
        self.get_logger().info('Service ur_chess/move_to_position is ready.')
                
    def load_constraints_from_yaml(self):
        if not os.path.isfile(self.constraints_file):
            self.get_logger().error(f"Constraints file not found: {self.constraints_file}")
            return {}
        with open(self.constraints_file, 'r') as f:
            return yaml.safe_load(f)
    
    def move_to_position_callback(self, request, response):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position = make_point(request.x, request.y, request.z)
        pose.pose.orientation = make_quaternion(0.0, -1.0, 0.0, 0.0)

        goal_msg = self.create_movegroup_goal(pose)
        success = self.send_goal(goal_msg)

        response.success = success
        return response

    def build_constraints(self, pose: PoseStamped) -> Constraints:
        data = self.constraints_yaml
        constraints = Constraints()

        # Position constraint
        pos_c = PositionConstraint()
        pos_c.header = pose.header
        pos_c.link_name = data['position_constraint']['link_name']
        pos_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[data['position_constraint']['tolerance']]))
        pos_c.constraint_region.primitive_poses.append(pose.pose)
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        # Orientation constraint
        ori_c = OrientationConstraint()
        ori_c.header = pose.header
        ori_c.link_name = data['orientation_constraint']['link_name']
        ori_c.orientation = pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = data['orientation_constraint']['tolerances'][0]
        ori_c.absolute_y_axis_tolerance = data['orientation_constraint']['tolerances'][1]
        ori_c.absolute_z_axis_tolerance = data['orientation_constraint']['tolerances'][2]
        ori_c.weight = 1.0
        constraints.orientation_constraints.append(ori_c)

        # Joint constraint
        joint = JointConstraint()
        joint.joint_name = data['joint_constraint_shoulder_pan']['joint_name']
        joint.position = math.radians(data['joint_constraint_shoulder_pan']['angle_deg'])
        joint.tolerance_above = math.radians(data['joint_constraint_shoulder_pan']['tolerance_deg'])
        joint.tolerance_below = math.radians(data['joint_constraint_shoulder_pan']['tolerance_deg'])
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
        
        joint = JointConstraint()
        joint.joint_name = data['joint_constraint_wrist1']['joint_name']
        joint.position = math.radians(data['joint_constraint_wrist1']['angle_deg'])
        joint.tolerance_above = math.radians(data['joint_constraint_wrist1']['tolerance_deg'])
        joint.tolerance_below = math.radians(data['joint_constraint_wrist1']['tolerance_deg'])
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
        return constraints

    def create_movegroup_goal(self, pose: PoseStamped) -> MoveGroup.Goal:
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.workspace_parameters.min_corner = WORKSPACE_MIN_CORNER
        goal_msg.request.workspace_parameters.max_corner = WORKSPACE_MAX_CORNER

        goal_msg.request.goal_constraints.append(self.build_constraints(pose))
        return goal_msg

    def send_to_start(self):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position = make_point(0.0, -0.3, 0.6)
        pose.pose.orientation = make_quaternion(0.0, -1.0, 0.0, 0.0)
        goal_msg = self.create_movegroup_goal(pose)

        self.get_logger().info('Sending goal to start position...')
        return self.send_goal(goal_msg)

    def send_goal(self, goal_msg: MoveGroup.Goal):
        self.get_logger().info('Sending goal...')
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error('Timeout while waiting for goal handle.')
            return False
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server.')
            return False

        # Wait for result (returns a Future)
        get_result_future = goal_handle.get_result_async()
        #rclpy.spin_until_future_complete(self, get_result_future)
        # Get result
        result = get_result_future.result()
        if result.status == 6:  # ACTION_RESULT_ABORTED
            self.get_logger().error('Goal execution was aborted.')
            return False
        elif result.status == 4:  # ACTION_RESULT_SUCCEEDED
            self.get_logger().info('Goal succeeded!')
            return True
        else:
            self.get_logger().warn(f'Goal finished with status {result.status}')
            return False
        
    def feedback_callback(self, feedback: MoveGroup_FeedbackMessage):
        self.get_logger().info(f'Feedback: {feedback.feedback.state}')
        
    def move_to(self, x: float, y: float, z: float, orientation: Optional[Quaternion] = None):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position = make_point(x, y, z)
        pose.pose.orientation = orientation if orientation else make_quaternion(0.0, -1.0, 0.0, 0.0)

        goal_msg = self.create_movegroup_goal(pose)
        self.send_goal(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    commander = MoveItCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()
