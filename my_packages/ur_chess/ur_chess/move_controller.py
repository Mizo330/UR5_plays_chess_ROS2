import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, MoveGroupSequence
from moveit_msgs.action._move_group import MoveGroup_FeedbackMessage
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters, JointConstraint, MoveItErrorCodes, MotionSequenceItem
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from ur_chess.misc.math_helpers import make_vector3, make_quaternion, make_point
from ur_chess_msgs.srv import URChessMovePiece
from ur_chess_msgs.msg import URChessMoveResult
import math
import yaml
import os
from typing import Optional
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
import time
import threading
from ur_chess.misc.text_color import TextColor

WORKSPACE_MIN_CORNER = make_vector3(x=-1.0, y=-1.0, z=-1.0)
WORKSPACE_MAX_CORNER = make_vector3(x=1.0, y=1.0, z=1.0)

CHESSPIECE_CLEARANCE = 0.2  # Clearance above the chess piece

class MoveItCommander(Node):

    def __init__(self):
        super().__init__('moveit_goal_sender')

        # Declare and load the constraints file parameter
        self.declare_parameter('constraints_file', '/home/appuser/ros2_ws/src/my_packages/ur_chess/config/planning_constaints.yaml')
        self.constraints_file = self.get_parameter('constraints_file').get_parameter_value().string_value

        self.constraints_yaml = self.load_constraints_from_yaml()

        # Create ActionClient and wait for the server
        self.get_logger().info('Waiting for MoveGroup action server...')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveGroup action server is available.')

        # Create a publisher for the feedback messages
        self.move_res_pub = self.create_publisher(URChessMoveResult,  '/ur_chess/move_result', 10)
        
        # Create a service
        self.create_service(URChessMovePiece, 'ur_chess/move_piece', self.move_piece_callback)
        self.get_logger().info('Service ur_chess/move_piece is ready.')

        # Use ReentrantCallbackGroup for managing callbacks concurrently
        self._callback_group = ReentrantCallbackGroup()
        self._goal_queue = []   
        self._active_goal = None
        self._goal_done_condition = threading.Condition()
        
        self.get_logger().info(TextColor.color_text('MoveItCommander initialized', TextColor.OKCYAN))
        
    def load_constraints_from_yaml(self):
        if not os.path.isfile(self.constraints_file):
            self.get_logger().error(f"Constraints file not found: {self.constraints_file}")
            return {}
        with open(self.constraints_file, 'r') as f:
            return yaml.safe_load(f)
        
    def enqueue_goal(self, goal_msg, on_result_cb=None, goal_str =None):
        """Add a goal to the queue. on_result_cb is called with the result."""
        self._goal_queue.append((goal_msg, on_result_cb, goal_str))
        # if nothing is active right now, start the first one
        if self._active_goal is None:
            self._send_next()

    def _send_next(self):
        if not self._goal_queue:
            if self.res_msg.success:
                self.get_logger().info('All goals completed.')
            else:
                self.get_logger().error('All goals failed.')
            self.move_res_pub.publish(self.res_msg)
            return

        goal_msg, on_result_cb, goal_str = self._goal_queue.pop(0)
        if goal_str:
            self.get_logger().info(f'Sending goal: {goal_str}')
        else:
            self.get_logger().info(f'Sending next goal')
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda fut: self._on_goal_response(fut, on_result_cb)
        )
        # mark that a goal is pending acceptance
        self._active_goal = send_future
    
    def _on_goal_response(self, future, on_result_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self._active_goal = None
            self._send_next()
            return

        # now wait for completion
        result_future = goal_handle.get_result_async()  # async result :contentReference[oaicite:2]{index=2}
        result_future.add_done_callback(
            lambda f: self._on_result(f, on_result_cb)
        )
        # store the actual goal handle to block further sends until done
        self._active_goal = goal_handle
        
    def _on_result(self, future, on_result_cb):
        result: MoveGroup.Result = future.result().result
        status = future.result().status
        self.res_msg = URChessMoveResult()
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal succeeded')
            self.res_msg.success = True
            self.res_msg.message = 'Goal succeeded'
            self.res_msg.error_code = result.error_code.val
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            self.get_logger().warn(f'Error code: {result.error_code.message}')
            self.res_msg.success = False
            self.res_msg.message = f'Goal failed with status: {status}'
            self.res_msg.error_code = result.error_code.val
            #flush the queue
            self._goal_queue.clear()
                
        # call user‐provided callback if any
        if on_result_cb:
            try:
                on_result_cb(result, status)
            except Exception as e:
                self.get_logger().error(f'Callback throw: {e}')

        # clear active and send the next in queue
        self._active_goal = None
                
        time.sleep(1)
        self._send_next()
        
    def move_piece_callback(self, request:URChessMovePiece.Request, response:URChessMovePiece.Response):
        """Callback for the move_piece service."""
        self.get_logger().info('Received request to move to position')
        try:
            #TODO do some assertions here
            start_x = request.start_x
            start_y = request.start_y
            start_z = request.start_z
            end_x = request.end_x
            end_y = request.end_y
            end_z = request.end_z
            
            #Above the start piece
            goal1 = self.create_mg_from_point(start_x, start_y, start_z+CHESSPIECE_CLEARANCE)
            #At the start piece
            goal2 = self.create_mg_from_point(start_x, start_y, start_z)
            #Above the end piece
            goal3 = self.create_mg_from_point(end_x, end_y, end_z+CHESSPIECE_CLEARANCE)
            #At the end piece
            goal4 = self.create_mg_from_point(end_x, end_y, end_z)
            # Queue the goals
            self.enqueue_goal(goal1, goal_str='Move above piece')
            self.enqueue_goal(goal2, goal_str='Grab piece')
            self.enqueue_goal(goal1, goal_str='Move up')
            self.enqueue_goal(goal3, goal_str='Move to goal position')
            self.enqueue_goal(goal4, goal_str='Place piece')
            self.enqueue_goal(goal3, goal_str='Move up')
            
            self.get_logger().info('Waiting for goals to complete...')
            response.success = True
            response.message = 'Goals queued successfully, await succecs result on /robot_result'
            return response
        except Exception as e:
            self.get_logger().error(f'Error in move_piece_callback: {e}')
            response.message = f'Error in move_piece_callback: {e}'
            response.success = False
            return response

    def create_mg_from_point(self,x, y, z):
        """Create a MoveGroup goal from a point."""
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position = make_point(x, y, z)
        pose.pose.orientation = make_quaternion(0.0, -1.0, 0.0, 0.0)
        goal_msg = self.create_movegroup_goal(pose)
        return goal_msg
        
        # if result.error_code.val == MoveItErrorCodes.SUCCESS:
        #     self.get_logger().info('Goal execution was successful!')
        #     self._goal_success = True
        # elif result.error_code.val == MoveItErrorCodes.FAILURE:
        #     self.get_logger().error('Goal execution failed.')
        #     self._goal_success = False
        # elif result.error_code.val == MoveItErrorCodes.PLANNING_FAILED:
        #     self.get_logger().error('Planning failed.')
        #     self._goal_success = False
        # elif result.error_code.val == MoveItErrorCodes.TIMED_OUT:
        #     self.get_logger().error('Goal execution timed out.')
        #     self._goal_success = False
        # elif result.error_code.val == MoveItErrorCodes.ABORT:
        #     self.get_logger().error('Goal execution was aborted.')
        #     self._goal_success = False
        # else:
        #     self.get_logger().error(f'Goal execution failed with error code: {result.error_code}')
        #     self._goal_success = False
        
    
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
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.goal_constraints.append(self.build_constraints(pose))
        return goal_msg

    # def send_to_start(self):
    #     pose = PoseStamped()
    #     pose.header.frame_id = 'base_link'
    #     pose.pose.position = make_point(0.0, -0.3, 0.6)
    #     pose.pose.orientation = make_quaternion(0.0, -1.0, 0.0, 0.0)
    #     goal_msg = self.create_movegroup_goal(pose)

    #     self.get_logger().info('Sending goal to start position...')
    #     return self.send_goal(goal_msg)
        
def main(args=None):
    rclpy.init(args=args)
    commander = MoveItCommander()
    
    # Use a single threaded executor to handle the callbacks
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(commander)
    executor.spin()

    commander.destroy_node()
    rclpy.shutdown()
