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
# For configuration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.utils import create_params_file_from_dict
from ament_index_python import get_package_share_directory
from std_msgs.msg import String, Bool
import threading
from queue import Queue
from moveit_msgs.msg import Constraints, JointConstraint
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanningComponent, TrajectoryExecutionManager, PlanRequestParameters
from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.controller_manager import ExecutionStatus
import numpy as np
WORKSPACE_MIN_CORNER = make_vector3(x=-1.0, y=-1.0, z=-1.0)
WORKSPACE_MAX_CORNER = make_vector3(x=1.0, y=1.0, z=1.0)

CHESSPIECE_CLEARANCE = 0.2  # Clearance above the chess piece

class MoveItCommander(Node):

    def __init__(self):
        super().__init__('ur_moveit_commander')

        #Params
        self.declare_parameter("moveit_cpp_cfg", get_package_share_directory("ur_chess") + "/config/moveit_cpp.yaml")
        self.declare_parameter("moveit_cpp_srdf", get_package_share_directory("ur_chess") + "/config/moveit_cpp.srdf")
        self.declare_parameter("moveit_controller_cfg",get_package_share_directory("ur_moveit_config") + "/config/moveit_controllers.yaml")

        self.get_logger().info("Setting up moveit config.")
        moveit_config_builder = MoveItConfigsBuilder("ur")
        moveit_config_builder.moveit_cpp(self.get_parameter("moveit_cpp_cfg").value) 
        moveit_config_builder.robot_description_semantic(self.get_parameter("moveit_cpp_srdf").value)
        moveit_config_builder.trajectory_execution(self.get_parameter("moveit_controller_cfg").value)
        moveit_config_builder.planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        moveit_config_dict = moveit_config_builder.to_moveit_configs().to_dict()
        moveit_config_dict.update({'use_sim_time' : True})
        file = create_params_file_from_dict(moveit_config_dict, "/**")

        self.get_logger().info("Creating MoveItPy instance")
        self.ur_commander = MoveItPy(node_name="moveit_py", launch_params_filepaths=[file])
        self.ur_commander_arm: PlanningComponent = self.ur_commander.get_planning_component("ur_manipulator")
        self.ur_commander_gripper: PlanningComponent = self.ur_commander.get_planning_component("gripper")

            
        robot_model = self.ur_commander.get_robot_model()
        self.robot_state = RobotState(robot_model)
        self.planning_scene_monitor = self.ur_commander.get_planning_scene_monitor()

        self.execution_manager: TrajectoryExecutionManager = self.ur_commander.get_trajectory_execution_manager()
        self.execution_done = threading.Event()

        # Create a publisher for the feedback messages
        self.move_res_pub = self.create_publisher(URChessMoveResult,  '/ur_chess/move_result', 10)
        self.control_sub = self.create_subscription(
            String,
            '/ur_chess/game_control',
            self.control_callback,
            10
        )
        # Create a service
        self.create_service(URChessMovePiece, 'ur_chess/move_piece', self.move_piece_callback)
        self.get_logger().info('Service ur_chess/move_piece is ready.')

        # Use ReentrantCallbackGroup for managing callbacks concurrently
        self._callback_group = ReentrantCallbackGroup()

        
        self.get_logger().info(TextColor.color_text('MoveItCommander initialized', TextColor.OKCYAN))
    
    def control_callback(self, msg):
        command = msg.data.lower()
        if command == 'play':
            self.ur_commander_arm.set_start_state_to_current_state()
            self.ur_commander_arm.set_goal_state(configuration_name="chess_home")
            self.plan_and_execute(self.ur_commander_arm)
        elif command == 'pause':
            pass
        elif command == 'stop':
            pass
        else:
            self.get_logger().warn(f'Unknown game control command: {command}')  
                
    def plan_and_execute(self, planning_component, single_plan_parameters=None, multi_plan_parameters=None, sleep_time=0.0):
        """Plan and execute trajectory (blocking)."""
        for _ in range(10):
            self.get_logger().info("Planning trajectory")
            if multi_plan_parameters is not None:
                plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
            elif single_plan_parameters is not None:
                plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
            else:
                plan_result = planning_component.plan()

            if plan_result:
                self.get_logger().info("Executing trajectory")
                robot_trajectory = plan_result.trajectory
                self.ur_commander.execute(robot_trajectory, controllers=[])
                time.sleep(sleep_time)
                break
            else:
                self.get_logger().error("Planning failed")
    
    def plan_trajectory(self, planning_component, single_plan_parameters=None, multi_plan_parameters=None) -> Optional[MotionPlanResponse]:
        """Plan trajectory (non-blocking)."""
        for _ in range(10):
            self.get_logger().info("Planning trajectory")
            if multi_plan_parameters is not None:
                plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
            elif single_plan_parameters is not None:
                plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
            else:
                plan_result = planning_component.plan()

            if plan_result:
                self.get_logger().info("Planning succeeded")
                return plan_result
            else:
                self.get_logger().error("Planning failed")
        return None
                
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
            goal1 = self.create_pc_from_point(start_x, start_y, start_z+0.2)
            plan_result = self.plan_trajectory(goal1)#, goal_str='Move above piece')
            traj: RobotTrajectory = plan_result.trajectory
            traj_msg = traj.get_robot_trajectory_msg()
            self.execution_manager.push(traj_msg,controllers=[])

            final_positions = traj_msg.joint_trajectory.points[-1].positions
            # Build a RobotState for the final pose
            goal2 = self.create_pc_from_point(start_x, start_y, start_z)
            state = RobotState(self.ur_commander.get_robot_model())
            state.set_joint_group_positions("ur_manipulator", np.array(final_positions))
            goal2.set_start_state(configuration_name=None, robot_state=state)
            plan_result = self.plan_trajectory(goal2)#, goal_str='Move above piece')
            traj: RobotTrajectory = plan_result.trajectory
            msg2 = traj.get_robot_trajectory_msg()

            self.execution_manager.push(msg2,controllers=[])

            
            self.execution_manager.execute(callback=self.success_callback)#, auto_clear=True)
            # self.execution_manager.wait_for_trajectory_completion()
            # self.execution_manager.stop_execution()
            #self.execution_manager.stop_execution()
            #self.plan_and_execute(goal1, sleep_time=1.0)#'Move above piece'
            # #At the start piece
            # goal2 = self.create_pc_from_point(start_x, start_y, start_z)
            # self.plan_and_execute(goal2)#, goal_str='Grab piece')
            
            # self.ur_commander_gripper.set_start_state_to_current_state()
            # self.ur_commander_gripper.set_goal_state(configuration_name="closed")
            # self.plan_and_execute(self.ur_commander_gripper, sleep_time=2.0)
            
            # goal1 = self.create_pc_from_point(start_x, start_y, start_z+CHESSPIECE_CLEARANCE)
            # self.plan_and_execute(goal1)#'Move above piece'
            # #Above the end piece
            # goal3 = self.create_pc_from_point(end_x, end_y, end_z+CHESSPIECE_CLEARANCE)
            # self.plan_and_execute(goal3, sleep_time=1.0)#, goal_str='Move to goal position')

            # #At the end piece
            # goal4 = self.create_pc_from_point(end_x, end_y, end_z)
            # self.plan_and_execute(goal4)#, goal_str='Place piece')
            # self.ur_commander_gripper.set_start_state_to_current_state()
            # self.ur_commander_gripper.set_goal_state(configuration_name="open")
            # self.plan_and_execute(self.ur_commander_gripper, sleep_time=2.0)

            # goal3 = self.create_pc_from_point(end_x, end_y, end_z+CHESSPIECE_CLEARANCE)
            # self.plan_and_execute(goal3)#, goal_str='Move to goal position')
            
            # self.ur_commander_arm.set_start_state_to_current_state()
            # self.ur_commander_arm.set_goal_state(configuration_name="chess_home")
            # self.plan_and_execute(self.ur_commander_arm)

            response.success = True
            response.message = 'Success'
            return response
        except Exception as e:
            self.get_logger().error(f'Error in move_piece_callback: {e}')
            response.message = f'Error in move_piece_callback: {e}'
            response.success = False
            return response

    def success_callback(self, msg: ExecutionStatus):
        print(msg.status)
    
    def create_pc_from_point(self,x, y, z):
        """Create a planning component from a point."""
        ur_pc = self.ur_commander_arm
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position = make_point(x, y, z)
        pose.pose.orientation = make_quaternion(0.70710678118, 0.70710678118, 0.0, 0.0)
        ur_pc.set_start_state_to_current_state()
        ur_pc.set_goal_state(pose_stamped_msg=pose, pose_link="end_effector_link")
        ur_pc.set_path_constraints(self.create_constraints())
        return ur_pc
        
    def create_constraints(self):
        # Create a Constraints message
        joint_constraints = Constraints()
        joint_constraints.name = "constaints"
        
        # Define a joint constraint for 'joint_1'
        # jc1 = JointConstraint()
        # jc1.joint_name = "elbow_joint"
        # jc1.position = math.radians(-50)  # Desired joint position in radians
        # jc1.tolerance_above = math.radians(50)   # Upper tolerance in radians
        # jc1.tolerance_below = math.radians(80)  # Lower tolerance in radians
        # jc1.weight = 1.0  # Importance of this constraint

        jc2 = JointConstraint()
        jc2.joint_name = "shoulder_lift_joint"
        jc2.position = math.radians(-75)
        jc2.tolerance_above = math.radians(75)
        jc2.tolerance_below = math.radians(75)
        jc2.weight = 1.0
        
        jc3 = JointConstraint()
        jc3.joint_name = "wrist_1_joint"
        jc3.position = math.radians(-110)
        jc3.tolerance_above = math.radians(100)
        jc3.tolerance_below = math.radians(70)
        jc3.weight = 1.0
        
        jc4 = JointConstraint()
        jc4.joint_name = "wrist_2_joint"
        jc4.position = math.radians(90)
        jc4.tolerance_above = math.radians(100)
        jc4.tolerance_below = math.radians(100)
        jc4.weight = 1.0
        
        jc2 = JointConstraint()
        jc2.joint_name = "wrist_3_joint"
        jc2.position = math.radians(90)
        jc2.tolerance_above = math.radians(90)
        jc2.tolerance_below = math.radians(90)
        jc2.weight = 1.0
        
        jc5 = JointConstraint()
        jc5.joint_name = "shoulder_pan_joint"
        jc5.position = math.radians(90)
        jc5.tolerance_above = math.radians(45)
        jc5.tolerance_below = math.radians(45)
        jc5.weight = 1.0
        # Add the joint constraint to the Constraints message
        #joint_constraints.joint_constraints.append(jc1)
        joint_constraints.joint_constraints.append(jc2)
        joint_constraints.joint_constraints.append(jc3)
        joint_constraints.joint_constraints.append(jc4)
        joint_constraints.joint_constraints.append(jc5)
        
        return joint_constraints
        
def main(args=None):
    rclpy.init(args=args)
    commander = MoveItCommander()
    
    # Use a single threaded executor to handle the callbacks
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(commander)
    executor.spin()

    commander.destroy_node()
    rclpy.shutdown()
