import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, MoveGroupSequence
from moveit_msgs.action._move_group import MoveGroup_FeedbackMessage
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters, JointConstraint, MoveItErrorCodes, MotionSequenceItem
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
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
from moveit.planning import MoveItPy, PlanningComponent, TrajectoryExecutionManager, PlanRequestParameters, PlanningSceneMonitor
from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.controller_manager import ExecutionStatus
import numpy as np
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import AttachedCollisionObject
import chess
from moveit_msgs.msg import ObjectColor
from std_msgs.msg import ColorRGBA
CHESSPIECE_CLEARANCE = 0.2  # Clearance above the chess piece

class ChessPiece():
    def __init__(self, current_pos, color, start_pose: Pose, size):
        super().__init__()
        self.obj = CollisionObject()
        self.obj.id = current_pos
        self.obj.header.frame_id = "base_link"
        self.color = color
        if self.color == "white":
            clr = ColorRGBA()
            clr.r = 1.0
            clr.g = 1.0
            clr.b = 1.0
            clr.a = 1.0
        elif self.color == "black":
            clr = ColorRGBA()
            clr.r = 0.0
            clr.g = 0.0
            clr.b = 0.0
            clr.a = 1.0
        else:
            raise ValueError("Invalid piece color. Use 'white' or 'black'.")
        self.object_color = ObjectColor(id=self.obj.id, color=clr)
        self.obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[size[0], size[1]])]
        self.obj.primitive_poses.append(start_pose)
    
    def add_to_scene(self, scene):
        """Add the chess piece to the planning scene."""
        self.obj.operation = CollisionObject.ADD
        scene.apply_collision_object(self.obj, self.object_color)
        scene.current_state.update()    
    
    def remove_from_scene(self, scene):
        """Remove the chess piece from the planning scene."""
        self.obj.operation = CollisionObject.REMOVE
        scene.apply_collision_object(self.obj)
        scene.current_state.update()
        
    def attach_to_robot(self, scene):
        """Attach the chess piece to the robot."""
        attached_collision_object = AttachedCollisionObject()
        attached_collision_object.link_name = "end_effector_link"
        attached_collision_object.object = self.obj
        attached_collision_object.touch_links = ["rh_p12_rn_l1", "rh_p12_rn_l2", "rh_p12_rn_r1", "rh_p12_rn_r2"]
        scene.process_attached_collision_object(attached_collision_object)
        scene.current_state.update()
        
    def detach_from_robot(self, scene, new_pos):
        """Detach the chess piece from the robot. This also needs the pos to be updated."""
        #Remove from world
        self.obj.operation = CollisionObject.REMOVE
        detach_object = AttachedCollisionObject()
        detach_object.link_name = "end_effector_link"
        detach_object.object = self.obj
        scene.process_attached_collision_object(detach_object)
        self.obj.id = new_pos
        #Reinsert into world
        self.obj.operation = CollisionObject.ADD
        scene.apply_collision_object(self)
        scene.current_state.update()
        
        
class MoveItCommander(Node):

    def __init__(self):
        super().__init__('ur_moveit_commander')

        #Params
        self.declare_parameter("moveit_cpp_cfg", get_package_share_directory("ur_chess") + "/config/moveit_cpp.yaml")
        self.declare_parameter("moveit_cpp_srdf", get_package_share_directory("ur_chess") + "/config/moveit_cpp.srdf")
        self.declare_parameter("moveit_controller_cfg",get_package_share_directory("ur_moveit_config") + "/config/moveit_controllers.yaml")
        self.declare_parameter("board_layout", get_package_share_directory("ur_chess") + "/config/board_layout.yaml")

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
        self.planning_scene_monitor: PlanningSceneMonitor = self.ur_commander.get_planning_scene_monitor()

        self.execution_manager: TrajectoryExecutionManager = self.ur_commander.get_trajectory_execution_manager()
        
        
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

        self.create_chess_pieces()
        
        # with self.planning_scene_monitor.read_only() as scene:
        #     piece = self.chess_pieces[4]
        #     attached_collision_object = AttachedCollisionObject()
        #     attached_collision_object.link_name = "end_effector_link"
        #     attached_collision_object.object = piece
        #     attached_collision_object.touch_links = ["rh_p12_rn_l1", "rh_p12_rn_l2", "rh_p12_rn_r1", "rh_p12_rn_r2"]

        #     scene.process_attached_collision_object(attached_collision_object)
        #     scene.current_state.update()  # Important to ensure the scene is updated
        self.get_logger().info(TextColor.color_text('MoveItCommander initialized', TextColor.OKCYAN))
    
    def create_chess_pieces(self):
        """Create the chess pieces to be used in the planning scene."""
        # Load the board layout from the YAML file
        board_layout_path = self.get_parameter("board_layout").value
        with open(board_layout_path, 'r') as f:
            board_layout = yaml.safe_load(f)
        corner_a1 = board_layout['a1']
        tile_size = board_layout['tile_size']
        board_orientation = board_layout['orientation']

        # Map from file/rank to board indices
        files = 'abcdefgh'
        size = [tile_size * 1.5, tile_size / 4]

        board = chess.Board()
        self.chess_pieces = []
        with self.planning_scene_monitor.read_only() as scene:
            for square in chess.SQUARES:
                piece = board.piece_at(square)
                if piece:
                    file = chess.square_file(square)
                    rank = chess.square_rank(square)
                    file_letter = files[file]

                    color_name = 'white' if piece.color == chess.WHITE else 'black'

                    piece_id = f"{file_letter}{rank}"

                    pose = Pose()
                    pose.position.x = corner_a1[0] + board_orientation[0] * ((file * tile_size) + tile_size / 2)
                    pose.position.y = corner_a1[1] + board_orientation[1] * ((rank * tile_size) + tile_size / 2)
                    pose.position.z = corner_a1[2]

                    self.chess_pieces.append(ChessPiece(piece_id, color_name, pose, size).add_to_scene(scene))
                    
            #create a table
            table = CollisionObject()
            table.id = "table"
            table.header.frame_id = "base_link"
            table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.5, 0.5, 0.05])]
            #TODO make this dynamic
            table.primitive_poses.append(Pose(position=make_point(0.0, -0.4, 0.25)))
            table.operation = CollisionObject.ADD
            table_color = ObjectColor(id=table.id, color=ColorRGBA(r=0.7, g=0.7, b=0.1, a=1.0))
            scene.apply_collision_object(table, table_color)
            scene.current_state.update()  # Important to ensure the scene is updated
    
            
    def control_callback(self, msg):
        command = msg.data.lower()
        if command == 'play':
            last_exec_status = self.execution_manager.get_last_execution_status().status
            self.ur_commander_arm.set_start_state_to_current_state()
            self.ur_commander_arm.set_goal_state(configuration_name="chess_home")
            self.plan_and_execute(self.ur_commander_arm, sleep_time=2)
            if last_exec_status == "PREEMPTED" and len(self.trajectories) > 0:
                self.get_logger().info("Execution was preempted, restarting previous trajectory")
                self.push_and_execute(self.trajectories)
        elif command == 'pause':
            self.get_logger().info("Pausing execution")
            self.execution_manager.stop_execution()
        elif command == 'stop':
            self.get_logger().info("Stopping execution")
            self.execution_manager.stop_execution()
            self.trajectories.clear()
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
        for _ in range(5):
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
    
    def push_and_execute(self, trajectory: list[RobotTrajectory]):
        """Push and execute trajectory (non-blocking)."""
        for traj in trajectory:
            self.execution_manager.push(traj, controllers=[])
        self.execution_manager.execute(callback=self.motion_callback, auto_clear=True)
    
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
            
            
            self.trajectories = self.plan_chess_move_trajectory(start_x, start_y, start_z, end_x, end_y, end_z)
            #Start execution with pickup
            self.push_and_execute(self.trajectories)
            
            response.success = True
            response.message = 'Planned trajectories. Await result on /ur_chess/move_result.'
            
            return response
        except Exception as e:
            self.get_logger().error(f'Error in move_piece_callback: {e}')
            response.message = f'Error in move_piece_callback: {e}'
            response.success = False
            return response     

    def motion_callback(self, msg: ExecutionStatus):
        if msg.status == 'SUCCEEDED':
            self.get_logger().info("Arrived home successfully")
            self.move_res_pub.publish(URChessMoveResult(success=True, message="Arrived home successfully"))
        elif msg.status == 'ABORTED':
            self.get_logger().error("Execution aborted")
            self.move_res_pub.publish(URChessMoveResult(success=False, message="Execution aborted"))
        elif msg.status == 'PREEMPTED':
            self.get_logger().warn("Process preempted")
            self.move_res_pub.publish(URChessMoveResult(success=False, message="Process preempted"))
    
    def plan_chess_move_trajectory(self, start_x, start_y, start_z, end_x, end_y, end_z):

        joint_group = "ur_manipulator"
        model = self.ur_commander.get_robot_model()
        trajectories = []

        def plan(pc, last_joint_positions=None):
            if last_joint_positions is not None:
                state = RobotState(model)
                state.set_joint_group_positions(joint_group, np.array(last_joint_positions))
                pc.set_start_state(configuration_name=None, robot_state=state)
            plan_result = self.plan_trajectory(pc)
            traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
            return traj_msg, traj_msg.joint_trajectory.points[-1].positions

        def plan_gripper(opening: bool):
            pc = self.create_gripper_pc(opening)
            plan_result = self.plan_trajectory(pc)
            return plan_result.trajectory.get_robot_trajectory_msg()
        
        joint_pos = None

        # 1. Above start piece
        pc1 = self.create_pc_from_point(start_x, start_y, start_z + CHESSPIECE_CLEARANCE)
        msg1, joint_pos = plan(pc1)
        trajectories.append(msg1)

        # Gripper open (ensure it's open before pickup)
        msg_grip_open_1 = plan_gripper(opening=True)
        trajectories.append(msg_grip_open_1)

        # 2. At start piece
        pc2 = self.create_pc_from_point(start_x, start_y, start_z)
        msg2, joint_pos = plan(pc2, joint_pos)
        trajectories.append(msg2)

        # Gripper close (grasp piece)
        msg_grip_close = plan_gripper(opening=False)
        trajectories.append(msg_grip_close)

        # 3. Back above start piece
        pc3 = self.create_pc_from_point(start_x, start_y, start_z + CHESSPIECE_CLEARANCE)
        msg3, joint_pos = plan(pc3, joint_pos)
        trajectories.append(msg3)

        # 4. Above end piece
        pc4 = self.create_pc_from_point(end_x, end_y, end_z + CHESSPIECE_CLEARANCE)
        msg4, joint_pos = plan(pc4, joint_pos)
        trajectories.append(msg4)

        # 5. At end piece
        pc5 = self.create_pc_from_point(end_x, end_y, end_z)
        msg5, joint_pos = plan(pc5, joint_pos)
        trajectories.append(msg5)

        # Gripper open (release piece)
        msg_grip_open_2 = plan_gripper(opening=True)
        trajectories.append(msg_grip_open_2)
        
        # 6. Back above end piece
        pc6 = self.create_pc_from_point(end_x, end_y, end_z + CHESSPIECE_CLEARANCE)
        msg6, joint_pos = plan(pc6, joint_pos)
        trajectories.append(msg6)

        # 7. Back to home position
        ur_pc = self.ur_commander_arm
        ur_pc.set_path_constraints(self.create_constraints())
        state = RobotState(model)
        state.set_joint_group_positions(joint_group, np.array(joint_pos))
        ur_pc.set_start_state(configuration_name=None, robot_state=state)
        ur_pc.set_goal_state(configuration_name="chess_home")
        plan_result = self.plan_trajectory(ur_pc)
        msg7 = plan_result.trajectory.get_robot_trajectory_msg()
        trajectories.append(msg7)
        
        return trajectories
    
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
    
    def create_gripper_pc(self, open: bool):
        """Create a planning component for the gripper."""
        ur_pc = self.ur_commander_gripper
        ur_pc.set_start_state_to_current_state()
        if open:
            ur_pc.set_goal_state(configuration_name="open")
        else:
            ur_pc.set_goal_state(configuration_name="closed")
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
