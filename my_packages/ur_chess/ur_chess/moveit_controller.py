import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, MoveGroupSequence
from moveit_msgs.action._move_group import MoveGroup_FeedbackMessage
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, WorkspaceParameters, JointConstraint, MoveItErrorCodes, MotionSequenceItem
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from ur_chess.misc.math_helpers import make_vector3, make_quaternion, make_point, Waypoint
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
    def __init__(self, start_pos, color, start_pose: Pose, size):
        self.id = start_pos
        self.current_pos = start_pos
        self.color = color
        self.size = size
        self.pose = start_pose

        if self.color == "white":
            clr = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        elif self.color == "black":
            clr = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        else:
            raise ValueError("Invalid piece color. Use 'white' or 'black'.")

        self.object_color = ObjectColor(id=self.id, color=clr)

    def _create_collision_object(self, pose: Pose):
        obj = CollisionObject()
        obj.id = self.id
        obj.header.frame_id = "base_link"
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=self.size)]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def add_to_scene(self, planning_scene_monitor):
        """Add the chess piece to the planning scene."""
        with planning_scene_monitor.read_write() as scene:
            obj = self._create_collision_object(self.pose)
            scene.apply_collision_object(obj, self.object_color)
            scene.current_state.update()

    def remove_from_scene(self, planning_scene_monitor):
        """Remove the chess piece from the planning scene."""
        with planning_scene_monitor.read_write() as scene:
            obj = CollisionObject()
            obj.id = self.id
            obj.header.frame_id = "base_link"
            obj.operation = CollisionObject.REMOVE
            scene.apply_collision_object(obj)
            scene.current_state.update()

    def attach_to_robot(self, planning_scene_monitor):
        """Attach the chess piece to the robot (remove from world first)."""
        with planning_scene_monitor.read_write() as scene:
            # Remove from world if present
            remove_obj = CollisionObject()
            remove_obj.id = self.id
            remove_obj.header.frame_id = "base_link"
            remove_obj.operation = CollisionObject.REMOVE
            scene.apply_collision_object(remove_obj)

            # Create new object to attach
            attach_obj = self._create_collision_object(self.pose)
            attached = AttachedCollisionObject()
            attached.link_name = "end_effector_link"
            attached.object = attach_obj
            attached.object.operation = CollisionObject.ADD
            attached.touch_links = ["rh_p12_rn_l1", "rh_p12_rn_l2", "rh_p12_rn_r1", "rh_p12_rn_r2"]
            scene.process_attached_collision_object(attached)
            scene.current_state.update()

    def detach_from_robot(self, planning_scene_monitor, new_pos_uci, new_pos_xyz):
        """Detach and reinsert piece at new world pose."""
        with planning_scene_monitor.read_write() as scene:
            # Detach
            dummy = CollisionObject()
            dummy.id = self.id
            dummy.header.frame_id = "base_link"
            dummy.operation = CollisionObject.REMOVE

            detach_obj = AttachedCollisionObject()
            detach_obj.link_name = "end_effector_link"
            detach_obj.object = dummy
            scene.process_attached_collision_object(detach_obj)

            # Add to world at new pose
            new_pose = Pose()
            new_pose.position = make_point(new_pos_xyz[0], new_pos_xyz[1], self.pose.position.z)
            new_pose.orientation.w = 1.0  # Default orientation

            world_obj = self._create_collision_object(new_pose)
            scene.apply_collision_object(world_obj, self.object_color)
            self.pose = new_pose
            self.current_pos = new_pos_uci

            scene.current_state.update()
        
class MoveItCommander(Node):

    def __init__(self):
        super().__init__('ur_moveit_commander')

        #Params
        self.declare_parameter("moveit_cpp_cfg", get_package_share_directory("ur_chess") + "/config/moveit_cpp.yaml")
        self.declare_parameter("moveit_cpp_srdf", get_package_share_directory("ur_chess") + "/config/moveit_cpp.srdf")
        self.declare_parameter("moveit_controller_cfg",get_package_share_directory("ur_moveit_config") + "/config/moveit_controllers.yaml")
        self.declare_parameter("board_layout", get_package_share_directory("ur_chess") + "/config/board_layout.yaml")

        # Load the board layout from the YAML file
        board_layout_path = self.get_parameter("board_layout").value
        with open(board_layout_path, 'r') as f:
            board_layout = yaml.safe_load(f)
        self.corner_a1 = board_layout['a1']
        self.tile_size = board_layout['tile_size']
        self.board_orientation = board_layout['orientation']

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
        
        # Create 3d scene
        self.get_logger().info("Creating objects..")
        self.create_chess_pieces()
        self.white_captures = 0
        self.black_captures = 0

        # Use ReentrantCallbackGroup for managing callbacks concurrently
        self._callback_group = ReentrantCallbackGroup()
        
        # Create a publisher for the feedback messages
        self.move_res_pub = self.create_publisher(URChessMoveResult,  '/ur_chess/move_result', 10, callback_group=self._callback_group)
        # Create a subscriber for the control messages 
        self.control_sub = self.create_subscription(
            String,
            '/ur_chess/game_control',
            self.control_callback,
            10,
            callback_group=self._callback_group
        )
        self.create_service(URChessMovePiece, '/ur_chess/move_piece', self.move_piece_callback, callback_group=self._callback_group)
        
        self.get_logger().info("Created service for move_piece")

        self.get_logger().info(TextColor.color_text('MoveItCommander initialized', TextColor.OKCYAN))
    
    def create_chess_pieces(self):
        """Create the chess pieces to be used in the planning scene."""

        corner_a1 = self.corner_a1
        tile_size = self.tile_size
        board_orientation = self.board_orientation

        # Map from file/rank to board indices
        files = 'abcdefgh'
        size = [tile_size * 1.5, tile_size / 4]

        board = chess.Board()
        self.chess_pieces = []
        for square in chess.SQUARES:
            piece = board.piece_at(square)
            if piece:
                file = chess.square_file(square)
                rank = chess.square_rank(square)
                file_letter = files[file]

                color_name = 'white' if piece.color == chess.WHITE else 'black'

                piece_id = f"{file_letter}{rank+1}"

                pose = Pose()
                pose.position.x = corner_a1[0] + board_orientation[0] * ((file * tile_size) + tile_size / 2)
                pose.position.y = corner_a1[1] + board_orientation[1] * ((rank * tile_size) + tile_size / 2)
                pose.position.z = corner_a1[2]

                chess_piece = ChessPiece(piece_id, color_name, pose, size)
                chess_piece.add_to_scene(self.planning_scene_monitor)
                self.chess_pieces.append(chess_piece)
                
        self.get_logger().info("Creating table")
        with self.planning_scene_monitor.read_write() as scene:            
            #create a table
            table = CollisionObject()
            table.id = "table"
            table.header.frame_id = "base_link"
            table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.5, 0.5, 0.05])]
            #TODO make this dynamic
            table.primitive_poses.append(Pose(position=make_point(-0.125, -0.425, 0.248)))
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
            # if last_exec_status == "PREEMPTED" and len(self.trajectories) > 0:
            #     self.get_logger().info("Execution was preempted, restarting previous trajectory")
            #     self.push_and_execute(self.trajectories)
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
                raise RuntimeError("Planning failed")
        return None
    
    #! unused        
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
    
    def move_piece_callback(self, request:URChessMovePiece.Request, response:URChessMovePiece.Response):
        """Callback for the move_piece service."""
        self.get_logger().info('Received request to move to position')
        try:
            if request.is_capture:
                self.get_logger().info('Capture move detected')
                piece = self.find_chess_piece(request.uci[2:4])
                if piece is not None:
                    start_x, start_y, start_z = self.square_to_xyz(request.uci[2:4])
                    end_pos, end_place = self.capture_to_xyz(request.capturer_color)
                    end_x, end_y, end_z = end_pos
                    trajectories = self.plan_grab_piece(start_x,start_y,start_z)
                    for traj in trajectories:
                        self.execution_manager.push(traj, controllers=[])
                    self.execution_manager.execute_and_wait(auto_clear=True)
                    
                    piece.attach_to_robot(self.planning_scene_monitor)

                    trajectories = self.plan_move_piece(start_x, start_y, start_z, end_x, end_y, end_z)
                    for traj in trajectories:
                        self.execution_manager.push(traj, controllers=[])
                    self.execution_manager.execute_and_wait(auto_clear=True)

                    piece.detach_from_robot(self.planning_scene_monitor, end_place, [end_x, end_y])    
                    
                    trajectories = self.plan_move_home(end_x, end_y, end_z)
                    for traj in trajectories:
                        self.execution_manager.push(traj, controllers=[])
                    self.execution_manager.execute_and_wait(auto_clear=True)
                else:
                    self.get_logger().warn(f'No piece found at {request.uci[2:4]} to capture')
            
            # Get the start and end positions from the request
            start_x, start_y, start_z = self.square_to_xyz(request.uci[:2])
            end_x, end_y, end_z = self.square_to_xyz(request.uci[2:4])
            piece = self.find_chess_piece(request.uci[:2])

            trajectories = self.plan_grab_piece(start_x,start_y,start_z)
            for traj in trajectories:
                self.execution_manager.push(traj, controllers=[])
            self.execution_manager.execute_and_wait(auto_clear=True)
            
            piece.attach_to_robot(self.planning_scene_monitor)

            trajectories = self.plan_move_piece(start_x, start_y, start_z, end_x, end_y, end_z)
            for traj in trajectories:
                self.execution_manager.push(traj, controllers=[])
            self.execution_manager.execute_and_wait(auto_clear=True)

            piece.detach_from_robot(self.planning_scene_monitor, request.uci[2:4], [end_x, end_y])    
            
            trajectories = self.plan_move_home(end_x, end_y, end_z)
            for traj in trajectories:
                self.execution_manager.push(traj, controllers=[])
            self.execution_manager.execute_and_wait(auto_clear=True)

            response.success = True
            response.message = 'Succesful movement.'
            
            return response
        except Exception as e:
            self.get_logger().error(f'Error in move_piece_callback: {e}')
            response.message = f'Error in move_piece_callback: {e}'
            response.success = False
            return response     


    def find_chess_piece(self, square: str) -> Optional[ChessPiece]:
        """Find the chess piece at the given square."""
        for piece in self.chess_pieces:
            if piece.current_pos == square:
                return piece    
        return None
    
    
    def capture_to_xyz(self, capturer_color: str):
        if capturer_color == True: #white
            self.white_captures += 1
            if self.white_captures < 8:
                file = 'i'
            else:
                file = 'j'
            rank = self.white_captures % 9
            return self.square_to_xyz(file + str(rank)) , f'{file}{rank}'
        elif capturer_color == False: #black
            self.black_captures += 1
            if self.black_captures < 8:
                #Backtick file its before a in unicode
                file = '`'
            else:
                file = '_'
            rank = (9-self.black_captures) % 9
            return self.square_to_xyz(file + str(rank)) , f'{file}{rank}'
        else:
            raise ValueError("Invalid piece color. Use True for white or False for black.")
            
    def square_to_xyz(self, square: str):
        # square: 'a1'..'h8'
        file = ord(square[0]) - ord('a')
        rank = int(square[1]) - 1
        # center offset: add half tile
        x = self.corner_a1[0] + self.board_orientation[0]*(file * self.tile_size + self.tile_size/2)
        y = self.corner_a1[1] + self.board_orientation[1]*(rank * self.tile_size + self.tile_size/2)
        # z could be constant above board
        z = self.corner_a1[2] + self.tile_size*0.75
        return round(x, 3), round(y, 3), round(z, 3)
    
    def plan_grab_piece(self,start_x, start_y, start_z):
        waypoints = []
        waypoints.append(Waypoint(position=make_point(start_x,start_y,start_z+CHESSPIECE_CLEARANCE),gripper="open")) #above piece
        waypoints.append(Waypoint(position=make_point(start_x,start_y,start_z),gripper="close")) #at the piece
        trajectories = self.plan_move_trajectory(waypoints)
        return trajectories
    
    def plan_move_piece(self,start_x,start_y,start_z,end_x,end_y,end_z):
        waypoints = []
        waypoints.append(Waypoint(position=make_point(start_x,start_y,start_z)))
        waypoints.append(Waypoint(position=make_point(start_x,start_y,start_z+CHESSPIECE_CLEARANCE)))
        waypoints.append(Waypoint(position=make_point(end_x,end_y,end_z+CHESSPIECE_CLEARANCE)))
        waypoints.append(Waypoint(position=make_point(end_x,end_y,end_z), gripper="open"))
        trajectories = self.plan_move_trajectory(waypoints)
        return trajectories

    def plan_move_home(self,end_x,end_y,end_z):
        waypoints = []
        waypoints.append(Waypoint(position=make_point(end_x,end_y,end_z+CHESSPIECE_CLEARANCE)))
        waypoints.append(Waypoint(named_position="chess_home"))
        trajectories = self.plan_move_trajectory(waypoints)

        return trajectories
    
    def plan_move_trajectory(self, waypoints : tuple[Waypoint]):
        """Plan a trajectory based on the given positions"""
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

        def plan_gripper(pc, last_joint_positions=None):
            if last_joint_positions is not None:
                state = RobotState(model)
                state.set_joint_group_positions('gripper', np.array(last_joint_positions))
                pc.set_start_state(configuration_name=None, robot_state=state)
            plan_result = self.plan_trajectory(pc)
            traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
            return traj_msg , traj_msg.joint_trajectory.points[-1].positions
        
        last_joint_positions = None
        gripper_last_pos = None
        
        for wp in waypoints:
            if wp.named_position:
                pc = self.ur_commander_arm
                pc.set_goal_state(configuration_name=wp.named_position)
            else:
                pc = self.create_pc_from_point(wp.position.x, wp.position.y, wp.position.z)
            msg, last_joint_positions = plan(pc, last_joint_positions)
            trajectories.append(msg)
            
            if wp.gripper:
                gripper_pc = self.create_gripper_pc(open=(wp.gripper == "open"))
                msg_gripper, gripper_last_pos = plan_gripper(gripper_pc, gripper_last_pos)
                trajectories.append(msg_gripper)
        
        return trajectories
            
    def create_pc_from_point(self,x, y, z):
        """Create a planning component from a point."""
        ur_pc = self.ur_commander_arm
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position = make_point(x, y, z)
        pose.pose.orientation = make_quaternion(0.924, 0.383, 0.0, 0.0)
        ur_pc.set_start_state_to_current_state()
        ur_pc.set_goal_state(pose_stamped_msg=pose, pose_link="end_effector_link")
        ur_pc.set_path_constraints(self.create_constraints())
        return ur_pc
    
    def create_gripper_pc(self, open: bool):
        """Create a planning component for the gripper."""
        ur_pc = self.ur_commander_gripper
        ur_pc.set_start_state_to_current_state()
        if open:
            ur_pc.set_goal_state(configuration_name="part_open")
        else:
            ur_pc.set_goal_state(configuration_name="grab_piece")
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

        # jc2 = JointConstraint()
        # jc2.joint_name = "shoulder_lift_joint"
        # jc2.position = math.radians(-75)
        # jc2.tolerance_above = math.radians(75)
        # jc2.tolerance_below = math.radians(75)
        # jc2.weight = 1.0
        
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
        jc2.position = math.radians(135)
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
