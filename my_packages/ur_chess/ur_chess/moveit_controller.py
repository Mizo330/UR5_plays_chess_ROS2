import os
import math
import time
import yaml
from typing import Optional
import numpy as np
import chess

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    WorkspaceParameters,
    JointConstraint,
    MoveItErrorCodes,
    MotionSequenceItem
)
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.utils import create_params_file_from_dict
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanningComponent,
    TrajectoryExecutionManager,
    PlanRequestParameters,
    PlanningSceneMonitor
)
from moveit.core.planning_interface import MotionPlanResponse
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.controller_manager import ExecutionStatus

from ur_chess.misc.math_helpers import make_vector3, make_quaternion, make_point, Waypoint
from ur_chess.misc.moveit_col_objects import ChessPiece, create_chess_environment
from ur_chess.misc.text_color import TextColor
from ur_chess_msgs.srv import URChessMovePiece, URChessMoveNamedPos
from ur_chess_msgs.msg import URChessMoveInfo

from ament_index_python import get_package_share_directory
        
        
class MoveItCommander(Node):

    def __init__(self):
        super().__init__('ur_moveit_commander')

        #Params
        self.declare_parameter("moveit_cpp_cfg", get_package_share_directory("ur_chess") + "/config/moveit_cpp.yaml")
        self.declare_parameter("moveit_cpp_srdf", get_package_share_directory("ur_chess") + "/config/moveit_cpp.srdf")
        self.declare_parameter("moveit_controller_cfg",get_package_share_directory("ur_moveit_config") + "/config/moveit_controllers.yaml")
        self.declare_parameter("max_plan_tries", 10)
        self.declare_parameter("a1",[0.1, -0.3, 0.4])
        self.declare_parameter("tile_size",0.0375)
        self.declare_parameter("orientation",[-1, -1])

        self.max_plan_tries = self.get_parameter("max_plan_tries").value

        self.corner_a1 = self.get_parameter("a1").value
        self.tile_size = self.get_parameter("tile_size").value
        self.board_orientation = self.get_parameter("orientation").value
        
        self.chesspiece_clearance = 0.1 # Clearance above the chess piece where the arm should move across.
        
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
        
        # Create a subscriber for the control messages 
        self.create_service(URChessMoveNamedPos, '/ur_chess/move_named_pos', self.move_named_pos_cb, callback_group=self._callback_group)
        self.create_service(URChessMovePiece, '/ur_chess/move_piece', self.move_piece_callback, callback_group=self._callback_group)
        self.get_logger().info("Created service for move_piece")

        self.get_logger().info(TextColor.color_text('MoveItCommander initialized', TextColor.OKCYAN))
    
    def create_chess_pieces(self):
        """Create the chess pieces to be used in the planning scene."""

        a1_0 = self.corner_a1
        tile_size = self.tile_size
        o = self.board_orientation

        self.get_logger().info("Creating environment")
        create_chess_environment(self.planning_scene_monitor,a1_0,o,tile_size)
        # Map from file/rank to board indices
        files = 'abcdefgh'
        size = [tile_size * 1.4, tile_size / 4]

        board = chess.Board()
        self.chess_pieces = []
        self.get_logger().info("Creating chess pieces")
        for square in chess.SQUARES:
            piece = board.piece_at(square)
            if piece:
                file = chess.square_file(square)
                rank = chess.square_rank(square)
                file_letter = files[file]

                color_name = 'white' if piece.color == chess.WHITE else 'black'

                piece_id = f"{file_letter}{rank+1}"

                pose = Pose()
                pose.position.x = a1_0[0] + o[0] * ((file * tile_size) + tile_size / 2)
                pose.position.y = a1_0[1] + o[1] * ((rank * tile_size) + tile_size / 2)
                pose.position.z = a1_0[2] + size[0]/2 #so that the base of the piece is at z_0

                chess_piece = ChessPiece(piece_id, color_name, pose, size)
                chess_piece.add_to_scene(self.planning_scene_monitor)
                self.chess_pieces.append(chess_piece)
            
            
    def move_named_pos_cb(self, request : URChessMoveNamedPos.Request,response:URChessMoveNamedPos.Response):
        named_pos = request.named_posisiton
        self.ur_commander_arm.set_start_state_to_current_state()
        is_valid = self.ur_commander_arm.set_goal_state(configuration_name=named_pos)
        if not is_valid:
            response.success= False
            response.message = f"No predefined joint state found for target name '{named_pos}'"
            return response
        
        result = self.plan_and_execute(self.ur_commander_arm, sleep_time=1)
        if result:
            response.success= True
        else:
            response.success = False
            response.message = 'Failed to plan trajectory. Probably something is blocking the robot.'
        return response
    
    def plan_and_execute(self, planning_component, single_plan_parameters=None, multi_plan_parameters=None, sleep_time=0.0):
        """Plan and execute trajectory (blocking)."""
        for _ in range(self.max_plan_tries):
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
                return True
            else:
                self.get_logger().warn("Planning failed")
                
        self.get_logger().error(f"Failed to plan under {self.max_plan_tries} tries")
        return False
    
    def plan_trajectory(self, planning_component, single_plan_parameters=None, multi_plan_parameters=None) -> Optional[MotionPlanResponse]:
        """Plan trajectory (non-blocking)."""
        for _ in range(self.max_plan_tries):
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
        self.get_logger().info('Received MovePiece request')
        moveinfo = request.moveinfo
        try:
            special_move = moveinfo.special_move
            if special_move != URChessMoveInfo.SPECIAL_MOVE_NONE:
                if special_move == URChessMoveInfo.SPECIAL_MOVE_CAPTURE:
                    self.get_logger().info('Handling capture')
                    self.handle_capture(moveinfo)
                elif special_move == URChessMoveInfo.SPECIAL_MOVE_KINGSIDE_CASTLE: 
                    self.get_logger().info('Handling kingside castle')
                    self.handle_castle(moveinfo)
                elif special_move == URChessMoveInfo.SPECIAL_MOVE_QUEENSIDE_CASTLE: 
                    self.get_logger().info('Handling queenside castle')
                    self.handle_castle(moveinfo)
                elif special_move == URChessMoveInfo.SPECIAL_MOVE_EN_PASSANT: 
                    self.get_logger().info('Handling en passant')
                    self.handle_en_passant(moveinfo)
            
            # Get the start and end positions from the request
            start_pos = self.square_to_xyz(moveinfo.move_uci[:2])
            end_pos = self.square_to_xyz(moveinfo.move_uci[2:4])
            piece = self.find_chess_piece(moveinfo.move_uci[:2])

            self.pick_and_place(start_pos,end_pos,piece,moveinfo.moved_piece)

            response.success = True
            response.message = 'Succesful movement.'
            
            return response
        except Exception as e:
            self.get_logger().error(f'Error in move_piece_callback: {e}')
            response.message = f'Error in move_piece_callback: {e}'
            response.success = False
            return response     

    def pick_and_place(self,start_pos,end_pos,piece:ChessPiece, piece_type=None):
        start_x, start_y, start_z = start_pos
        end_x, end_y, end_z = end_pos
        if piece_type == 'p':
            end_z -= self.tile_size*0.2
            start_z -= self.tile_size*0.2
        trajectories = self.plan_grab_piece(start_x,start_y,start_z)
        for traj in trajectories:
            self.execution_manager.push(traj, controllers=[])
        self.execution_manager.execute_and_wait(auto_clear=True)

        piece.attach_to_robot(self.planning_scene_monitor)

        trajectories = self.plan_move_piece(start_x, start_y, start_z, end_x, end_y, end_z)
        for traj in trajectories:
            self.execution_manager.push(traj, controllers=[])
        self.execution_manager.execute_and_wait(auto_clear=True)
        
        new_square = self.xy_to_square(end_x, end_y)
        piece.detach_from_robot(self.planning_scene_monitor, new_square, [end_x, end_y])    
            
        trajectories = self.plan_move_home(end_x, end_y, end_z)
        for traj in trajectories:
            self.execution_manager.push(traj, controllers=[])
        self.execution_manager.execute_and_wait(auto_clear=True)
        
    def handle_capture(self, moveinfo: URChessMoveInfo):
        piece = self.find_chess_piece(moveinfo.move_uci[2:4])
        if piece is not None:
            start_pos = self.square_to_xyz(moveinfo.move_uci[2:4])
            graveyard_pos, graveyard_sqaure = self.get_graveyard_pos(moveinfo.turn)
            self.pick_and_place(start_pos,graveyard_pos,piece,moveinfo.moved_piece)
        else:
            self.get_logger().warn(f'No piece found at {moveinfo.move_uci[2:4]} to capture')
            
    def handle_castle(self, moveinfo: URChessMoveInfo):
        if moveinfo.turn: #White
            if moveinfo.special_move == URChessMoveInfo.SPECIAL_MOVE_KINGSIDE_CASTLE:
                move_uci = 'h1f1' #move white kinside rook to castling pos
            else: #Queenside
                move_uci = 'a1d1'
        else: #Black
            if moveinfo.special_move == URChessMoveInfo.SPECIAL_MOVE_KINGSIDE_CASTLE:
                move_uci = 'h8f8'
            else:
                move_uci = 'a8d8'
        rook = self.find_chess_piece(move_uci[:2])
        start_pos = self.square_to_xyz(move_uci[:2])
        end_pos = self.square_to_xyz(move_uci[2:4])
        self.pick_and_place(start_pos,end_pos,rook,moveinfo.moved_piece)
        
    def handle_en_passant(self,moveinfo: URChessMoveInfo):
        capturer_end = moveinfo.move_uci[2:4]
        captured_square = capturer_end[0] + str(int(capturer_end[1])-1) if moveinfo.turn else capturer_end[0] + str(int(capturer_end[1])+1)
        piece = self.find_chess_piece(captured_square)
        if piece is not None:
            start_pos = self.square_to_xyz(captured_square)
            graveyard_pos, graveyard_sqaure = self.get_graveyard_pos(moveinfo.turn)
            self.pick_and_place(start_pos,graveyard_pos,piece,moveinfo.moved_piece)
        else:
            self.get_logger().warn(f'No piece found at {moveinfo.move_uci[2:4]} to capture')
        
    def find_chess_piece(self, square: str) -> Optional[ChessPiece]:
        """Find the chess piece at the given square."""
        for piece in self.chess_pieces:
            if piece.current_pos == square:
                return piece    
        return None
    
    def get_graveyard_pos(self, capturer_color: str):
        if capturer_color == True: #white
            if self.white_captures < 8:
                file = 'i'
            else:
                file = 'j'
            rank = 1 + (self.white_captures % 8)
            self.white_captures += 1
            return self.square_to_xyz(file + str(rank)) , f'{file}{rank}'
        elif capturer_color == False: #black
            if self.black_captures < 8:
                #Backtick file its before a in unicode
                file = '`'
            else:
                file = '_'
            rank = 8 - ((self.black_captures) % 8) 
            self.black_captures += 1
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
        z = self.corner_a1[2] + self.tile_size*1.6
        return round(x, 3), round(y, 3), round(z, 3)
    
    def xy_to_square(self, x: float, y: float) -> str:
        dx = x - self.corner_a1[0]
        dy = y - self.corner_a1[1]

        file_pos = dx / self.tile_size if self.board_orientation[0] > 0 else -dx / self.tile_size
        rank_pos = dy / self.tile_size if self.board_orientation[1] > 0 else -dy / self.tile_size

        file = math.floor(file_pos)
        rank = math.floor(rank_pos)

        file_char = chr(ord('a') + file)
        square = f"{file_char}{rank + 1}"
        return square
    
    def plan_grab_piece(self,start_x, start_y, start_z):
        waypoints = []
        waypoints.append(Waypoint(gripper="open"))
        waypoints.append(Waypoint(position=make_point(start_x,start_y,start_z+self.chesspiece_clearance),gripper="open")) #above piece
        waypoints.append(Waypoint(position=make_point(start_x,start_y,start_z),gripper="close")) #at the piece
        trajectories = self.plan_move_trajectory(waypoints)
        return trajectories
    
    def plan_move_piece(self,start_x,start_y,start_z,end_x,end_y,end_z):
        waypoints = []
        waypoints.append(Waypoint(position=make_point(start_x,start_y,start_z)))
        waypoints.append(Waypoint(position=make_point(start_x,start_y,start_z+self.chesspiece_clearance)))
        waypoints.append(Waypoint(position=make_point(end_x,end_y,end_z+self.chesspiece_clearance)))
        waypoints.append(Waypoint(position=make_point(end_x,end_y,end_z+0.005), gripper="open"))
        trajectories = self.plan_move_trajectory(waypoints)
        return trajectories

    def plan_move_home(self,end_x,end_y,end_z):
        waypoints = []
        waypoints.append(Waypoint(position=make_point(end_x,end_y,end_z+self.chesspiece_clearance)))
        #waypoints.append(Waypoint(named_position="chess_home"))
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
                msg, last_joint_positions = plan(pc, last_joint_positions)
                trajectories.append(msg)
            elif wp.position:
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
        
        jc = JointConstraint()
        jc.joint_name = "wrist_1_joint"
        jc.position = math.radians(-110)
        jc.tolerance_above = math.radians(100)
        jc.tolerance_below = math.radians(70)
        jc.weight = 1.0
        joint_constraints.joint_constraints.append(jc)

        jc = JointConstraint()
        jc.joint_name = "wrist_2_joint"
        jc.position = math.radians(90)
        jc.tolerance_above = math.radians(100)
        jc.tolerance_below = math.radians(100)
        jc.weight = 1.0
        joint_constraints.joint_constraints.append(jc)

        jc = JointConstraint()
        jc.joint_name = "wrist_3_joint"
        jc.position = math.radians(135)
        jc.tolerance_above = math.radians(90)
        jc.tolerance_below = math.radians(90)
        jc.weight = 1.0
        joint_constraints.joint_constraints.append(jc)

        jc = JointConstraint()
        jc.joint_name = "shoulder_pan_joint"
        jc.position = math.radians(90)
        jc.tolerance_above = math.radians(45)
        jc.tolerance_below = math.radians(45)
        jc.weight = 1.0
        joint_constraints.joint_constraints.append(jc)
        
        return joint_constraints
        
def main(args=None):
    rclpy.init(args=args)
    commander = MoveItCommander()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(commander)
    executor.spin()

    commander.destroy_node()
    rclpy.shutdown()
