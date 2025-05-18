from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import ObjectColor
from ur_chess.misc.math_helpers import make_point
from moveit.planning import PlanningSceneMonitor

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
            
def create_chess_environment(planning_scene_monitor:PlanningSceneMonitor,a1_0,orientation,tile_size):
    """Create table, chessboard and such for the chess scene.

    Args:
        planning_scene_monitor: The planning scene monitor to update into
        a1_0 (_[x,y,z]_): Coordinate for the left corner of the chessboard (corner of A1)
        orientation (_[x,y]_): 
        tile_size (_float_): size of the chessboard squares (side lenght) 
    """
    with planning_scene_monitor.read_write() as scene:    
        board_base = CollisionObject()
        board_base.id = "chessboard_w_base"
        board_base.header.frame_id = "base_link"
        board_base.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[tile_size*10,tile_size*10,0.01]))
        board_base.primitive_poses.append(Pose(
        position=make_point(
            a1_0[0] + orientation[0]*4*tile_size, 
            a1_0[1] + orientation[1]*4*tile_size, 
            a1_0[2]-0.01)))
        board_base.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[tile_size*20,tile_size*12,0.09]))
        board_base.primitive_poses.append(Pose(
        position=make_point(
            a1_0[0] + orientation[0]*4*tile_size, 
            a1_0[1] + orientation[1]*4*tile_size, 
            a1_0[2]-0.055)))
        board_base.operation = CollisionObject.ADD
        chessboard_color = ObjectColor(id=board_base.id, color=ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0))
        scene.apply_collision_object(board_base, chessboard_color)

        table = CollisionObject()
        table.id = "table"
        table.header.frame_id = "base_link"
        table.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[tile_size*30,tile_size*14,0.03]))
        table.primitive_poses.append(Pose(
            position=make_point(
                a1_0[0] + orientation[0]*4*tile_size, 
                a1_0[1] + orientation[1]*4*tile_size, 
                a1_0[2]-0.115)))
        table_color = ObjectColor(id=board_base.id, color=ColorRGBA(r=0.6, g=0.4, b=0.0, a=1.0))
        scene.apply_collision_object(table, table_color)
        scene.current_state.update()  # Important to ensure the scene is updated