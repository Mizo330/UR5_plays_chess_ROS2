import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder

class UR5Controller(Node):
    def __init__(self):
        super().__init__('urchess_control')
        moveit_config = (
            MoveItConfigsBuilder(robot_name="ur5e", package_name="ur_moveit_config")
            .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
            .robot_description_semantic(file_path="srdf/ur.srdf.xacro", mappings={"name": "ur5e"})
            .to_moveit_configs()
        )

        
        self.moveit_py = MoveItPy(node_name='urchess_control',config_dict=moveit_config.to_dict())
        self.get_logger().info('MoveItPy initialized')
        self.planning_component = self.moveit_py.get_planning_component('manipulator')

    def move_to_pose(self, x, y, z, qx, qy, qz, qw):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw

        self.planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link='ee_link')
        plan_result = self.planning_component.plan()
        if plan_result:
            self.moveit_py.execute(plan_result.trajectory)
        else:
            self.get_logger().error('Planning failed')

def main():
    rclpy.init()
    controller = UR5Controller()
    controller.move_to_pose(
        x=0.4, y=0.1, z=0.3,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0
    )
    rclpy.shutdown()

if __name__ == '__main__':
    main()
