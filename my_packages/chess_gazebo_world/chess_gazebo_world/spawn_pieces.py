#Only if spawning is needed
# TODO: Finish the code

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import time

class ChessSpawner(Node):
    def __init__(self):
        super().__init__('chess_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn_entity service not available, waiting...')
        self.spawn_pieces()

    def spawn_piece(self, name, model_path, x, y, z=0.01):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = open(model_path, 'r').read()
        req.robot_namespace = name
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.reference_frame = 'world'
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawned {name}")
        else:
            self.get_logger().error(f"Failed to spawn {name}")

    def spawn_pieces(self):
        # Koordináták példa szerint, finomhangolható
        start_x = -0.105  # bal alsó mező (a1) x
        start_y = -0.105  # bal alsó mező (a1) y
        step = 0.03  # mezők távolsága

        # Bábu pozíciók és típusok
        white_back_row = ['w_rook1', 'w_knight1', 'w_bishop1', 'w_queen', 'w_king', 'w_bishop2', 'w_knight2', 'w_rook2']
        black_back_row = ['b_rook1', 'b_knight1', 'b_bishop1', 'b_queen', 'b_king', 'b_bishop2', 'b_knight2', 'b_rook2']

        # Fehér gyalogok
        for i in range(8):
            x = start_x + i * step
            self.spawn_piece(f"w_pawn{i+1}", '/home/appuser/ros2_ws/src/my_packages/chess_models/w_pawn/model.sdf', x, start_y + step)

        # Fekete gyalogok
        for i in range(8):
            x = start_x + i * step
            self.spawn_piece(f"b_pawn{i+1}", '/home/appuser/ros2_ws/src/my_packages/chess_models/b_pawn/model.sdf', x, start_y + 6 * step)

        # Fehér hátulsó sor
        for i, name in enumerate(white_back_row):
            x = start_x + i * step
            self.spawn_piece(name, f"/home/appuser/ros2_ws/src/my_packages/chess_models/{name}/model.sdf", x, start_y)

        # Fekete hátulsó sor
        for i, name in enumerate(black_back_row):
            x = start_x + i * step
            self.spawn_piece(name, f"/home/appuser/ros2_ws/src/my_packages/chess_models/{name}/model.sdf", x, start_y + 7 * step)

        self.get_logger().info("All pieces spawned.")

def main(args=None):
    rclpy.init(args=args)
    spawner = ChessSpawner()
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
