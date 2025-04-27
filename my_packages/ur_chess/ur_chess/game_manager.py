import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import yaml
import chess
from ur_chess_msgs.srv import URChessMovePiece

class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager')
        # Load board transform config
        config_path = self.declare_parameter('config_file', '/home/appuser/ros2_ws/src/my_packages/ur_chess/config/board_layout.yaml').get_parameter_value().string_value
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        # config: { 'a1': [x, y, z], 'tile_size': float }
        self.corner_a1 = cfg['a1']
        self.tile_size = cfg['tile_size']

        # Internal chess board
        self.board = chess.Board()


        self.state = 'stopped' # 'running', 'stopped'
        # Publishers & Subscribers
        self.move_sub = self.create_subscription(
            String,
            '/current_move',
            self.move_callback,
            10)
        self.control_sub = self.create_subscription(
            String,
            '/game_control',
            self.control_callback,
            10
        )
        # Send target coords as string "x1,y1,x2,y2"
        self.cmd_pub = self.create_publisher(String, '/robot_command', 10)
        # Receive execution result
        self.controller_client = self.create_client(URChessMovePiece, 'ur_chess/move_piece')
        self.controller_client.wait_for_service()
        # Publish updated FEN
        self.fen_pub = self.create_publisher(String, '/chessboard_state', 10)

        # Buffer for current request
        self.pending_move = None

        self.get_logger().info('GameManager node started.')

    def control_callback(self, msg):
        command = msg.data.lower()
        if command == 'play':
            if self.state in ('paused', 'stopped'):
                self.get_logger().info('Playing game')
                self.state = 'running'
        elif command == 'pause':
            if self.state == 'running':
                self.get_logger().info('Pausing game')
                self.state = 'stopped'
        elif command == 'stop':
            self.get_logger().info('Stopping and resetting game')
            self.state = 'stopped'
            self.board.reset()
            fen = self.board.fen()
            self.fen_pub.publish(String(data=fen))
        else:
            self.get_logger().warn(f'Unknown game control command: {command}')
    
    def move_callback(self, msg: String):
        if self.state != 'running':
            self.get_logger().warn('Game is not running; ignoring move command')
            return
        move_str = msg.data  # e.g. "e2e4"
        try:
            board = self.board.copy()
            board.parse_uci(move_str)
        except chess.InvalidMoveError:
            self.get_logger().error(f'Invalid UCI format: {move_str}')
            return
        except chess.IllegalMoveError:
            self.get_logger().error(f'Illegal in this position: {move_str}')
            return

        if self.pending_move is not None:
            self.get_logger().warn('Still waiting for previous move result; ignoring {}'.format(move_str))
            return
        try:
            # compute coordinates
            start_sq = chess.SQUARE_NAMES.index(move_str[:2])
            end_sq = chess.SQUARE_NAMES.index(move_str[2:4])
            x1, y1 = self.square_to_xy(move_str[:2])
            x2, y2 = self.square_to_xy(move_str[2:4])
            # send command to robot
            req = URChessMovePiece.Request()
            req.start_x = x1
            req.start_y = y1
            req.start_z = self.corner_a1[2]
            req.end_x = x2
            req.end_y = y2
            req.end_z = self.corner_a1[2]
            self.get_logger().info(f'Calling move_piece with {x1}, {y1}, {x2}, {y2}')
            handle = self.controller_client.call_async(req)
            handle.add_done_callback(self.result_callback)
            self.pending_move = move_str
        except Exception as e:
            self.get_logger().error(f'Invalid move format: {move_str} ({e})')

    def result_callback(self, future):
        msg = future.result()  # wait for result
        if self.pending_move is None:
            return  # spurious
        if msg.success:
            # successful
            self.board.push_uci(self.pending_move)
            fen = self.board.fen()
            self.fen_pub.publish(String(data=fen))
            self.get_logger().info(f'Move {self.pending_move} succeeded. Published FEN.')
        else:
            self.get_logger().error(f'Move {self.pending_move} failed.')
        # clear pending
        self.pending_move = None

    def square_to_xy(self, square: str):
        # square: 'a1'..'h8'
        file = ord(square[0]) - ord('a')
        rank = int(square[1]) - 1
        # center offset: add half tile
        x = self.corner_a1[0] + file * self.tile_size + self.tile_size/2
        y = self.corner_a1[1] + rank * self.tile_size + self.tile_size/2
        # z could be constant above board
        z = self.corner_a1[2]
        return round(x, 3), round(y, 3)


def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
