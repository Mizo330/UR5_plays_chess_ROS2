import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import yaml
import chess
from ur_chess_msgs.srv import URChessMovePiece
from ur_chess_msgs.msg import URChessMoveResult
from ur_chess.misc.text_color import TextColor

class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager')

        # Internal chess board
        self.board = chess.Board()


        self.state = 'stopped' # 'running', 'stopped'
        # Publishers & Subscribers
        self.move_sub = self.create_subscription(
            String,
            '/ur_chess/current_move',
            self.move_callback,
            10)
        # Receive game control commands
        self.control_sub = self.create_subscription(
            String,
            '/ur_chess/game_control',
            self.control_callback,
            10
        )
        # # Receive move result
        # self.result_sub = self.create_subscription(
        #     URChessMoveResult,
        #     '/ur_chess/move_result',
        #     self.result_callback,
        #     10
        # )
        # Receive execution result
        self.get_logger().info('Waiting for move_piece service...')
        self.controller_client = self.create_client(URChessMovePiece, '/ur_chess/move_piece')
        self.controller_client.wait_for_service()
        # Publish updated FEN
        self.fen_pub = self.create_publisher(String, '/ur_chess/chessboard_state', 10)

        # Buffer for current request
        self.pending_move = None

        self.get_logger().info(TextColor.color_text('GameManager initialized', TextColor.OKCYAN))

    def control_callback(self, msg):
        command = msg.data.lower()
        if command == 'play':
            if self.state in ('paused', 'stopped'):
                self.get_logger().info(TextColor.color_text('Starting game', TextColor.OKGREEN))
                self.state = 'running'
        elif command == 'pause':
            if self.state == 'running':
                self.get_logger().info(TextColor.color_text('Pausing game', TextColor.WARNING))
                self.state = 'stopped'
        elif command == 'stop':
            self.get_logger().info(TextColor.color_text('Stopping game', TextColor.FAIL))
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
        if self.pending_move is not None:
            self.get_logger().warn('Still waiting for previous move result; ignoring {}'.format(move_str))
            return
        try:
            board = self.board.copy()
            move = chess.Move.from_uci(move_str)
            if move not in board.legal_moves:
                self.get_logger().error(f'Illegal in this position: {move_str}')
                return
        except ValueError:
            self.get_logger().error(f'Invalid UCI format: {move_str}')
            return
        req = URChessMovePiece.Request()
        req.uci = move_str
        req.is_capture = board.is_capture(move)
        req.capturer_color = board.turn
        self.get_logger().info(f'Sending move request: {move_str}')
        result = self.controller_client.call_async(req)
        result.add_done_callback(self.srv_response_callback)
        self.pending_move = move_str

    def srv_response_callback(self, future):
        if future.result() is None:
            self.get_logger().error('Service call failed')
            return
        result = future.result()
        if result.success:
            self.board.push_uci(self.pending_move)
            fen = self.board.fen()
            self.fen_pub.publish(String(data=fen))
            self.get_logger().info(f'Move {self.pending_move} succeeded. Published FEN.')
        else:
            self.get_logger().error(f'Move {self.pending_move} failed.')
            raise RuntimeError(f'Move queue failed: {result.message}') #? maybe dont raise error?
        self.pending_move = None


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
