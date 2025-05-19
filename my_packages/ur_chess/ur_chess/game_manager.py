import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import yaml
import chess
from ur_chess_msgs.srv import URChessMovePiece, URChessMoveNamedPos
from ur_chess_msgs.msg import URChessMoveInfo, URChessMoveStatus
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
        self.controller_client = self.create_client(URChessMovePiece, '/ur_chess/move_piece')
        self.controller_named_pos_client = self.create_client(URChessMoveNamedPos, '/ur_chess/move_named_pos')
        self.get_logger().info('Waiting for moveit controller services...')
        self.controller_client.wait_for_service()
        self.controller_named_pos_client.wait_for_service()
        # Publish updated FEN
        self.fen_pub = self.create_publisher(String, '/ur_chess/chessboard_state', 10)
        self.move_status_pub = self.create_publisher(URChessMoveStatus, '/ur_chess/move_status',10)
        self.stop_pub = self.create_publisher(String, '/trajectory_execution_event', 10)

        # Buffer for current request
        self.pending_move = None

        self.get_logger().info(TextColor.color_text('GameManager initialized', TextColor.OKCYAN))

    def control_callback(self, msg):
        command = msg.data.lower()
        if command == 'play':
            if self.state in ('paused', 'stopped'):
                self.get_logger().info(TextColor.color_text('Starting game', TextColor.OKGREEN))
                req = URChessMoveNamedPos.Request()
                req.named_posisiton = "chess_home"       
                future = self.controller_named_pos_client.call_async(req)
                future.add_done_callback(self.move_named_pos_callback)
        elif command == 'pause':
            if self.state == 'running':
                self.get_logger().info(TextColor.color_text('Pausing game', TextColor.WARNING))
                self.stop_pub.publish(String(data='stop'))
                self.state = 'stopped'
        elif command == 'stop':
            self.get_logger().info(TextColor.color_text('Stopping game', TextColor.FAIL))
            self.state = 'stopped'
            self.stop_pub.publish(String(data='stop'))
            self.board.reset()
            fen = self.board.fen()
            self.fen_pub.publish(String(data=fen))
        else:
            self.get_logger().warn(f'Unknown game control command: {command}')
    
    def move_named_pos_callback(self,future):
        if future.result() is None:
            self.get_logger().error('Service call failed')
            return
        result = future.result()
        if result.success:
            self.get_logger().info(TextColor.BOLD+TextColor.OKGREEN+'Game is ready. Good luck!'+TextColor.ENDC)
            self.state = 'running'
            fen = self.board.fen()
            self.fen_pub.publish(String(data=fen))
        else:
            self.get_logger().fatal("Failed to move to position! Closing manager.")
            raise KeyboardInterrupt
            
    def move_callback(self, msg: String):
        
        if self.state != 'running':
            self.get_logger().warn('Game is not running; ignoring move command')
            return
        
        move_str = msg.data  # e.g. "e2e4"
        if self.pending_move is not None:
            self.get_logger().warn(f'Still waiting for previous move result; ignoring {move_str}')
            return

        board = self.board.copy()
        try:
            move = chess.Move.from_uci(move_str)
        except ValueError:
            self.get_logger().error(f'Invalid UCI format: {move_str}')
            msg = URChessMoveStatus(status=3,message="Invalid UCI format")
            self.move_status_pub.publish(msg)
            return
        # Check if it's a legal move
        if move not in board.legal_moves:
            # Check if the move is pseudo-legal (syntactically valid, piece exists and can move like that)
            if move in board.pseudo_legal_moves:
                error_msg = f'Move {move_str} is pseudo-legal but not allowed: likely wrong side to move or king in check.'
                self.get_logger().error(error_msg)
            else:
                piece = board.piece_at(move.from_square)
                if piece is None:
                    error_msg = f'No piece at source square {move_str[:2]}'
                    self.get_logger().error(error_msg)
                else:
                    # Check color mismatch
                    if piece.color != board.turn:
                        side = "white" if board.turn else "black"
                        error_msg = f"It is {side}'s turn. The piece at {move_str[:2]} is {('white' if piece.color else 'black')}."
                        self.get_logger().error(error_msg)
                    else:
                        error_msg = f'Move {move_str} is not legal in this position.'
                        self.get_logger().error(error_msg)
            msg = URChessMoveStatus(status=3,message="Illegal move.")
            self.move_status_pub.publish(msg)
            return
        
        moveinfo = URChessMoveInfo()
        moveinfo.move_uci = move_str
        moveinfo.move_number = board.fullmove_number
        moveinfo.ply = board.ply()
        moveinfo.moved_piece = board.piece_at(move.from_square) #TODO handle this (for different piece size..)
        moveinfo.turn = board.turn
        moveinfo.is_promotion = move.promotion is not None
        
        # moved_piece should be the piece symbol as string (e.g. 'P', 'n', etc.)
        moved_piece = board.piece_at(move.from_square)
        moveinfo.moved_piece = moved_piece.symbol() if moved_piece else ''

        moveinfo.is_promotion = move.promotion is not None
        moveinfo.is_check = board.gives_check(move)
        moveinfo.is_checkmate = False

        # Determine special_move enum:
        if board.is_capture(move):
            moveinfo.special_move = URChessMoveInfo.SPECIAL_MOVE_CAPTURE
            if board.is_en_passant(move):
                moveinfo.special_move = URChessMoveInfo.SPECIAL_MOVE_EN_PASSANT
        elif board.is_kingside_castling(move):
            moveinfo.special_move = URChessMoveInfo.SPECIAL_MOVE_KINGSIDE_CASTLE
        elif board.is_queenside_castling(move):
            moveinfo.special_move = URChessMoveInfo.SPECIAL_MOVE_QUEENSIDE_CASTLE
        else:
            moveinfo.special_move = URChessMoveInfo.SPECIAL_MOVE_NONE
        
        req = URChessMovePiece.Request()
        req.moveinfo = moveinfo
        
        status_msg = URChessMoveStatus()
        status_msg.status = URChessMoveStatus.STATUS_IN_PROGRESS
        status_msg.moveinfo = moveinfo
        self.get_logger().info(f'Sending move request: {move_str}')
        future = self.controller_client.call_async(req)
        self.move_status_pub.publish(status_msg)
        future.add_done_callback(self.srv_response_callback)
        self.pending_move = moveinfo

    def srv_response_callback(self, future):
        if future.result() is None:
            self.get_logger().error('Service call failed')
            status_msg = URChessMoveStatus()
            status_msg.status = URChessMoveStatus.STATUS_FAILED
            status_msg.moveinfo = self.pending_move
            self.move_status_pub.publish(status_msg)
            return
        result = future.result()
        if result.success:
            self.board.push_uci(self.pending_move.move_uci)
            fen = self.board.fen()
            self.get_logger().info(f'Move {self.pending_move.move_uci} succeeded. Published FEN.')
            status_msg = URChessMoveStatus()
            status_msg.status = URChessMoveStatus.STATUS_SUCCESS
            status_msg.moveinfo = self.pending_move
            status_msg.moveinfo.is_checkmate = self.board.is_checkmate()
            self.move_status_pub.publish(status_msg)
            self.fen_pub.publish(String(data=fen))

            if self.board.is_checkmate(): self.handle_checkmate()
                
        else:
            self.get_logger().error(f'Move {self.pending_move.move_uci} failed: {result.message}')
            #raise RuntimeError(f'Move queue failed: {result.message}') #? maybe dont raise error?
            status_msg = URChessMoveStatus()
            status_msg.status = URChessMoveStatus.STATUS_FAILED
            status_msg.moveinfo = self.pending_move
            self.move_status_pub.publish(status_msg)
        self.pending_move = None
    
    def handle_checkmate(self):
        #TODO expand this
        self.get_logger().info(f"{TextColor.OKGREEN}{"Checkmate!"}{TextColor.ENDC}",)
        
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
