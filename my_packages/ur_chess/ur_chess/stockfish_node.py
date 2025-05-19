from stockfish import Stockfish
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import yaml
import chess
from ur_chess_msgs.srv import URChessMovePiece
from ur_chess_msgs.msg import URChessMoveInfo, URChessMoveStatus
from ur_chess.misc.text_color import TextColor


class GameManager(Node):
    def __init__(self):
        super().__init__('stockfish_player')
        self.fen = chess.STARTING_FEN
        self.stockfish = Stockfish(path="/home/appuser/ros2_ws/src/stockfish/stockfish-ubuntu-x86-64-avx2")
        self.stockfish.set_fen_position(self.fen)
        
        self.create_subscription(String,'/ur_chess/chessboard_state',self.new_board_callback,10)
        self.current_move_pub = self.create_publisher(String,'/ur_chess/current_move',10)

    
    def new_board_callback(self,msg:String):
        self.fen = msg.data
        self.stockfish.set_fen_position(self.fen)
        move = self.stockfish.get_best_move()
        self.get_logger().info("Stockfish: selected move: "+ move)
        #? maybe stockfish.get_eval()
        self.current_move_pub.publish(String(data=move))
        
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