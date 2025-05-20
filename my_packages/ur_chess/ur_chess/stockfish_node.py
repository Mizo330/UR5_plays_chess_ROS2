from stockfish import Stockfish
import rclpy
from rclpy.node import Node
import rclpy.task
from std_msgs.msg import String, Bool
import yaml
import chess
from ur_chess_msgs.srv import URChessMovePiece
from ur_chess_msgs.msg import URChessMoveInfo, URChessMoveStatus
from ur_chess.misc.text_color import TextColor


class GameManager(Node):
    def __init__(self):
        super().__init__('stockfish_player')
        self.declare_parameter("mode", "FishVFish")
        
        mode = self.get_parameter("mode").value
        
        if mode == "PVP":
            self.get_logger().warn("We play in PVP mode! Theres no need for me!")
            self.get_logger().info(TextColor.FAIL+TextColor.UNDERLINE+"Goodbye!"+TextColor.ENDC)
            raise SystemExit("Exiting cleanly.")
        # Determine player color for single-player modes
        if mode == "P(w)VFish":
            self.singleplayercolor = 'w'  # white
        elif mode == "P(b)VFish":
            self.singleplayercolor = 'b'  # black
        else:
            self.singleplayercolor = None  # not singleplayer
            
        self.fen = chess.STARTING_FEN
        self.stockfish = Stockfish(path="/home/appuser/ros2_ws/src/stockfish/stockfish-ubuntu-x86-64-avx2")
        self.stockfish.set_fen_position(self.fen)
        
        self.stockfish.get_best_move()
        self.stockfish.get_evaluation()
        
        self.create_subscription(String,'/ur_chess/chessboard_state',self.new_board_callback,10)
        self.current_move_pub = self.create_publisher(String,'/ur_chess/current_move',10)

        #*We only create this client so we wait for the moveit controller to start up.
        #Is this necessary? No, but nice to have all nodes init at the same time.
        waitclient = self.create_client(URChessMovePiece,"/ur_chess/move_piece")
        waitclient.wait_for_service()
        self.destroy_client(waitclient)
        
        self.get_logger().info(TextColor.OKBLUE+TextColor.UNDERLINE+"Fish is ready to play!"+TextColor.ENDC)
    
    def new_board_callback(self,msg:String):
        self.fen = msg.data
        turn = self.fen.split()[1]
        if self.singleplayercolor is not None:
            if self.singleplayercolor == turn:
                self.get_logger().info("Not my turn, skipping.")
                return
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