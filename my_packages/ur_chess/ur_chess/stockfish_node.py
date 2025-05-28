from stockfish import Stockfish
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ur_chess_msgs.srv import URChessMovePiece
from ur_chess.misc.text_color import TextColor
import chess


class GameManager(Node):
    def __init__(self):
        super().__init__('stockfish_player')
        
        self.declare_parameter("mode", "FishVFish")
        self.declare_parameter("stockfish1_skill_level", 20)
        self.declare_parameter("stockfish1_contempt", 50)
        self.declare_parameter("stockfish2_skill_level", 15)
        self.declare_parameter("stockfish2_contempt", 40)
        
        self.mode = self.get_parameter("mode").value
        self.skill_levels = {
            'w': self.get_parameter("stockfish1_skill_level").value,
            'b': self.get_parameter("stockfish2_skill_level").value,
        }
        self.contempts = {
            'w': self.get_parameter("stockfish1_contempt").value,
            'b': self.get_parameter("stockfish2_contempt").value,
        }

        if self.mode == "PVP":
            self.get_logger().warn("We play in PVP mode! There's no need for me!")
            self.get_logger().info(f"{TextColor.FAIL}{TextColor.UNDERLINE}Goodbye!{TextColor.ENDC}")
            raise SystemExit("Exiting cleanly.")

        self.singleplayer_color = {'P(w)VFish': 'w', 'P(b)VFish': 'b'}.get(self.mode)

        self.fen = chess.STARTING_FEN
        self.stockfish_engines = self._init_stockfish_engines()

        self.create_subscription(String, '/ur_chess/chessboard_state', self.new_board_callback, 10)
        self.current_move_pub = self.create_publisher(String, '/ur_chess/current_move', 10)

        self._wait_for_move_service()

        self.get_logger().info(f"{TextColor.OKBLUE}{TextColor.UNDERLINE}Fish is ready to play!{TextColor.ENDC}")


    def _init_stockfish_engines(self):
        path = "/home/appuser/ros2_ws/src/stockfish/stockfish-ubuntu-x86-64-avx2"
        engines = {}
        for color in ['w', 'b']:
            engine = Stockfish(path=path)
            engine.update_engine_parameters({
                "Skill Level": self.skill_levels[color],
                "Contempt": self.contempts[color]
            })
            engine.set_fen_position(self.fen)
            engines[color] = engine
        return engines

    def _wait_for_move_service(self):
        client = self.create_client(URChessMovePiece, "/ur_chess/move_piece")
        client.wait_for_service()
        self.destroy_client(client)

    def new_board_callback(self, msg: String):
        self.fen = msg.data
        turn = self.fen.split()[1]

        # Skip move if it's not the AI's turn in singleplayer
        if self.singleplayer_color and self.singleplayer_color == turn:
            self.get_logger().info("Not my turn, skipping.")
            return

        engine = self.stockfish_engines[turn]
        engine.set_fen_position(self.fen)
        move = engine.get_best_move()

        color_name = "Stockfish" if self.singleplayer_color else f"Stockfish({turn})"
        self.get_logger().info(f"{color_name}: selected move: {move}")

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