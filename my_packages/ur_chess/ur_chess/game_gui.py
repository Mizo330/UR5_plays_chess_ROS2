import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from PyQt5.QtCore import QObject, QThread, pyqtSignal, Qt, QSize
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QFormLayout, QLabel, QPushButton, QToolButton, QStyle, QMessageBox
)
from PyQt5.QtGui import QPainter, QColor, QFont
import chess
from ur_chess_msgs.msg import URChessMoveInfo, URChessMoveStatus

class ROSWorker(QThread):
    jointStateReceived = pyqtSignal(object)
    poseReceived = pyqtSignal(object)
    gripperStateReceived = pyqtSignal(str)
    modeReceived = pyqtSignal(str)
    fenUpdated = pyqtSignal(str)
    currentMoveReceived = pyqtSignal(URChessMoveStatus)
    
    

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = Node('ros_gui_worker')
        self.node.declare_parameter("mode", "FishVFish")
        self.node.declare_parameter("remote", False)
        self.pub_estop = None
        self.pub_game_control = None
        self._running = True

    def run(self):

        self.pub_estop = self.node.create_publisher(String, '/trajectory_execution_event', 10)
        self.pub_game_control = self.node.create_publisher(String, '/ur_chess/game_control', 10)
        self.current_move_pub = self.node.create_publisher(String, '/ur_chess/current_move', 10)

        self.node.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.node.create_subscription(String, '/robot/mode', self._mode_cb, 10)
        self.node.create_subscription(String, '/ur_chess/chessboard_state', self._fen_cb, 10)
        self.node.create_subscription(URChessMoveStatus, '/ur_chess/move_status',self._move_status_cb,10)

        executor = rclpy.get_global_executor()
        executor.add_node(self.node)
        while rclpy.ok() and self._running:
            executor.spin_once(timeout_sec=0.1)
        self.node.destroy_node()
        rclpy.shutdown()

    def shutdown(self):
        self._running = False

    def _joint_cb(self, msg):
        # Gripper open/close based on joint[3]
        try:
            gripper_pos = msg.position[3]
            is_closed = gripper_pos > 0.9
            self.gripperStateReceived.emit("closed" if is_closed else "open")
        except IndexError:
            print("Joint[3] not available in joint state message")
        # Filter out joints at index 1, 2, 3
        filtered_positions = [p for i, p in enumerate(msg.position) if i not in [1, 2, 3, 4]]
        
        # Emit filtered joint states
        filtered_msg = msg
        filtered_msg.position = filtered_positions
        self.jointStateReceived.emit(filtered_msg)

    def _move_status_cb(self,msg:URChessMoveStatus): self.currentMoveReceived.emit(msg)
    def _mode_cb(self, msg): self.modeReceived.emit(msg.data)
    def _fen_cb(self, msg): self.fenUpdated.emit(msg.data.split(' ')[0])

    def send_estop(self, event):
        if self.pub_estop:
            self.pub_estop.publish(String(data=event))

    def send_game_control(self, command: str):
        if self.pub_game_control:
            self.pub_game_control.publish(String(data=command))
            
    def publish_current_move(self, move: str):
        if self.current_move_pub:
            self.current_move_pub.publish(String(data=move))


class ChessboardWidget(QWidget):
    moveSelected = pyqtSignal(str)  # emits UCI move string like "e2e4"

    def __init__(self, parent=None):
        super().__init__(parent)
        self.fen = chess.STARTING_BOARD_FEN
        self.current_move = ''
        self.square_size = 60
        self.setMinimumSize(self.square_size * 8, self.square_size * 8)
        self.selected_squares = []  # to hold square names like "e2", "e4"
        self.block_move = False
        self.mode = None
        self.singleplayercolor = None
        self.turn = None
        
    def update_board(self, fen: str):
        self.fen = fen
        self.current_move = ''
        self.selected_squares.clear()
        self.update()

    def highlight_move(self, move: str):
        self.current_move = move
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        board = self._fen_to_matrix(self.fen)
        for rank in range(8):
            for file in range(8):
                color = QColor(240,217,181) if (rank + file) % 2 == 0 else QColor(181,136,99)
                x = file * self.square_size
                y = rank * self.square_size
                painter.fillRect(x, y, self.square_size, self.square_size, color)

                square_name = self._index_to_square(file, rank)
                if square_name in self.selected_squares:
                    painter.setBrush(QColor(0, 255, 0, 100))
                    painter.drawRect(x, y, self.square_size, self.square_size)

                piece = board[rank][file]
                if piece:
                    symbol = chess.UNICODE_PIECE_SYMBOLS.get(piece, '')
                    painter.setFont(QFont('Arial', 32))
                    painter.drawText(x, y, self.square_size, self.square_size, Qt.AlignCenter, symbol)

        if len(self.current_move) == 4:
            sf, sr = ord(self.current_move[0]) - 97, 8 - int(self.current_move[1])
            ef, er = ord(self.current_move[2]) - 97, 8 - int(self.current_move[3])
            pen = painter.pen()
            pen.setWidth(3)
            pen.setColor(QColor(255, 0, 0))
            painter.setPen(pen)
            painter.drawRect(sf * self.square_size, sr * self.square_size, self.square_size, self.square_size)
            painter.drawRect(ef * self.square_size, er * self.square_size, self.square_size, self.square_size)

    def mousePressEvent(self, event):
        if self.block_move or self.mode == "FishVFish": 
            return
        if self.singleplayercolor is not None:
            if self.singleplayercolor != self.turn:
                return
        file = event.x() // self.square_size
        rank = event.y() // self.square_size
        square = self._index_to_square(file, rank)
        self.selected_squares.append(square)

        if len(self.selected_squares) == 2:
            move = self.selected_squares[0] + self.selected_squares[1]
            self.current_move = move
            self.moveSelected.emit(move)  # emit signal
            self.selected_squares.clear()

        self.update()

    def _index_to_square(self, file: int, rank: int) -> str:
        """Convert (file, rank) indices to algebraic notation."""
        return f"{chr(97 + file)}{8 - rank}"

    def _fen_to_matrix(self, fen: str):
        """Convert FEN board part into 2D array."""
        fen = fen.split()[0]  # ignore move/turn info
        rows = fen.split('/')
        board = []
        for row in rows:
            line = []
            for c in row:
                line.extend([''] * int(c) if c.isdigit() else [c])
            board.append(line)
        return board
    
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('UR5 Chess GUI')
        self.ros_worker = ROSWorker()
        self._init_ui()
        self._connect_signals()
        self.ros_worker.start()

    def _init_ui(self):
        central = QWidget()
        layout = QVBoxLayout()

        # Chessboard
        self.chessboard = ChessboardWidget()

        #Get mode 
        mode = self.ros_worker.node.get_parameter("mode").value # choices=['FishVFish',"PVP","P(b)VFish","P(w)VFish"])
        self.chessboard.mode = mode
        # Determine player color for single-player modes
        if mode == "P(w)VFish":
            self.chessboard.singleplayercolor = True  # white
        elif mode == "P(b)VFish":
            self.chessboard.singleplayercolor = False  # black
        else:
            self.chessboard.singleplayercolor = None  # not singleplayer
                
        # Status Panel
        status = QGroupBox('Status')
        form = QFormLayout()
        self.mode_label = QLabel(mode)
        self.joint_label = QLabel('---')
        self.gripper_label = QLabel('Unknown')
        self.movestatus_label = QLabel("Unkown")
        self.current_side_label = QLabel("White")
        form.addRow('Mode:', self.mode_label)
        form.addRow('Joints:', self.joint_label)
        form.addRow('Gripper:', self.gripper_label)
        form.addRow("Last move status:", self.movestatus_label)
        form.addRow("Current side", self.current_side_label)
        status.setLayout(form)
        layout.addWidget(status)

        # E-Stop
        self.estop_button = QPushButton('E-STOP')
        self.estop_button.setStyleSheet('background:red;color:white;font-size:18pt;')
        layout.addWidget(self.estop_button)

        #Add the chessboard widget
        layout.addWidget(self.chessboard)

        # Controls
        ctrl = QHBoxLayout()
        style = self.style()
        self.play_button = QToolButton()
        self.play_button.setIcon(style.standardIcon(QStyle.SP_MediaPlay))
        self.play_button.setIconSize(QSize(32,32))
        self.play_button.setToolButtonStyle(Qt.ToolButtonIconOnly)
        self.play_button.setFixedSize(50,50)
        play_lbl = QLabel('Start'); play_lbl.setAlignment(Qt.AlignCenter)
        play_layout = QVBoxLayout(); play_layout.addWidget(self.play_button); play_layout.addWidget(play_lbl)
        ctrl.addLayout(play_layout)

        self.pause_button = QToolButton()
        self.pause_button.setIcon(style.standardIcon(QStyle.SP_MediaPause))
        self.pause_button.setIconSize(QSize(32,32))
        self.pause_button.setToolButtonStyle(Qt.ToolButtonIconOnly)
        self.pause_button.setFixedSize(50,50)
        pause_lbl = QLabel('Pause'); pause_lbl.setAlignment(Qt.AlignCenter)
        pause_layout = QVBoxLayout(); pause_layout.addWidget(self.pause_button); pause_layout.addWidget(pause_lbl)
        #ctrl.addLayout(pause_layout)

        self.stop_button = QToolButton()
        self.stop_button.setIcon(style.standardIcon(QStyle.SP_MediaStop))
        self.stop_button.setIconSize(QSize(32,32))
        self.stop_button.setToolButtonStyle(Qt.ToolButtonIconOnly)
        self.stop_button.setFixedSize(50,50)
        stop_lbl = QLabel('Stop'); stop_lbl.setAlignment(Qt.AlignCenter)
        stop_layout = QVBoxLayout(); stop_layout.addWidget(self.stop_button); stop_layout.addWidget(stop_lbl)
        ctrl.addLayout(stop_layout)
        remote = self.ros_worker.node.get_parameter("remote").value
        if not remote:
            layout.addLayout(ctrl)
        central.setLayout(layout)
        self.setCentralWidget(central)

        # Initial button states
        self.play_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)

    def handle_current_move(self, msg: URChessMoveStatus):
        self.chessboard.block_move = False
        if msg.moveinfo:
            self.chessboard.highlight_move(msg.moveinfo.move_uci)
            self.current_side_label.setText('White' if msg.moveinfo.turn else "Black")
        else:
            self.chessboard.highlight_move("")

        if msg.status == URChessMoveStatus.STATUS_FAILED:
            self.movestatus_label.setText("FAILED")
            reply = QMessageBox.question(
                self,
                "Robot move error",
                "Robot failed the move! Do you want to retry?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.chessboard.moveSelected.emit(msg.moveinfo.move_uci) #re-send move
        
        elif msg.status == URChessMoveStatus.STATUS_SUCCESS:
            self.movestatus_label.setText("SUCCESS")
            self.chessboard.highlight_move("")

            moveinfo = msg.moveinfo
            if moveinfo and self.chessboard.mode != "FishVFish":
                self.chessboard.turn = not moveinfo.turn
                is_player_turn = (
                    self.chessboard.singleplayercolor is not None and
                    self.chessboard.singleplayercolor != moveinfo.turn
                )

                # Notify player if it's their turn
                if is_player_turn:
                    QMessageBox.information(
                        self,
                        "Notification",
                        "Your Turn!"
                    )

                # Checkmate or check notifications
                if moveinfo.is_checkmate or moveinfo.is_check:
                    title = "Checkmate" if moveinfo.is_checkmate else "Check"
                    message = f"{'White' if moveinfo.turn else 'Black'} has delivered a {'checkmate! Good game!' if moveinfo.is_checkmate else 'check!'}"

                    # Only notify if it's the player's turn or no player color is set
                    if is_player_turn or self.chessboard.singleplayercolor is None:
                        QMessageBox.information(
                            self,
                            title,
                            message
                        )
                        
            self.chessboard.block_move = False
        elif msg.status == URChessMoveStatus.STATUS_IN_PROGRESS:
            self.movestatus_label.setText("IN PROGRESS")
            self.chessboard.block_move = True
        elif msg.status == URChessMoveStatus.STATUS_INVALID_MOVE:
            self.movestatus_label.setText("INVALID: " + msg.message)
            self.chessboard.selected_squares.clear()
            
        
    def _connect_signals(self):
        w = self.ros_worker
        w.modeReceived.connect(lambda m: self.mode_label.setText(m))
        w.jointStateReceived.connect(lambda js: self.joint_label.setText(', '.join(f'{p:.2f}' for p in js.position)))
        w.gripperStateReceived.connect(lambda g: self.gripper_label.setText('Open' if g == 'open' else 'Closed'))

        w.fenUpdated.connect(self.chessboard.update_board)
        w.currentMoveReceived.connect(self.handle_current_move)

        self.estop_button.clicked.connect(lambda: w.send_estop("stop"))
        self.play_button.clicked.connect(self.on_play)
        self.pause_button.clicked.connect(self.on_pause)
        self.stop_button.clicked.connect(self.on_stop)
        
        self.chessboard.moveSelected.connect(w.publish_current_move)

    def on_play(self):
        self.ros_worker.send_game_control('play')
        self.play_button.setEnabled(False)
        self.pause_button.setEnabled(True)
        self.stop_button.setEnabled(True)

    def on_pause(self):
        self.ros_worker.send_game_control('pause')
        self.play_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def on_stop(self):
        self.ros_worker.send_game_control('stop')
        self.play_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)

    def closeEvent(self, event):
        self.ros_worker.shutdown()
        self.ros_worker.wait()
        super().closeEvent(event)

def main():
    app=QApplication(sys.argv)
    window=MainWindow(); window.show(); sys.exit(app.exec_())
