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
    QGroupBox, QFormLayout, QLabel, QPushButton, QToolButton, QStyle
)
from PyQt5.QtGui import QPainter, QColor, QFont

# Unicode chess symbols mapping
default_piece_unicode = {
    'r': '♜', 'n': '♞', 'b': '♝', 'q': '♛', 'k': '♚', 'p': '♟',
    'R': '♖', 'N': '♘', 'B': '♗', 'Q': '♕', 'K': '♔', 'P': '♙'
}

class ROSWorker(QThread):
    jointStateReceived = pyqtSignal(object)
    poseReceived = pyqtSignal(object)
    gripperStateReceived = pyqtSignal(bool)
    modeReceived = pyqtSignal(str)
    fenUpdated = pyqtSignal(str)
    currentMoveReceived = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.pub_estop = None
        self.pub_game_control = None
        self._running = True

    def run(self):
        rclpy.init()
        self.node = Node('ros_gui_worker')
        self.pub_estop = self.node.create_publisher(Bool, '/ur_chess/e_stop', 10)
        self.pub_game_control = self.node.create_publisher(String, '/ur_chess/game_control', 10)

        self.node.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.node.create_subscription(PoseStamped, '/tcp_pose', self._pose_cb, 10)
        self.node.create_subscription(Bool, '/gripper/state', self._gripper_cb, 10)
        self.node.create_subscription(String, '/robot/mode', self._mode_cb, 10)
        self.node.create_subscription(String, '/ur_chess/chessboard_state', self._fen_cb, 10)
        self.node.create_subscription(String, '/ur_chess/current_move', self._move_cb, 10)

        executor = rclpy.get_global_executor()
        executor.add_node(self.node)
        while rclpy.ok() and self._running:
            executor.spin_once(timeout_sec=0.1)
        self.node.destroy_node()
        rclpy.shutdown()

    def shutdown(self):
        self._running = False

    def _joint_cb(self, msg): self.jointStateReceived.emit(msg)
    def _pose_cb(self, msg): self.poseReceived.emit(msg)
    def _gripper_cb(self, msg): self.gripperStateReceived.emit(msg.data)
    def _mode_cb(self, msg): self.modeReceived.emit(msg.data)
    def _fen_cb(self, msg): self.fenUpdated.emit(msg.data.split(' ')[0])
    def _move_cb(self, msg): self.currentMoveReceived.emit(msg.data)

    def send_estop(self, engaged: bool):
        if self.pub_estop:
            self.pub_estop.publish(Bool(data=engaged))

    def send_game_control(self, command: str):
        if self.pub_game_control:
            self.pub_game_control.publish(String(data=command))

class ChessboardWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.fen = 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR'
        self.current_move = ''
        self.square_size = 60
        self.setMinimumSize(self.square_size * 8, self.square_size * 8)

    def update_board(self, fen: str):
        self.fen = fen; self.update()

    def highlight_move(self, move: str):
        self.current_move = move; self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        board = self._fen_to_matrix(self.fen)
        for rank in range(8):
            for file in range(8):
                color = QColor(240,217,181) if (rank+file)%2==0 else QColor(181,136,99)
                painter.fillRect(file*self.square_size, rank*self.square_size,
                                  self.square_size, self.square_size, color)
                piece = board[rank][file]
                if piece:
                    symbol = default_piece_unicode.get(piece, '')
                    painter.setFont(QFont('Arial', 32))
                    painter.drawText(file*self.square_size, rank*self.square_size,
                                     self.square_size, self.square_size,
                                     Qt.AlignCenter, symbol)
        if len(self.current_move)==4:
            sf, sr = ord(self.current_move[0])-97, 8-int(self.current_move[1])
            ef, er = ord(self.current_move[2])-97, 8-int(self.current_move[3])
            pen = painter.pen(); pen.setWidth(3); pen.setColor(QColor(255,0,0)); painter.setPen(pen)
            painter.drawRect(sf*self.square_size, sr*self.square_size,
                             self.square_size, self.square_size)
            painter.drawRect(ef*self.square_size, er*self.square_size,
                             self.square_size, self.square_size)

    def _fen_to_matrix(self, fen: str):
        rows = fen.split('/')
        board=[]
        for row in rows:
            line=[]
            for c in row:
                line.extend(['']*int(c) if c.isdigit() else [c])
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

        # Status Panel
        status = QGroupBox('Status')
        form = QFormLayout()
        self.mode_label = QLabel('Unknown')
        self.joint_label = QLabel('---')
        self.gripper_label = QLabel('Unknown')
        form.addRow('Mode:', self.mode_label)
        form.addRow('Joints:', self.joint_label)
        form.addRow('Gripper:', self.gripper_label)
        status.setLayout(form)
        layout.addWidget(status)

        # E-Stop
        self.estop_button = QPushButton('E-STOP (WIP)')
        self.estop_button.setStyleSheet('background:red;color:white;font-size:18pt;')
        layout.addWidget(self.estop_button)
        #! disable estop button for now
        self.estop_button.setEnabled(False)
        self.estop_button.setStyleSheet('background:gray;color:white;font-size:18pt;')


        # Chessboard
        self.chessboard = ChessboardWidget()
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
        ctrl.addLayout(pause_layout)

        self.stop_button = QToolButton()
        self.stop_button.setIcon(style.standardIcon(QStyle.SP_MediaStop))
        self.stop_button.setIconSize(QSize(32,32))
        self.stop_button.setToolButtonStyle(Qt.ToolButtonIconOnly)
        self.stop_button.setFixedSize(50,50)
        stop_lbl = QLabel('Stop'); stop_lbl.setAlignment(Qt.AlignCenter)
        stop_layout = QVBoxLayout(); stop_layout.addWidget(self.stop_button); stop_layout.addWidget(stop_lbl)
        ctrl.addLayout(stop_layout)

        layout.addLayout(ctrl)
        central.setLayout(layout)
        self.setCentralWidget(central)

        # Initial button states
        self.play_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)

    def _connect_signals(self):
        w = self.ros_worker
        w.modeReceived.connect(lambda m: self.mode_label.setText(m))
        w.jointStateReceived.connect(lambda js: self.joint_label.setText(', '.join(f'{p:.2f}' for p in js.position)))
        w.gripperStateReceived.connect(lambda g: self.gripper_label.setText('Open' if g else 'Closed'))
        w.fenUpdated.connect(self.chessboard.update_board)
        w.currentMoveReceived.connect(self.chessboard.highlight_move)

        self.estop_button.clicked.connect(lambda: w.send_estop(True))
        self.play_button.clicked.connect(self.on_play)
        self.pause_button.clicked.connect(self.on_pause)
        self.stop_button.clicked.connect(self.on_stop)

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
