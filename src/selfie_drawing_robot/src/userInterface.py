#!/home/lucas/cornEnv/bin/python

from PySide6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QPushButton, QStackedWidget, QHBoxLayout, QSizePolicy, QSpacerItem, QTextEdit, QLineEdit
from PySide6.QtGui import QIcon, QPixmap, QPainter, QColor
from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer

import sys
import os
import subprocess
import select
import socket

class MessageReceiver(QThread):
    message_received = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.host = '127.0.0.1'
        self.port = 65433  # Same port as in the ROS node

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            while True:
                conn, addr = s.accept()
                with conn:
                    data = conn.recv(1024)
                    if data:
                        message = data.decode('utf-8')
                        self.message_received.emit(message)

class BackgroundWidget(QWidget):
    def __init__(self, image_path, background_color=None, parent=None):
        super().__init__(parent)
        self.pixmap = QPixmap(image_path)
        self.background_color = background_color

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.background_color:
            painter.fillRect(self.rect(), QColor(self.background_color))
        painter.drawPixmap(self.rect(), self.pixmap)

class BasePage(BackgroundWidget):
    def __init__(self, text, image_path, background_color=None, parent=None):
        super().__init__(image_path, background_color, parent)
        self.layout = QVBoxLayout(self)
        self.label = QLabel(text)
        self.label.setStyleSheet("color: white; font-size: 24px; font-weight: bold;")
        self.label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self.layout.addWidget(self.label)
        spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.layout.addItem(spacer)

class CalibrationPage(BasePage):
    def __init__(self, image_path, background_color=None, parent=None):
        super().__init__("Cornelius Calibration", image_path, background_color, parent)
        self.image_paths = [
            os.path.join(os.path.dirname(__file__), 'UI_files/canvas_top_left.png'),
            os.path.join(os.path.dirname(__file__), 'UI_files/canvas_top_right.png'),
            os.path.join(os.path.dirname(__file__), 'UI_files/canvas_bottom_left.png'),
            os.path.join(os.path.dirname(__file__), 'UI_files/canvas_bottom_right.png')
        ]
        self.current_image_index = 0
        self.recorded_values = []
        container = QWidget()
        container_layout = QVBoxLayout(container)
        self.image_stack = QStackedWidget()
        for image_path in self.image_paths:
            label = QLabel()
            pixmap = QPixmap(image_path).scaled(800, 600, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            label.setPixmap(pixmap)
            label.setAlignment(Qt.AlignCenter)
            self.image_stack.addWidget(label)
        container_layout.addWidget(self.image_stack)
        self.back_button = QPushButton("Back")
        self.next_button = QPushButton("Next")
        self.record_button = QPushButton("Record")
        self.back_button.setStyleSheet("color: white; background-color: #3244a8; padding: 10px;")
        self.next_button.setStyleSheet("color: white; background-color: #3244a8; padding: 10px;")
        self.record_button.setStyleSheet("color: white; background-color: #32a852; padding: 10px;")
        self.back_button.clicked.connect(self.go_back_image)
        self.next_button.clicked.connect(self.go_next_image)
        self.record_button.clicked.connect(self.record_value)
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.back_button)
        button_layout.addWidget(self.next_button)
        button_layout.addWidget(self.record_button)
        container_layout.addLayout(button_layout)
        self.layout.addWidget(container, alignment=Qt.AlignTop)

    def go_next_image(self):
        if self.current_image_index < len(self.image_paths) - 1:
            self.current_image_index += 1
            self.image_stack.setCurrentIndex(self.current_image_index)

    def go_back_image(self):
        if self.current_image_index > 0:
            self.current_image_index -= 1
            self.image_stack.setCurrentIndex(self.current_image_index)

    def record_value(self):
        if self.current_image_index == 0:
            print("calibrate top left")
        elif self.current_image_index == 1:
            print("calibrate top right")
        elif self.current_image_index == 2:
            print("calibrate bottom left")
        self.send_socket_message(f"calibrate {self.current_image_index}")

    def send_socket_message(self, message):
        host = '127.0.0.1'
        port = 65432
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((host, port))
            s.sendall(message.encode('utf-8'))

class CommandRunner(QThread):
    command_output = Signal(str)
    command_error = Signal(str)
    command_input_required = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.process = None

    def run(self, command):
        print("CommandRunner thread started")
        try:
            script_path = os.path.join(os.path.dirname(__file__), 'run_line_script.sh')
            self.process = subprocess.Popen(
                [script_path],
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE,
                bufsize=1,
                text=True,
                executable='/bin/bash'
            )
            while True:
                ready_to_read, _, _ = select.select([self.process.stdout, self.process.stderr], [], [])
                if self.process.stdout in ready_to_read:
                    output = self.process.stdout.readline()
                    if output:
                        self.command_output.emit(output.strip())
                if self.process.stderr in ready_to_read:
                    error = self.process.stderr.readline()
                    if error:
                        print("Command error:", error.strip())
                        self.command_error.emit(error.strip())
                if self.process.poll() is not None:
                    break
            for output in iter(self.process.stdout.readline, ''):
                if output:
                    print("Command output:", output.strip())
                    self.command_output.emit(output.strip())
            for error in iter(self.process.stderr.readline, ''):
                if error:
                    print("Command error:", error.strip())
                    self.command_error.emit(error.strip())
        except subprocess.CalledProcessError as e:
            print("Command execution failed")
            print("Error:", e.stderr)
            self.command_error.emit(e.stderr)

    @Slot(str)
    def send_input(self, user_input):
        if self.process:
            self.process.stdin.write(user_input + '\n')
            self.process.stdin.flush()

class CapturePage(BasePage):
    def __init__(self, image_path, background_color=None, parent=None):
        super().__init__("Cornelius Capture | Contour", image_path, background_color, parent)
        self.svgName = ""
        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setStyleSheet("background-color: black; color: white;")
        self.layout.addWidget(self.console_output)
        run_command_button = QPushButton("Run ROS Command")
        run_command_button.setStyleSheet("color: white; background-color: #3244a8; padding: 10px;")
        self.layout.addWidget(run_command_button)
        self.user_input = QLineEdit()
        self.user_input.setStyleSheet("color: white; background-color: #1c1b1c; padding: 10px;")
        self.layout.addWidget(self.user_input)
        send_input_button = QPushButton("Send Input")
        send_input_button.setStyleSheet("color: white; background-color: #3c32a8; padding: 10px;")
        self.layout.addWidget(send_input_button)
        record_button = QPushButton("Record")
        record_button.setStyleSheet("color: white; background-color: #32a852; padding: 10px;")
        self.layout.addWidget(record_button)
        run_command_button.clicked.connect(self.run_ros_command)
        send_input_button.clicked.connect(self.send_input)
        record_button.clicked.connect(self.record_contour)

    def run_ros_command(self):
        self.console_output.append("Running ROS command...")
        print("Starting CommandRunner thread")
        self.command_runner = CommandRunner()
        self.command_runner.command_output.connect(self.display_output)
        self.command_runner.command_error.connect(self.display_error)
        self.command_runner.command_input_required.connect(self.request_input)
        self.command_runner.start()

    def display_output(self, output):
        self.console_output.append(output)

    def display_error(self, error):
        self.console_output.append("Error occurred while running the command:")
        self.console_output.append(error)

    def request_input(self, prompt):
        self.console_output.append(prompt)

    def send_input(self):
        user_input = self.user_input.text()
        self.user_input.clear()

        if user_input != "0" or user_input != "1":
            self.svgName = user_input

        self.console_output.append(f"Sending input: {user_input}")
        self.command_runner.send_input(user_input)

    def record_data(self):
        message = f"Recording data from page: {self.image_stack.currentIndex() + 1}"
        print(message)  # Log the message instead of sending it to ROS
        self.console_output.append(message)
        self.send_socket_message(message)

    def record_contour(self):
        if self.svgName == "":
            message = "Please save a contour before recording"
            print(message)
        else:
            message = "contour filename: " + self.svgName + ".svg"
            print(message)  # Log the message instead of sending it to ROS
            self.console_output.append(message)
            self.send_socket_message(message)

    def send_socket_message(self, message):
        host = '127.0.0.1'
        port = 65432
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((host, port))
            s.sendall(message.encode('utf-8'))

class ContourPage(BasePage):
    def __init__(self, image_path, background_color=None, parent=None):
        super().__init__("Cornelius Caress", image_path, background_color, parent)
        contour_label = QLabel("words")
        contour_label.setStyleSheet("color: white;")
        self.layout.addWidget(contour_label)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.calibrated_0 = False
        self.calibrated_1 = False
        self.calibrated_2 = False
        self.calibrated_3 = False

        self.setWindowTitle("Cornelius Demon Interface")
        self.setGeometry(100, 100, 800, 600)
        self.layout = QVBoxLayout()
        self.stack = QStackedWidget(self)
        self.layout.addWidget(self.stack)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        background_path = os.path.join(script_dir, "UI_files/background.png")
        background_color = "#141414"
        self.stack.addWidget(CalibrationPage(background_path, background_color))
        self.stack.addWidget(CapturePage(background_path, background_color))
        self.stack.addWidget(ContourPage(background_path, background_color))
        self.button_layout = QHBoxLayout()
        self.back_button = QPushButton("Back")
        self.next_button = QPushButton("Next")
        self.button_layout.addWidget(self.back_button)
        self.button_layout.addWidget(self.next_button)
        self.back_button.clicked.connect(self.go_back)
        self.next_button.clicked.connect(self.go_next)
        self.layout.addLayout(self.button_layout)
        self.setLayout(self.layout)

        self.message_receiver = MessageReceiver()
        self.message_receiver.message_received.connect(self.recieve_msg)
        self.message_receiver.start()

    @Slot(str)
    def recieve_msg(self, message):
        # Logic to display the message in the UI
        print(f"Received message: {message}")
        self.last_message = message

        if message == "calibrated 0":
            self.calibrated_0 = True
        
        if message == "calibrated 1":
            self.calibrated_1 = True
        
        if message == "calibrated 2":
            self.calibrated_2 = True
        
        if message == "calibrated 3":
            self.calibrated_3 = True

    def go_next(self):
        current_index = self.stack.currentIndex()

        # if current_index == 0:
        #     if self.calibrated_0 and self.calibrated_1 and self.calibrated_2 and self.calibrated_3:
        #         self.stack.setCurrentIndex(current_index + 1)

        # else:
        #     if current_index < self.stack.count() - 1:
        #         self.stack.setCurrentIndex(current_index + 1)
        
        if current_index < self.stack.count() - 1:
                self.stack.setCurrentIndex(current_index + 1)

    def go_back(self):
        current_index = self.stack.currentIndex()
        if current_index > 0:
            self.stack.setCurrentIndex(current_index - 1)

def main():
    print("Starting main function")
    app = QApplication(sys.argv)
    app.setApplicationDisplayName("Cornelius Demon Interface")
    app.setApplicationName("Cornelius Demon Interface")
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("Cornelius Demon Interface")
    app.setOrganizationDomain("Cornelius Demon Interface")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    icon = QIcon(script_dir + "/UI_files/devil_face.png")
    app.setWindowIcon(icon)
    main_window = MainWindow()
    main_window.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
