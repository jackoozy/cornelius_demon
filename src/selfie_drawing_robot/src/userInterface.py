import sys
import os
from PySide6.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QPushButton, QStackedWidget, QHBoxLayout, QSizePolicy, QSpacerItem
from PySide6.QtWidgets import QRadioButton, QGridLayout, QSpacerItem, QSizePolicy


from PySide6.QtGui import QIcon, QPixmap, QPainter, QColor
from PySide6.QtCore import Qt


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

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Cornelious Demon Interface")
        self.setGeometry(100, 100, 800, 600)

        # Main layout
        self.layout = QVBoxLayout()

        # Stack of windows
        self.stack = QStackedWidget(self)
        self.layout.addWidget(self.stack)

        # Add custom pages to stack
        script_dir = os.path.dirname(os.path.abspath(__file__))
        background_path = os.path.join(script_dir, "UI_files/background.png")
        background_color = "#141414"  # Set to your desired background color

        self.stack.addWidget(CalibrationPage(background_path, background_color))
        self.stack.addWidget(CapturePage(background_path, background_color))
        self.stack.addWidget(ContourPage(background_path, background_color))

        # Navigation buttons
        self.button_layout = QHBoxLayout()
        self.back_button = QPushButton("Back")
        self.next_button = QPushButton("Next")
        self.button_layout.addWidget(self.back_button)
        self.button_layout.addWidget(self.next_button)

        self.back_button.clicked.connect(self.go_back)
        self.next_button.clicked.connect(self.go_next)

        self.layout.addLayout(self.button_layout)
        self.setLayout(self.layout)

    def go_next(self):
        current_index = self.stack.currentIndex()
        if current_index < self.stack.count() - 1:
            self.stack.setCurrentIndex(current_index + 1)

    def go_back(self):
        current_index = self.stack.currentIndex()
        if current_index > 0:
            self.stack.setCurrentIndex(current_index - 1)


class BasePage(BackgroundWidget):
    def __init__(self, text, image_path, background_color=None, parent=None):
        super().__init__(image_path, background_color, parent)
        
        # Layout for the page
        self.layout = QVBoxLayout(self)

        # Add the text label on top of the background
        self.label = QLabel(text)
        self.label.setStyleSheet("color: white; font-size: 24px; font-weight: bold;")
        self.label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self.layout.addWidget(self.label)

        # Add a spacer to push other content to the bottom
        spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.layout.addItem(spacer)


class CalibrationPage(BasePage):
    def __init__(self, image_path, background_color=None, parent=None):
        super().__init__("Cornelious Calibration", image_path, background_color, parent)
        
        # Create a container widget for custom content
        container = QWidget()
        container_layout = QVBoxLayout(container)


        # Create a grid layout for precise control over radio button positions
        grid_layout = QGridLayout()
        
        # Create radio buttons
        radio_button1 = QRadioButton("Option 1")
        radio_button2 = QRadioButton("Option 2")
        radio_button3 = QRadioButton("Option 3")
        
        # Set styles for the radio buttons
        radio_button1.setStyleSheet("color: white;")
        radio_button2.setStyleSheet("color: white;")
        radio_button3.setStyleSheet("color: white;")
        
        # Add radio buttons to the grid layout with specified positions
        grid_layout.addWidget(radio_button1, 0, 0)
        grid_layout.addWidget(radio_button2, 0, 2)
        grid_layout.addWidget(radio_button3, 1, 0)
        
        # Add the grid layout to the container layout
        container_layout.addLayout(grid_layout)
        
        
        # Create a label for the title
        title_label = QLabel("Calibration steps:")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; color: white;")
        title_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        container_layout.addWidget(title_label, alignment=Qt.AlignLeft | Qt.AlignTop)

        # Create a vertical layout for the steps
        steps_layout = QVBoxLayout()
        steps = [
            "1. Free-drive end-effector tool to the location selected above. Ensure the pencil is just contact the page and press record.",
            "2. Perform step 1 on the top left corner of the page.", 
            "3. Perform step 1 on the top right corner of the page."
        ]
        
        for step in steps:
            step_label = QLabel(step)
            step_label.setStyleSheet("font-size: 12px; color: white;")
            step_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            steps_layout.addWidget(step_label, alignment=Qt.AlignLeft | Qt.AlignTop)
        
        # Add the steps layout to the container layout
        container_layout.addLayout(steps_layout)
        
        # Add the container widget to the main layout
        self.layout.addWidget(container, alignment=Qt.AlignTop)


class CapturePage(BasePage):
    def __init__(self, image_path, background_color=None, parent=None):
        super().__init__("Cornelious Capture", image_path, background_color, parent)
        # Add custom widgets and layout for CapturePage here
        capture_label = QLabel("Capture content goes here")
        capture_label.setStyleSheet("color: white;")
        self.layout.addWidget(capture_label)

class ContourPage(BasePage):
    def __init__(self, image_path, background_color=None, parent=None):
        super().__init__("Cornelious Contour", image_path, background_color, parent)
        # Add custom widgets and layout for ContourPage here
        contour_label = QLabel("Contour content goes here")
        contour_label.setStyleSheet("color: white;")
        self.layout.addWidget(contour_label)




def main():
    app = QApplication(sys.argv)

    # Set application properties
    app.setApplicationDisplayName("Cornelious Demon Interface")
    app.setApplicationName("Cornelious Demon Interface")
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("Cornelious Demon Interface")
    app.setOrganizationDomain("Cornelious Demon Interface")

    # Get current file path
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Correctly set the window icon using QIcon
    icon = QIcon(script_dir + "/UI_files/devil_face.png")
    app.setWindowIcon(icon)

    main_window = MainWindow()
    main_window.show()

    # Start the event loop
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
