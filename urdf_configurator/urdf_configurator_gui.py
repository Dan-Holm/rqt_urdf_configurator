# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import random
import signal
import sys
import threading

import rclpy

from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QFormLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit, QComboBox
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtCore import QRect
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QFontMetrics


from urdf_configurator.urdf_configurator import UrdfConfigurator

RANGE = 10000
LINE_EDIT_WIDTH = 45
SLIDER_WIDTH = 200
INIT_NUM_SLIDERS = 7  # Initial number of sliders to show in window

# Defined by style - currently using the default style
DEFAULT_WINDOW_MARGIN = 11
DEFAULT_CHILD_MARGIN = 9
DEFAULT_BTN_HEIGHT = 25
DEFAULT_SLIDER_HEIGHT = 64  # Is the combination of default heights in Slider

# Calculate default minimums for window sizing
MIN_WIDTH = SLIDER_WIDTH + DEFAULT_CHILD_MARGIN * 4 + DEFAULT_WINDOW_MARGIN * 2
MIN_HEIGHT = DEFAULT_BTN_HEIGHT * 2 + DEFAULT_WINDOW_MARGIN * 2 + DEFAULT_CHILD_MARGIN * 2

class Slider(QWidget):
    def __init__(self, name):
        super().__init__()

        self.joint_layout = QVBoxLayout()
        self.row_layout = QHBoxLayout()

        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout.addWidget(self.label)

        self.display = QLineEdit("0.00")
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setReadOnly(True)
        self.display.setFixedWidth(LINE_EDIT_WIDTH)
        self.row_layout.addWidget(self.display)

        self.joint_layout.addLayout(self.row_layout)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, RANGE)
        self.slider.setValue(int(RANGE / 2))
        self.slider.setFixedWidth(SLIDER_WIDTH)

        self.joint_layout.addWidget(self.slider)

        self.setLayout(self.joint_layout)

    def remove(self):
        self.joint_layout.removeWidget(self.slider)
        self.slider.setParent(None)

        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)

        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)

        self.row_layout.setParent(None)


class MyPopup(QWidget):
    return_inputs_signal = pyqtSignal(list)

    def __init__(self):
        QWidget.__init__(self)
        self.title = 'Add assembly'
        self.left = 10
        self.top = 10
        self.width = 400
        self.height = 140
        self.inputs = []
    
    def initPopup(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
    
        # Create a button in the window
        self.button = QPushButton('Add assembly', self)
        self.button.move(20,80)

        # connect button to function on_click
        self.button.clicked.connect(self.on_click)
        self.show()

    def add_input(self, name, input):
         # Create textbox
        title = QLabel(name, self)
        textbox = QLineEdit(self)
        textbox.setObjectName(input)

        if self.inputs:
            textbox.move(self.inputs[-1].geometry().right() + 20, 20)
        else:
            textbox.move(20, 20)

        title.move(textbox.geometry().left(), textbox.geometry().top() - 20)
        title.font = QFont("Helvetica", 9, QFont.Bold)

        textbox.resize(100,30)

        title.show()
        textbox.show()
        
        self.inputs.append(textbox)

    def add_dropdown(self, name, input, options):
        # Create dropdown with title above
        title = QLabel(name, self)
        dropdown = QComboBox(self)
        dropdown.setObjectName(input)
        dropdown.addItems(options)

        if self.inputs:
            dropdown.move(self.inputs[-1].geometry().right() + 20, 20)
        else:
            dropdown.move(20, 20)

        title.move(dropdown.geometry().left(), dropdown.geometry().top() - 20)
        title.font = QFont("Helvetica", 9, QFont.Bold)

        # Set size of dropdown to the maximum size of the options
        max_width = 0
        for option in options:
            width = QFontMetrics(QFont("Helvetica", 9, QFont.Bold)).width(option)
            if width > max_width:
                max_width = width

        dropdown.resize(max_width+40,30)

        title.show()
        dropdown.show()
        
        self.inputs.append(dropdown)

    def resizeWidget(self):
        self.width = self.inputs[-1].geometry().right() + 20
        self.setGeometry(self.left, self.top, self.width, self.height)

    def on_click(self):
        # TODO: Create checks if inputs are valid
        # TODO: - either here, og in the receive_inputs function to use a urdf "validation check" from urdf_parser

        self.return_inputs_signal.emit(self.inputs)
        self.close()


class urdfConfiguratorGUI(QMainWindow):
    def __init__(self, name, configurator):
        super(urdfConfiguratorGUI, self).__init__()


        self.setWindowTitle(name)

        # Button for updating the configuration
        self.update_button = QPushButton("Update", self)
        self.update_button.clicked.connect(self.update)

        # Button for adding a new link and joint
        self.add_link_button = QPushButton("Add link", self)
        self.add_link_button.clicked.connect(self.add_link)

        # Button for removing a link or joint
        self.remove_link_button = QPushButton("Remove link", self)
        self.remove_link_button.clicked.connect(self.remove_link)

        # Button for saving the configuration
        self.save_button = QPushButton("Save", self)
        self.save_button.clicked.connect(self.save_config)

        # Main layout
        self.main_layout = QVBoxLayout()

        # Add buttons and scroll area to main layout
        self.main_layout.addWidget(self.update_button)
        self.main_layout.addWidget(self.add_link_button)
        self.main_layout.addWidget(self.remove_link_button)
        self.main_layout.addWidget(self.save_button)

        # central widget
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)

        self.configurator = configurator



    def receive_assembly_inputs(self, inputs):

        inputmap = {'ln': '', 'jn': '', 'pl': '', 'cl': '', 'jt': '', 'xyz': '', 'rpy': ''}
        for input in inputs:
            if isinstance(input, QLineEdit):
                inputmap[input.objectName()] = input.text()
                print(input.objectName(), input.text())
            elif isinstance(input, QComboBox):
                inputmap[input.objectName()] = input.currentText()
                print(input.objectName(), input.currentText())

        self.configurator.add_assembly(inputmap['ln'], inputmap['jn'], inputmap['pl'], inputmap['cl'], inputmap['jt'], inputmap['xyz'], inputmap['rpy'])
        self.configurator.update_robot()

    @pyqtSlot()
    def update(self):
        self.configurator.update_robot()
        pass

    def add_link(self):
        self.w = MyPopup()
        # self.w.setGeometry(QRect())
        self.w.initPopup()
        self.w.add_input("Link name", "ln")
        self.w.add_input("Joint name", "jn")
        self.w.add_dropdown("Parent link", "pl", self.configurator.get_link_names())
        self.w.add_dropdown("Child link", "cl", self.configurator.get_link_names())
        self.w.add_dropdown("Joint type", "jt", ["fixed", "revolute", "continuous", "prismatic", "floating", "planar"])
        self.w.add_input("Joint origin", "xyz")
        self.w.add_input("Joint origin", "rpy")
        self.w.resizeWidget()

        self.w.return_inputs_signal.connect(self.receive_assembly_inputs)


    def remove_link(self):
        pass

    def save_config(self):
        pass


    def loop(self):
        while True:
            rclpy.spin_once(self.configurator, timeout_sec=0.1)





def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file', help='URDF file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    app = QApplication(sys.argv)
    configurator_gui = urdfConfiguratorGUI('URDF Configurator', UrdfConfigurator(parsed_args.urdf_file))

    configurator_gui.show()

    threading.Thread(target=configurator_gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
