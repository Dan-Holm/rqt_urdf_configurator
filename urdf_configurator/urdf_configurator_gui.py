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
import os
from ament_index_python import get_resource

import threading

import rclpy

from tf2_msgs.srv import FrameGraph
import tf2_ros

from python_qt_binding import loadUi
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
from python_qt_binding.QtWidgets import QSlider, QGraphicsScene
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget, QFileDialog
from python_qt_binding.QtGui import QPainter, QIcon, QImage

from python_qt_binding.QtCore import QRect
from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QFontMetrics, QPixmap

from qt_dotgraph.pydotfactory import PydotFactory
from qt_dotgraph.dot_to_qt import DotToQtGenerator, NodeItem
from rqt_graph.interactive_graphics_view import InteractiveGraphicsView 


from .urdf_configurator import UrdfConfigurator, assemblySetup
from .dotcode_tf import RosTfTreeDotcodeGenerator


RANGE = 10000
LINE_EDIT_WIDTH = 45
SLIDER_WIDTH = 200
INIT_NUM_SLIDERS = 7  # Initial number of sliders to show in window

# Defined by style - currently using the default style
DEFAULT_WINDOW_MARGIN = 11
DEFAULT_CHILD_MARGIN = 9
DEFAULT_BTN_HEIGHT = 25
DEFAULT_SEPERATION_MARGIN = 20
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
        self.left = 100
        self.top = 10
        self.width = 70
        self.height = 540
        self.inputs = []
    
    def initPopup(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
    
        # Create a button in the window
        self.button = QPushButton('Add assembly', self)
        self.button.move(DEFAULT_SEPERATION_MARGIN, self.height - 40)

        # connect button to function on_click
        self.button.clicked.connect(self.on_click)
        self.show()

    def add_input(self, name, input):
         # Create textbox
        title = QLabel(name, self)
        textbox = QLineEdit(self)
        textbox.setObjectName(input)

        if self.inputs:
            textbox.move(DEFAULT_SEPERATION_MARGIN, self.inputs[-1].geometry().bottom() + 30)
        else:
            textbox.move(DEFAULT_SEPERATION_MARGIN, DEFAULT_SEPERATION_MARGIN)

        title.move(textbox.geometry().left(), textbox.geometry().top() - DEFAULT_SEPERATION_MARGIN)
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
            dropdown.move(DEFAULT_SEPERATION_MARGIN, self.inputs[-1].geometry().bottom() + 30)
        else:
            dropdown.move(DEFAULT_SEPERATION_MARGIN, DEFAULT_SEPERATION_MARGIN)

        title.move(dropdown.geometry().left(), dropdown.geometry().top() - DEFAULT_SEPERATION_MARGIN)
        title.font = QFont("Helvetica", 9, QFont.Bold)

        # Set size of dropdown to the maximum size of the options
        max_width = 0
        for option in options:
            width = QFontMetrics(QFont("Helvetica", 9, QFont.Bold)).width(option)
            if width > max_width:
                max_width = width
        dropdown.resize(max_width+40,30)

        # save max size for a proper sized GUI
        if self.width < max_width + DEFAULT_SEPERATION_MARGIN:
            self.width = max_width + DEFAULT_SEPERATION_MARGIN

        title.show()
        dropdown.show()
        
        self.inputs.append(dropdown)

    def fileBrowserButton(self, name):
        browserButton = QPushButton(name, self)
        self.fileTextbox = QLineEdit(self)

        if self.inputs:
            browserButton.move(DEFAULT_SEPERATION_MARGIN, self.inputs[-1].geometry().bottom() + 30)
            self.fileTextbox.move(browserButton.geometry().right() + DEFAULT_SEPERATION_MARGIN, browserButton.geometry().top())
        else:
            browserButton.move(DEFAULT_SEPERATION_MARGIN, DEFAULT_SEPERATION_MARGIN)
            self.fileTextbox.move(browserButton.geometry().right() + DEFAULT_SEPERATION_MARGIN, browserButton.geometry().top())
        browserButton.clicked.connect(self.getfile)

        browserButton.show()
        self.fileTextbox.show()



		

    def getfile(self):
        self.fname = QFileDialog.getOpenFileName(self, 'Open file', 
         '/home/daniel/master_ws/src/Danitech-master/wagon_description/meshes/bases/',"3D mesh files (*.shp *.dxf *.dae, *.stl)")
        # show fname in fileTextbox
        print("mesh filename", self.fname[0])
        self.fileTextbox.setInputMask(self.fname[0])
        self.inputs.append(self.fname)



        

    def resizeWidget(self):
        self.setGeometry(self.left, self.top, self.width, self.height)

    def on_click(self):
        # TODO: Create checks if inputs are valid
        # TODO: - either here, og in the receive_inputs function to use a urdf "validation check" from urdf_parser

        self.return_inputs_signal.emit(self.inputs)
        self.close()



class dotGraphwidget(QWidget):
    def __init__(self, urdf, parent=None):
        super(dotGraphwidget, self).__init__(parent)

        self.last_drawargs = None

        self.dotcode_factory = PydotFactory()
        self.dotcode_generator = RosTfTreeDotcodeGenerator()
        self.urdf_data = urdf
        self._current_dotcode = None
        QWidget.__init__(self)
        self._widget = QWidget()

        # self.generate_dotcode()



    def generate_dotcode(self,
                         rank='same',   # None, same, min, max, source, sink
                         ranksep=0.2,   # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         ):


        drawing_args = {
            'dotcode_factory': self.dotcode_factory,
            'rank': rank,
            'rankdir': rankdir,
            'ranksep': ranksep}

        selection_changed = False
        if self.last_drawargs != drawing_args:
            selection_changed = True
            self.last_drawargs = drawing_args

            self.dotcode_factory = self.dotcode_factory
            self.rank = rank
            self.rankdir = rankdir
            self.ranksep = ranksep

        # generate new dotcode
   
        self.graph = self.generate(self.urdf_data)
        self.dotcode = self.dotcode_factory.create_dot(self.graph)
        return self.dotcode

    def generate(self, urdf_data):
        # print("generating dotcode for: ", urdf_data.child_map)
        # print(urdf_data.link_map)
        # print(urdf_data.joint_map)
        graph = self.dotcode_factory.get_graph(rank=self.rank,
                                        rankdir=self.rankdir,
                                        ranksep=self.ranksep)

        if urdf_data is None or urdf_data.links is None:
            self.dotcode_factory.add_node_to_graph(graph, 'No urdf recieved')
            print("No urdf recieved")
            return graph

        root = urdf_data.links[0].name
        for parent in urdf_data.child_map:

            self.dotcode_factory.add_node_to_graph(graph,
                                                str(parent),
                                                shape='ellipse')
            for child in urdf_data.child_map[parent]:
                # print("parent:", parent, "child:", child)

                
                self.dotcode_factory.add_edge_to_graph(graph,
                                                       str(parent),
                                                       str(child[1]),
                                                       )


                # self.dotcode_factory.add_node_to_graph(
                #     graph, frame_dict, shape='ellipse')

                # self.dotcode_factory.add_edge_to_graph(graph,
                #                                     str(tf_frame_values['parent']),
                #                                     frame_dict,
                #                                     )
                
                # # self.dotcode_factory.add_edge_to_graph(graph,
                # #                                     str(tf_frame_values['parent']),
                # #                                     frame_dict,
                # #                                     label=edge_label)

        # create legend before root node
        legend_label = "urdf file"
        self.dotcode_factory.add_node_to_graph(graph, legend_label)
        self.dotcode_factory.add_edge_to_graph(graph,
                                               legend_label,
                                               root,
                                               style='invis')

        # dot += ' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";\n'
        # dot += '"Recorded at time: '+str(rospy.Time.now().to_sec())+'"[ shape=plaintext ] ;\n'
        # dot += '}->"'+root+'"[style=invis];\n}'
        return graph
    
class linkItem(NodeItem):
    
    def __init__(self, node_item):
        self.node_item = node_item
        


    def mousePressEvent(self, event):
        print("link clicked", event)

    def __getattr__(self, name):
        # Delegate attribute access to the original instance
        return getattr(self.node_item, name)





class urdfConfiguratorGUI():
    def __init__(self, name, urdf_configurator):
        super(urdfConfiguratorGUI, self).__init__()

        self.configurator = urdf_configurator

        # self.setWindowTitle(name)

        self.initialized = False

        # self.setObjectName('urdfConfigurator')


        self._current_dotcode = None

        self._widget = QWidget()
        self._widget.setObjectName('urdfConfiguratorUi')



        _, package_path = get_resource('packages', 'urdf_configurator')
        ui_file = os.path.join(package_path, 'share', 'urdf_configurator', 'resource', 'RosURDFConfig.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self.dotGraph = dotGraphwidget(self.configurator.get_robot())
        self.dot_to_qt = DotToQtGenerator()
        self._scene = QGraphicsScene()
        self._scene.setObjectName(name)

        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        # self._redraw_graph_view()
        self._update_graph_view(self.dotGraph.generate_dotcode())
        # if context.serial_number() > 1:
                # self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._widget.update_urdf_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.update_urdf_push_button.pressed.connect(self.update)



        self._widget.show()

    def _update_graph_view(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()


    def _redraw_graph_view(self):
        print("redraw graph view")
        self._scene.clear()

        if self._widget.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level)

        for node_item in nodes.values():
            link_item = linkItem(node_item)
            self._scene.addItem(link_item)
            #Make all node items to a button
            # node_item.setFlag(node_item.ItemIsSelectable, True)
            # connect keypresses to node_item
            

        for edge_items in edges.values():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)

        print(self._scene.items)
        # self._scene.setSceneRect(self._scene.itemsBoundingRect())
        # if self._widget.auto_fit_graph_check_box.isChecked():
        self._fit_in_view()


    def node_clicked(self, event):
        print("node clicked ", event)

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(),
                                             Qt.KeepAspectRatio)


    def oldWidget(self):

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


    def receive_assembly_inputs(self, inputs):

        inputmap = {'ln': '', 'jn': '', 'pl': '', 'cl': '', 'jt': '', 'xyz': '', 'rpy': '', 'mesh_file': ''}
        for input in inputs:
            if isinstance(input, QLineEdit):
                inputmap[input.objectName()] = input.text()
                print(input.objectName(), input.text())
            elif isinstance(input, QComboBox):
                inputmap[input.objectName()] = input.currentText()
                print(input.objectName(), input.currentText())
            elif isinstance(input, tuple):
                inputmap["mesh_file"] = input[0]
                print(input[0])
        self.new_assembly.configure_assembly(inputmap['ln'], inputmap['jn'], inputmap['pl'], inputmap['cl'], inputmap['jt'], inputmap['xyz'], inputmap['rpy'])
        
        self.configurator.add_link(self.new_assembly.get_link())
        self.configurator.update_robot()

    @pyqtSlot()

    def setupDocWidget(self):
        # factory builds generic dotcode items
        self.dotcode_factory = PydotFactory()
        # self.dotcode_factory = PygraphvizFactory()
        # generator builds rosgraph
        self.dotcode_generator = RosTfTreeDotcodeGenerator()
        self.tf2_buffer_ = tf2_ros.Buffer(node=self._node)
        self.tf2_listener_ = tf2_ros.TransformListener(self.tf2_buffer_, self._node)

        # dot_to_qt transforms into Qt elements using dot layout
        self.dot_to_qt = DotToQtGenerator()

    def update(self):
        self.configurator.update_robot()
        self._update_graph_view(self.dotGraph.generate_dotcode())

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
        self.w.fileBrowserButton("mesh file")

        self.w.resizeWidget()

        self.new_assembly = assemblySetup()

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

    # configurator_gui.show()

    threading.Thread(target=configurator_gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
