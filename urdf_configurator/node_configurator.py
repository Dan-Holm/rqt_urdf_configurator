from python_qt_binding.QtWidgets import QWidget, QFileDialog, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QStackedWidget, QLineEdit, QSlider
from python_qt_binding.QtCore import Qt, pyqtSlot, pyqtSignal
from python_qt_binding.QtGui import QFont
from urdf_configurator import urdf_configurator



RANGE = 200
LINE_EDIT_WIDTH = 90
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
    sliderUpdateTrigger = pyqtSignal(int)

    def __init__(self, name, value=0, angular=False):
        super().__init__()

        self.joint_layout = QVBoxLayout()
        self.row_layout = QHBoxLayout()

        self.link_value = value
        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout.addWidget(self.label)

        self.display = QLineEdit("0.00")
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setFixedWidth(LINE_EDIT_WIDTH)
        self.display.editingFinished.connect(self.display_edited)
        # Connect self.display to function that updates the value
        


        self.row_layout.addWidget(self.display)

        self.joint_layout.addLayout(self.row_layout)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.angular = angular

        if self.angular:
            self.slider.setRange(-314, 314)
            # self.slider.setTickInterval(0.01)
            self.slider.setValue(int(value*100))
            self.display.setText(str(value))
        else:
            self.slider.setRange(-RANGE, RANGE)
            # self.slider.setTickInterval(0.1)
            self.slider.setValue(int(value))
            self.display.setText(str(value))

        self.slider.setFixedWidth(SLIDER_WIDTH)


        self.slider.valueChanged.connect(self.update)

        self.joint_layout.addWidget(self.slider)

        self.setLayout(self.joint_layout)

    def display_edited(self):
        self.slider.setValue(int(float(self.display.text())*100))
        # will trigger the update function



    def remove(self):
        self.joint_layout.removeWidget(self.slider)
        self.slider.setParent(None)

        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)

        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)

        self.row_layout.setParent(None)

    # def sliderUpdateConnection(self, callback):
    #     self.sliderUpdateTrigger.connect(callback)
    #     self.sliderUpdateTrigger.emit()

    def update(self):
        value = self.slider.value()
        print("Slider Value changed, updating to, ", value)
        self.display.setText(str(value/100))
        self.link_value = value / 100
        self.sliderUpdateTrigger.emit(value)




class NodeConfigurator(QWidget):
    def __init__(self, node, menu):
        super(NodeConfigurator, self).__init__()
        self.setObjectName('NodeConfigurator')

        self.active_node = node
        self.node_menu = menu
        # self.node_menu.linkName.setText(self.active_node.name)

        self.generate_visuals_editor()

        self._stacked_widget = QStackedWidget()
        self._stacked_widget.addWidget(self.node_menu)
        self._stacked_widget.addWidget(self.visuals_editor)

        self._stacked_widget.setCurrentIndex(0)
        self._stacked_widget.show()



    def link_menu(self):
        # Remove current widget from modification area
        # print("Current modification area widget", self._widget.modification_area.widget().objectName())
        # self._widget.modification_area.takeWidget()
        # print("next modification area widget", self._widget.modification_area.widget())

        self._widget.modification_area.setWidget(self.node_menu)
        self._widget.linkName.setText(self.active_node.name)
     
    def generate_visuals_editor(self):
        # Add input fields for the pose of the node to modification_area
         # Horisontal layout with 3 sliders for rpy values
        
        self.visuals_editor = QWidget()
        self.visuals_editor.setObjectName('visuals editorx')
        self.visuals_editor.setLayout(QVBoxLayout())

        # TODO: Add collision option
            # length = Slider("Length", link.length)
            # radius = Slider("Radius", link.geometry.radius)
        
        title = QLabel(self.active_node.name)
        title.font = QFont("Helvetica", 9, QFont.Bold)
        self.visuals_editor.layout().addWidget(title)
        sliders = {}
        roll = Slider("Roll", self.active_node.visual.origin.rpy[0], angular=True)
        sliders["roll"] = roll
        pitch = Slider("Pitch", self.active_node.visual.origin.rpy[1], angular=True)
        sliders["pitch"] = pitch
        yaw = Slider("Yaw", self.active_node.visual.origin.rpy[2], angular=True)
        sliders["yaw"] = yaw
        x = Slider("X", self.active_node.visual.origin.xyz[0], angular=False)
        sliders["x"] = x
        y = Slider("Y", self.active_node.visual.origin.xyz[1], angular=False)
        sliders["y"] = y
        z = Slider("Z", self.active_node.visual.origin.xyz[2], angular=False)
        sliders["z"] = z

        self.connectSliders(sliders)


    def connectSliders(self, sliders):
      for slider_key, slider in sliders.items():
            self.visuals_editor.layout().addWidget(slider)
            slider.sliderUpdateTrigger.connect(self.sliderUpdate)
            # slider.sliderUpdateTrigger.emit(slider)


    @pyqtSlot()
    def sliderUpdate(self):
        slider = self.sender()  # Get the sender of the signal
        if slider.angular:
            axis = ["Roll", "Pitch", "Yaw"].index(slider.label.text())
            print( "axis no for ", slider.label.text(), axis)
            self.active_node.visual.origin.rpy[axis] = slider.link_value
        else:
            axis = ["X", "Y", "Z"].index(slider.label.text())
            self.active_node.visual.origin.xyz[axis] = slider.link_value
        self.configurator.update_robot()