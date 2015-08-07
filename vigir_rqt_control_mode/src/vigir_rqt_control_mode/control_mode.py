import roslib
roslib.load_manifest('vigir_rqt_control_mode')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, QAbstractListModel, Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QComboBox

from control_mode_widget import ControlModeWidget

class ControlModeDialog(Plugin):

    def __init__(self, context):
        super(ControlModeDialog, self).__init__(context)
        self.setObjectName('ControlModeDialog')

        self._parent = QWidget()
        self._widget = ControlModeWidget(self._parent)
        
        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

