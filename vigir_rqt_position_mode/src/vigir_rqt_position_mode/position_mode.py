import roslib
roslib.load_manifest('vigir_rqt_position_mode')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, QAbstractListModel, Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QComboBox

from position_mode_widget import PositionModeWidget

class PositionModeDialog(Plugin):

    def __init__(self, context):
        super(PositionModeDialog, self).__init__(context)
        self.setObjectName('PositionModeDialog')

        self._parent = QWidget()
        self._widget = PositionModeWidget(self._parent)
        
        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

