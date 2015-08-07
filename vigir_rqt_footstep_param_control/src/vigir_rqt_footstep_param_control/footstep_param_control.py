import roslib
roslib.load_manifest('vigir_rqt_footstep_param_control')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, QAbstractListModel, Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QComboBox

from footstep_param_control_widget import FootstepParamControlWidget

class FootstepParamControlDialog(Plugin):

    def __init__(self, context):
        super(FootstepParamControlDialog, self).__init__(context)
        self.setObjectName('FootstepParamControlDialog')

        self._parent = QWidget()
        self._widget = FootstepParamControlWidget(self._parent)
        
        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

