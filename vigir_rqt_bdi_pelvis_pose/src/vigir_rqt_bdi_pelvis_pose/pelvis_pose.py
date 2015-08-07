import roslib
roslib.load_manifest('vigir_rqt_joint_control')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, QAbstractListModel, Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QComboBox

from pelvis_pose_widget import BDIPelvisPoseWidget

class BDIPelvisPoseDialog(Plugin):

    def __init__(self, context):
        super(BDIPelvisPoseDialog, self).__init__(context)
        self.setObjectName('BDIPelvisPoseDialog')

        self._parent = QWidget()
        self._widget = BDIPelvisPoseWidget(self._parent)
        
        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

