import roslib
roslib.load_manifest('vigir_rqt_control_mode')

import rospy
import math

from python_qt_binding.QtCore import Slot, QCoreApplication, QBasicTimer, QAbstractListModel, Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QComboBox, QApplication

from control_mode_widget import ControlModeWidget

from std_msgs.msg import Int8

class ControlModeDialog(QWidget):

    def __init__(self):
        super(ControlModeDialog, self).__init__(None)

        self._control_mode_widget = ControlModeWidget(self)
        self._window_control_sub = rospy.Subscriber("/flor/ocs/window_control", Int8, self.processWindowControl)
        
        # #define WINDOW_CONTROL_MODE 9
        self._window = 9
        
        # window visibility configuration variables
        self._last_window_control_data = -self._window
        self._set_window_visibility = True

        # this is only used to make sure we close window if ros::shutdown has already been called
        self._timer = QBasicTimer()
        self._timer.start(33, self)

    def processWindowControl(self, visible):
        # set window visibility changed flag
        self._set_window_visibility = True
        self._last_window_control_data = visible.data

    def timerEvent(self, event):
        if rospy.is_shutdown():
            QCoreApplication.instance().quit();
            
        # needed to move qt calls out of the ros callback, otherwise qt crashes because of inter-thread communication
        if self._set_window_visibility:
            self._set_window_visibility = False
            if not self.isVisible() and self._last_window_control_data == self._window:
                self.show()
                self.setGeometry(self._geometry)
            elif self.isVisible() and (not self._last_window_control_data or self._last_window_control_data == -self._window):
                self._geometry = self.geometry()
                self.hide()

