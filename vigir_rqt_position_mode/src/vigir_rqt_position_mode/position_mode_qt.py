import roslib
roslib.load_manifest('vigir_rqt_position_mode')

import rospy

from python_qt_binding.QtCore import Slot, QCoreApplication, QBasicTimer, QAbstractListModel, Qt, QSettings
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QComboBox, QApplication

from position_mode_widget import PositionModeWidget

from std_msgs.msg import Int8

class PositionModeDialog(QWidget):

    def __init__(self):
        super(PositionModeDialog, self).__init__(None)

        self._widget = PositionModeWidget(self)
        self._window_control_sub = rospy.Subscriber("/flor/ocs/window_control", Int8, self.processWindowControl)
        self._window_control_pub = rospy.Publisher("/flor/ocs/window_control", Int8, queue_size=10)

        # #define WINDOW_POSITION_MODE       8
        self._window = 8
        
        # window visibility configuration variables
        self._last_window_control_data = -self._window
        self._set_window_visibility = True

        # this is only used to make sure we close window if ros::shutdown has already been called
        self._timer = QBasicTimer()
        self._timer.start(33, self)

        settings = QSettings("OCS", "position_mode")
        if settings.value("mainWindowGeometry") is not None:
            self.restoreGeometry(settings.value("mainWindowGeometry"))
        self.geometry_ = self.geometry()

    def processWindowControl(self, visible):
        # set window visibility changed flag
        self._set_window_visibility = True
        self._last_window_control_data = visible.data

    def timerEvent(self, event):
        if rospy.is_shutdown():
            QCoreApplication.instance().quit()
            
        # needed to move qt calls out of the ros callback, otherwise qt crashes because of inter-thread communication
        if self._set_window_visibility:
            self._set_window_visibility = False
            if not self.isVisible() and self._last_window_control_data == self._window:
                self.show()
                self.setGeometry(self._geometry)
            elif self.isVisible() and (not self._last_window_control_data or self._last_window_control_data == -self._window):
                self._geometry = self.geometry()
                self.hide()

    def closeEvent(self, event):
        settings = QSettings("OCS", "position_mode")
        settings.setValue("mainWindowGeometry", self.saveGeometry())
        msg = Int8()
        msg.data = -self._window
        self._window_control_pub.publish(msg)
        event.ignore()

    def resizeEvent(self, event):
        settings = QSettings("OCS", "position_mode")
        settings.setValue("mainWindowGeometry", self.saveGeometry())

    def moveEvent(self, event):
        settings = QSettings("OCS", "position_mode")
        settings.setValue("mainWindowGeometry", self.saveGeometry())

