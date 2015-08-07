import roslib
roslib.load_manifest('vigir_rqt_ghost_robot_control')

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QWidget, QVBoxLayout, QPushButton, QCheckBox

from sensor_msgs.msg import JointState

import rospy
#from flor_motion.motion_publisher import MotionPublisher

#from .position_editor_widget import PositionEditorWidget
#from .motion_editor_widget import MotionEditorWidget


class GhostRobotControlPlugin(Plugin):
    updateStateSignal = Signal(object)

    def __init__(self, context):
        super(GhostRobotControlPlugin, self).__init__(context)
        self.setObjectName('GhostRobotControlPlugin')


        self.joint_states_to_ghost_pub = rospy.Publisher('/joint_states_to_ghost', JointState, queue_size=10)

        self.ghost_joint_states_sub = rospy.Subscriber('/ghost/joint_states', JointState, self.ghost_joint_states_cb)
        self.real_joint_states_sub = rospy.Subscriber('/atlas/joint_states', JointState, self.real_joint_states_cb)


        self.ghost_joint_states = JointState()
        self.real_joint_states = JointState()

        self.move_real_robot = False


        self.widget = QWidget()
        vbox = QVBoxLayout()


        self.real_to_ghost_push_button = QPushButton('Set Ghost from real robot')
        self.real_to_ghost_push_button.clicked.connect(self.handle_set_real_to_ghost)
        vbox.addWidget(self.real_to_ghost_push_button)

        self.send_motion_plan_to_real_robot_check_box = QCheckBox('Motion GUI moves real robot')
        self.send_motion_plan_to_real_robot_check_box.stateChanged.connect(self.handle_send_motion_plan_to_real_robot_check_box)
        vbox.addWidget(self.send_motion_plan_to_real_robot_check_box)

        self.widget.setLayout(vbox)

        context.add_widget(self.widget)
        

    def ghost_joint_states_cb(self, data):
        self.ghost_joint_states = data

    def real_joint_states_cb(self, data):
        self.real_joint_states = data

    def handle_set_real_to_ghost(self):
        self.joint_states_to_ghost_pub.publish(self.real_joint_states)

    @Slot(bool)
    def handle_send_motion_plan_to_real_robot_check_box(self, checked):
        self.move_real_robot = checked



    #def shutdown_plugin(self):
        #self._widget.shutdown()

