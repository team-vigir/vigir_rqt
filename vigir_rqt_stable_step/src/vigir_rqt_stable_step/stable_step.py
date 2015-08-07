import roslib
roslib.load_manifest('vigir_rqt_control_mode')

import rospy
import math
import copy

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, QAbstractListModel
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton

from flor_atlas_nav_msgs.msg import AtlasFootstepPlan
from atlas_msgs.msg import AtlasSimInterfaceState, VigirBehaviorStepData

#class SimpleList(QAbstractListModel):
#    def __init__(self, contents):
#        super(SimpleList, self).__init__()
#        self.contents = contents
#
#    def rowCount(self, parent):
#        return len(self.contents)
#
#    def data(self, index, role):
#        if role == DisplayRole:
#            return str(self.contents[index.row()])
#

class StableStepDialog(Plugin):

    def __init__(self, context):
        super(StableStepDialog, self).__init__(context)
        self.setObjectName('StableStepDialog')

        self.bdi_state_msg  = AtlasSimInterfaceState()
        self.footstep_plan  = rospy.Publisher('/flor/controller/footstep_plan',AtlasFootstepPlan, None, False, True, None, queue_size=10)
        self.bdi_state_sub  = rospy.Subscriber("/atlas/atlas_sim_interface_state",   AtlasSimInterfaceState,   self.simStateCallback)

        self._widget = QWidget()
        vbox = QVBoxLayout()
        apply_command = QPushButton("Re-Center Steps")
        apply_command.clicked.connect(self.apply_command_callback)
        vbox.addWidget(apply_command)

        vbox.addStretch(1)
        self._widget.setLayout(vbox)

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        print "Shutting down ..."
        self.footstep_plan.unregister()
        self.bdi_state_sub.unregister()
        print "Done!"

    def apply_command_callback(self):

        print "Generate re-centering footsteps ..."
        steps = []

        L      = 0.15 #self.params["Forward Stride Length"]["value"]
        L_lat  = 0.15 #self.params["Lateral Stride Length"]["value"]
        W      = 0.25 #self.params["Stride Width"]["value"]

        dX     = copy.deepcopy(self.bdi_state_msg.foot_pos_est[1].position.x - self.bdi_state_msg.foot_pos_est[0].position.x)
        dY     = copy.deepcopy(self.bdi_state_msg.foot_pos_est[1].position.y - self.bdi_state_msg.foot_pos_est[0].position.y)

        dW     = math.sqrt(dX*dX + dY*dY)
        if (dW < 0.01):
            print "Invalid starting feet position! dW=",dW
            return

        if (math.fabs(dW-W) < 0.01):
            print "Feet are OK as is - publish empty plan !"
            footstep_plan = AtlasFootStepPlan()
            footstep_plan.header.stamp = rospy.get_rostime()
            footstep_plan.pos_est      = self.bdi_state_msg.pos_est
            footstep_plan.step_plan    = []
            self.footstep_plan.publish(footstep_plan)
        else:
        # Need to centering step
            center_x = copy.deepcopy(self.bdi_state_msg.foot_pos_est[0].position.x + 0.5*dX)
            center_y = copy.deepcopy(self.bdi_state_msg.foot_pos_est[0].position.y + 0.5*dY)

            # unit vector pointing from left foot to right
            dx = dX/dW
            dy = dY/dW

            left_x   = copy.deepcopy(center_x - W*0.5*dx - 0.1*dy)
            left_y   = copy.deepcopy(center_y - W*0.5*dy + 0.1*dx)

            right_x   = copy.deepcopy(center_x + W*0.5*dx - 0.1*dy)
            right_y   = copy.deepcopy(center_y + W*0.5*dy + 0.1*dx)

            # Right stance
            home_step = VigirBehaviorStepData()
            home_step.step_index = 0
            home_step.foot_index = 1
            home_step.pose = copy.deepcopy(self.bdi_state_msg.foot_pos_est[1]) # use right as dummy footstep to calc offset properly
            home_step.duration        = 0.6
            home_step.swing_height    = 0.1
            steps.append(home_step)

            # Left mid point
            home_step = VigirBehaviorStepData()
            home_step.step_index = 1
            home_step.foot_index = 0
            home_step.pose = copy.deepcopy(self.bdi_state_msg.foot_pos_est[0]) # use right as dummy footstep to calc offset properly
            home_step.pose.position.x = copy.deepcopy(0.5*(self.bdi_state_msg.foot_pos_est[0].position.x + left_x))
            home_step.pose.position.y = copy.deepcopy(0.5*(self.bdi_state_msg.foot_pos_est[0].position.y + left_y))
            home_step.duration        = 0.6
            home_step.swing_height    = 0.1
            steps.append(home_step)

            # Right mid point
            home_step = VigirBehaviorStepData()
            home_step.step_index = 2
            home_step.foot_index = 1
            home_step.pose = copy.deepcopy(self.bdi_state_msg.foot_pos_est[1]) # use right as dummy footstep to calc offset properly
            home_step.pose.position.x = 0.5*(self.bdi_state_msg.foot_pos_est[1].position.x + right_x)
            home_step.pose.position.y = 0.5*(self.bdi_state_msg.foot_pos_est[1].position.y + right_y)
            home_step.duration        = 0.6
            home_step.swing_height    = 0.1
            steps.append(home_step)

            # Left center
            home_step = VigirBehaviorStepData()
            home_step.step_index = 3
            home_step.foot_index = 0
            home_step.pose = copy.deepcopy(self.bdi_state_msg.foot_pos_est[0]) # use right as dummy footstep to calc offset properly
            home_step.pose.position.x = copy.deepcopy(left_x)
            home_step.pose.position.y = copy.deepcopy(left_y)
            home_step.duration        = 0.6
            home_step.swing_height    = 0.1
            steps.append(home_step)

            # Right center
            home_step = VigirBehaviorStepData()
            home_step.step_index = 4
            home_step.foot_index = 1
            home_step.pose = copy.deepcopy(self.bdi_state_msg.foot_pos_est[1]) # use right as dummy footstep to calc offset properly
            home_step.pose.position.x = copy.deepcopy(right_x)
            home_step.pose.position.y = copy.deepcopy(right_y)
            home_step.duration        = 0.6
            home_step.swing_height    = 0.1
            steps.append(home_step)

            print "Publish plan ..."
            footstep_plan = AtlasFootstepPlan()
            footstep_plan.header.stamp = rospy.get_rostime()
            footstep_plan.pos_est      = self.bdi_state_msg.pos_est
            footstep_plan.step_plan    = steps
            self.footstep_plan.publish(footstep_plan)
            print footstep_plan

    # Update BDI state
    def simStateCallback(self, state_msg):
        self.bdi_state_msg = state_msg


