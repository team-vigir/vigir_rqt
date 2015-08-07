import roslib
roslib.load_manifest('vigir_rqt_control_mode')

import rospy
import math

from python_qt_binding.QtCore import Slot, QAbstractListModel, Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QComboBox

from std_msgs.msg import Float64
from vigir_control_msgs.msg import VigirControlModeCommand, FlorControlMode, FlorGravityCompensationGain
from atlas_msgs.msg import AtlasSimInterfaceState, AtlasSimInterfaceCommand

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

class ControlModeWidget:

    def __init__(self, context):
      
        self.control_mode =  0
        self.mode_pub = rospy.Publisher('/flor/controller/mode_command', VigirControlModeCommand, queue_size=10)

        self._widget = context
        self.vbox = QVBoxLayout()

        #Add input for setting the spindle speed
        list_label = QLabel("Select Control Mode")
        self.vbox.addWidget(list_label)

        # Indexed list of allowed control modes from feedback
        self.allowed_modes = rospy.get_param("/atlas_controller/allowed_control_modes")

        self.allowed_modes=rospy.get_param("/atlas_controller/allowed_control_modes")
        self.mode_ids={}
        self.mode_ids
        for ndx,txt in enumerate(self.allowed_modes):
            self.mode_ids[txt] = ndx

        # API 2.13 ordering
        self.bdi_mode_names=['NONE       ',        'FREEZE     ',        'STAND_PREP ', \
                             'STAND      ',        'WALK       ',        'STEP       ',        'MANIPULATE ', \
                             'USER       ',        'CALIBRATE  ']


        self.list_box = QListWidget(None)
        self.list_box.addItems(self.allowed_modes)

        self.list_box.currentItemChanged.connect(self.handle_selection_change)
        self.vbox.addWidget(self.list_box)

        self.selection_label = QLabel("Flor Selected: "+self.allowed_modes[0]+"("+str(self.control_mode)+")")
        self.vbox.addWidget(self.selection_label)

        self.label          = QLabel("Flor Command : "+self.allowed_modes[0]+"("+str(self.control_mode)+")")
        self.vbox.addWidget(self.label)

        self.flor_mode         = 0
        self.bdi_current_mode  = 0
        self.bdi_desired_mode  = 0
        self.flor_mode_label = QLabel("Flor Mode    : "+self.allowed_modes[self.flor_mode]+"("+str(self.flor_mode)+")"+"  BDI:("+str(self.bdi_current_mode)+", "+str(self.bdi_desired_mode)+")")
        self.vbox.addWidget(self.flor_mode_label)

        #Add combo box for available settings
        self.available_modes = QComboBox();
        self.available_modes.addItem("");
        self.available_modes.addItem("BDI");
        self.available_modes.addItem("Enable Upper Body");
        self.available_modes.addItem("Enable Whole Body");

        self.available_modes.currentIndexChanged.connect(self.handle_avail_modes_changed)
        self.vbox.addWidget(self.available_modes);
        
        self.vbox.addStretch(1)

        #Add Button for sending the behavior mode command
        self.push_button = QPushButton("Set Mode")
        self.push_button.clicked.connect(self.handle_set_mode)
        self.vbox.addWidget(self.push_button)

        self.vbox.addStretch(1)


        hbox = QHBoxLayout()
        hbox.addStretch(1)
        self.stop_enable= QCheckBox()
        self.stop_enable.setChecked(False)
        hbox.addWidget(self.stop_enable)
        self.stop_enable.clicked.connect(self.handle_stop_enable)


        self.stop_button = QPushButton("STOP!")
        self.stop_button.clicked.connect(self.handle_stop)
        self.stop_button.setStyleSheet('QPushButton {background-color: gray }')
        hbox.addWidget(self.stop_button)
        hbox.addStretch(1)
        self.vbox.addLayout(hbox)
        self._widget.setLayout(self.vbox)

        #add stretch at end so all GUI elements are at top of dialog
        self.vbox.addStretch(1)
        
        self.flor_mode_cmd_sub    = rospy.Subscriber("/flor/controller/mode",              FlorControlMode,          self.florModeCallback)

    def shutdown_plugin(self):
        print "Shutting down ..."
        self.flor_mode_cmd_sub.unregister()
        self.mode_pub.unregister()
        print "Done!"

    # Update BDI state
    def simStateCallback(self, state):
        if ((self.bdi_current_state != state.current_behavior) or (self.bdi_desired_state != state.desired_behavior) or (self.bdi_error_code != state.error_code) or (self.bdi_behavior_status != state.behavior_feedback.status_flags) ):
            self.bdi_current_state   = state.current_behavior
            self.bdi_desired_state   = state.desired_behavior
            self.bdi_error_code      = state.error_code
            self.bdi_behavior_status = state.behavior_feedback.status_flags
            if ((self.bdi_current_state < 0) or (self.bdi_current_state >= length(self.bdi_mode_names))):
                self.bdi_state_label.setText(  " BDI State : "+self.bdi_mode_names[0]+"("+str(self.bdi_current_state)+", " + str(self.bdi_desired_state) + ") EC:("+str(self.bdi_error_code)+", " + str(self.bdi_behavior_status) +")")
            else:
                self.bdi_state_label.setText(  " BDI State : "+self.bdi_mode_names[self.bdi_current_state]+"("+str(self.bdi_current_state)+", " + str(self.bdi_desired_state) + ") EC:("+str(self.bdi_error_code)+", " + str(self.bdi_behavior_status) +")")

    def simCommandCallback(self, bdi_cmd):
        if (self.bdi_behavior != bdi_cmd.behavior) :
            self.bdi_behavior  = bdi_cmd.behavior
            self.bdi_command_label.setText(" BDI Cmd   : "+self.bdi_mode_names[self.bdi_behavior]+"("+str(self.bdi_behavior)+")")

    def florModeCallback(self, cmd):
        if ( (self.flor_mode != cmd.control_mode) or (self.bdi_current_mode != cmd.bdi_current_behavior)  or (self.bdi_desired_mode != cmd.bdi_desired_behavior) ):
            
            if (self.flor_mode != cmd.control_mode) and (self.control_mode != cmd.control_mode):
                print "Flor mode changed externally - clear selection"
                self.clearSelections()
                self.selection_label.setText("Flor Selected  : ")
                self.label.setText("Flor Command : ")
                
            self.flor_mode          = cmd.control_mode
            self.bdi_current_mode   = cmd.bdi_current_behavior
            self.bdi_desired_mode   = cmd.bdi_desired_behavior
            print "Flor mode: ", self.flor_mode, "  BDI: ",self.bdi_current_mode,", ",self.bdi_desired_mode
            if (self.flor_mode > 250):
                print "Invalid control mode "
                self.flor_mode = 0
            #print "Allowed modes:"
            #print self.allowed_modes

            self.flor_mode_label.setText(" Flor Mode   : "+self.allowed_modes[self.flor_mode]+"("+str(self.flor_mode)+") BDI:("+str(self.bdi_current_mode)+","+str(self.bdi_desired_mode)+")")

    def clearSelections(self):
        for i in range(self.list_box.count()):
            item = self.list_box.item(i)
            self.list_box.setItemSelected(item, False)
        self.list_box.setCurrentRow(-1)
                    
    #Slot for selecting
    def handle_selection_change(self, curr, prev):
        if (curr != None):
            self.control_mode = self.mode_ids[curr.text()]
            self.selection_label.setText("Flor Selected  : "+curr.text()+"("+str(self.control_mode)+")")
        #else:
        #    print "NULL selection"
            
        #self.label.setText(self.allowed_modes[selected])
        #self.spindle_speed_pub.publish(data=math.radians(degree_per_sec))
    @Slot(bool)
    def handle_set_mode(self):
        print "Selected=",self.allowed_modes[self.control_mode], ' id=', self.control_mode
        self.label.setText("Flor Command : "+self.allowed_modes[self.control_mode]+"("+str(self.control_mode)+")")
        mode_msg = VigirControlModeCommand()
        mode_msg.header.stamp = rospy.Time.now()
        mode_msg.requested_control_mode  = self.control_mode
        print mode_msg
        self.mode_pub.publish(mode_msg)

    @Slot(bool)
    def handle_stop(self):
        # FLOR_STOP command
        if (self.stop_enable.isChecked()):
            self.clearSelections()
            self.selection_label.setText("Flor Selected  : ")
            self.control_mode = self.mode_ids['stop']
            print "Selected=",self.allowed_modes[self.control_mode], ' id=', self.control_mode
            #self.list_box
            self.selection_label.setText("Flor Selected  : "+self.allowed_modes[self.control_mode]+"("+str(self.control_mode)+")")
            self.label.setText("Flor Command : "+self.allowed_modes[self.control_mode]+"("+str(self.control_mode)+")")
            mode_msg = VigirControlModeCommand()
            mode_msg.header.stamp = rospy.Time.now()
            mode_msg.requested_control_mode     =  self.control_mode
            print mode_msg
            self.mode_pub.publish(mode_msg)
            
        else:
            print "Stop disabled!"

    @Slot(bool)
    def handle_stop_enable(self):
        if (self.stop_enable.isChecked()):
            self.stop_button.setStyleSheet('QPushButton {background-color: red }')
            #self.stop_enable.setChecked(False)
        else:
            self.stop_button.setStyleSheet('QPushButton {background-color: gray }')
            #self.stop_enable.setChecked(True)


    @Slot(bool)
    def handle_avail_modes_changed(self, index):
        print 'no longer mapping allowable modes'
        return
        #for i in range(self.list_box.count()):
        #    item = self.list_box.item(i)
        #    item.setFlags( item.flags() | Qt.ItemIsEnabled )
        #    self.list_box.setItemSelected(item, False)
        #self.list_box.setCurrentRow(-1)

        #if (index == 1):  #bdi - no flor actions
        #
        #    self.control_mode = 6
        #    print "Selected=",self.mode_labels[self.control_mode], ' id=', self.control_mode
        #    #self.list_box
        #    self.selection_label.setText("Flor Selected  : "+self.mode_labels[self.control_mode]+"("+str(self.control_mode)+")")
        #
        #    for item in self.list_box.findItems( "flor_", Qt.MatchContains ):
        #        item.setFlags( item.flags() & ~Qt.ItemIsEnabled )
        ##mani
        #elif (index == 2):  #enable upper body - no stand, step, walk, dance, or wbc
        #
        #    self.control_mode = 6
        #    print "Selected=",self.mode_labels[self.control_mode], ' id=', self.control_mode
        #    #self.list_box
        #    self.selection_label.setText("Flor Selected  : "+self.mode_labels[self.control_mode]+"("+str(self.control_mode)+")")
        #
        #    for item in self.list_box.findItems( "stand", Qt.MatchStartsWith ):
        #        item.setFlags( item.flags() & ~Qt.ItemIsEnabled )
        #
        #    for item in self.list_box.findItems( "step", Qt.MatchStartsWith ):
        #        item.setFlags( item.flags() & ~Qt.ItemIsEnabled )
        #
        #    for item in self.list_box.findItems( "walk", Qt.MatchStartsWith ):
        #        item.setFlags( item.flags() & ~Qt.ItemIsEnabled )
        #
        #    for item in self.list_box.findItems( "dance", Qt.MatchContains ):
        #        item.setFlags( item.flags() & ~Qt.ItemIsEnabled )
        #
        #    for item in self.list_box.findItems( "wbc", Qt.MatchContains ):
        #        item.setFlags( item.flags() & ~Qt.ItemIsEnabled )
        ##wbc
        #elif (index == 3):  #enable whole body - do stand flor_stand flor_dance flor_wbc
        #
        #    self.control_mode = 100
        #    print "Selected=",self.mode_labels[self.control_mode], ' id=', self.control_mode
        #    #self.list_box
        #    self.selection_label.setText("Flor Selected  : "+self.mode_labels[self.control_mode]+"("+str(self.control_mode)+")")
        #
        #    for item in self.list_box.findItems( "", Qt.MatchContains ):
        #        item.setFlags( item.flags() & ~Qt.ItemIsEnabled )
        #
        #    for item in self.list_box.findItems( "stand", Qt.MatchContains ):
        #        item.setFlags( item.flags() | Qt.ItemIsEnabled )
        #
        #    for item in self.list_box.findItems( "dance", Qt.MatchContains ):
        #        item.setFlags( item.flags() | Qt.ItemIsEnabled )
        #
        #    for item in self.list_box.findItems( "wbc", Qt.MatchContains ):
        #        item.setFlags( item.flags() | Qt.ItemIsEnabled )
             

