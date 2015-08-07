import roslib
roslib.load_manifest('vigir_rqt_no_limit_joint_control')
from time import strftime, localtime
import rospy
import sys, copy
import tf
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Signal, Slot, Qt
from python_qt_binding.QtGui import QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QFrame, QSlider, QProgressBar, QSpacerItem, QRadioButton, QButtonGroup #QDoubleSpinBox, QFileDialog

from urdf_parser_py.urdf import URDF

#from atlas_msgs.msg  import AtlasCommand#, AtlasState
#from osrf_msgs.msg   import JointCommands
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math

class Joint(object):
    def __init__(self, name, controller):
        self.name             = name
        self.position         = 0.0
        self.current_position = 0.0
        self.oldPose          = 0.0
        self.velocity         = 0.0
        self.effort           = 0.0
        self.controller       = controller
        self.slider           = None
        self.progress_bar     = None

    def on_sliderMoved(self, value):
        self.position = value/10000.0
        #print (float(valuefile)/10000.0)
        self.controller.value_changed = True
        if (self.controller.parent.parent.radioGroup.checkedId() == 0):
            self.controller.publishGhostJointState()
        else:
            self.controller.publishRobotJointState()

class Controller(object):
    def __init__(self, parent, label, name):
        self.label    = label
        self.name     = name
        self.parent   = parent
        self.joints   = []
        self.oldPoses = False
        self.saveFile = ""
        #self.ghost_joints = []
        self.pub_robot    = rospy.Publisher('/flor/'+self.name+'_controller/joint_states',JointState, queue_size=10)
        self.pub_ghost    = rospy.Publisher('/flor/ghost/set_joint_states',JointState, queue_size=10)

    def publishRobotJointState(self):
        print "publishing new joint states for "+self.name+" to Atlas"
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        for joint in self.joints:
            joint_state.name.append(joint.name)
            joint_state.position.append(joint.position)
            joint_state.velocity.append(joint.velocity)
            joint_state.effort.append(joint.effort)

        print joint_state
        self.pub_robot.publish(joint_state)

    def publishGhostJointState(self):
        #print "publishing new joint states for "+self.name+" to the ghost robot"
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        for joint in self.joints:
            joint_state.name.append(joint.name)
            joint_state.position.append(joint.position)
            joint_state.velocity.append(joint.velocity)
            joint_state.effort.append(joint.effort)

        #print joint_state
        self.pub_ghost.publish(joint_state)

    def on_snapGhostPressed(self):
        print "Snap ",self.name," values to current ghost joint positions"
        self.blockSignals(True)
        self.resetGhostJointSliders()
        self.blockSignals(False)

    def on_snapCurrentPressed(self):
        print "Snap ",self.name," values to current joint positions"
        self.blockSignals(True)
        self.resetCurrentJointSliders()
        self.blockSignals(False)

    def on_applyRobotPressed(self):
        print "Send latest joint values directly to robot", self.name
        self.oldPoses = True
        for i,joint in enumerate(self.joints):
            joint.oldPose = joint.position
        self.publishRobotJointState()

    def on_saveJointsPressed(self):
        print "Saving latest joint values to file"
        if self.saveFile == "":
            self.createSaveFile()
        print "Appending save file"
        file = open(self.saveFile,'a')
        writeBuff = strftime('%H:%M:%S',localtime()) + ',    '
        for i,joint in enumerate(self.joints):
            writeBuff+=str(joint.position) + ',  '
        file.write(writeBuff[0:(len(writeBuff)-3)] + '\n')
        file.close()

    def on_undoPressed(self):
        print "Undo requested on ",self.name
        if self.oldPoses:
            for i,joint in enumerate(self.joints):
                joint.position = joint.oldPose
                joint.slider.setValue(int(joint.oldPose*10000.0))
        else:
            print "No old action to undo!"

    def createSaveFile(self):
        print "No old file found creating new save file."
        self.saveFile  = (roslib.packages.get_pkg_dir('vigir_rqt_position_mode') + '/launch/' + self.name + '_' + strftime('%m-%d_%H:%M:%S',localtime()) + '.txt')
        print "File location  = ",self.saveFile
        file = open(self.saveFile,'w')
        #self.saveFile = temp.name;
        line2 ='indicies, '
        line1 = self.name + ', '
        for i,joint in enumerate(self.joints):
            line1+= ',' + joint.name
            if joint.name =='back_bkz':
                line2+='1, '
            if joint.name =='back_bky':
                line2+='2, '
            elif joint.name =='back_bkx':
                line2+='3, '
            elif joint.name =='l_leg_hpz':
                line2+='4, '
            elif joint.name =='l_leg_hpx':
                line2+='5, '
            elif joint.name =='l_leg_hpy':
                line2+='6, '
            elif joint.name =='l_leg_kny':
                line2+='7, '
            elif joint.name =='l_leg_aky':
                line2+='8, '
            elif joint.name =='l_leg_akx':
                line2+='9, '
            elif joint.name =='r_leg_hpz':
                line2+='10, '
            elif joint.name =='r_leg_hpx':
                line2+='11, '
            elif joint.name =='r_leg_hpy':
                line2+='12, '
            elif joint.name =='r_leg_kny':
                line2+='13, '
            elif joint.name =='r_leg_aky':
                line2+='14, '
            elif joint.name =='r_leg_akx':
                line2+='15, '
            elif joint.name =='l_arm_shz':
                line2+='16, '
            elif joint.name =='l_arm_shx':
                line2+='17, '
            elif joint.name =='l_arm_ely':
                line2+='18, '
            elif joint.name =='l_arm_elx':
                line2+='19, '
            elif joint.name =='l_arm_wry':
                line2+='20, '
            elif joint.name =='l_arm_wrx':
                line2+='21, '
            elif joint.name =='r_arm_shz':
                line2+='22, '
            elif joint.name =='r_arm_shx':
                line2+='23, '
            elif joint.name =='r_arm_ely':
                line2+='24, '
            elif joint.name =='r_arm_elx':
                line2+='25, '
            elif joint.name =='r_arm_wry':
                line2+='26, '
            elif joint.name =='r_arm_wrx':
                line2+=',27 '
        file.write(line1[0:7] + line1[8:]+'\n')
        file.write(line2[0:len(line2)-2] +'\n')
        file.close()

    def blockSignals(self, block):
        for joint in self.joints:
            joint.slider.blockSignals(block)

    def resetCurrentJointSliders(self):
        for joint in self.joints:
            print "Setting joint ",joint.name," = ",str(joint.current_position)
            joint.slider.setValue(int(joint.current_position*10000.0))

    def resetGhostJointSliders(self):
        for i,joint in enumerate(self.joints):
            for i, ghost_joint_name in enumerate(self.parent.parent.ghost_joint_states.name):
                # if joint is there
                if ghost_joint_name == joint.name:
                    print "Setting ghost joint ",ghost_joint_name," = ",str(self.parent.parent.ghost_joint_states.position[i])
                    joint.slider.setValue(int(self.parent.parent.ghost_joint_states.position[i]*10000.0))
                    joint.position = self.parent.parent.ghost_joint_states.position[i]

    def __del__(self):
        print "Shutdown controller ",self.name
        self.pub_robot.unregister();
        self.pub_ghost.unregister();

class NoLimitJointControl(object):
    def __init__(self, parent, fileName, widget):

        self.controllers = []
        self.parent      = parent

        self.loadFile(fileName)

        robot = URDF.from_parameter_server()
        joint_list = {}
	for ndx,jnt in enumerate(robot.joints):
		joint_list[jnt.name] = ndx

        for controller in self.controllers:
            frame = QFrame()
            frame.setFrameShape(QFrame.StyledPanel);
            frame.setFrameShadow(QFrame.Raised);

            vbox = QVBoxLayout()
            label = QLabel()
            label.setText(controller.label)
            vbox.addWidget(label);
            print controller.name

            controller.snap_to_ghost_button = QPushButton("SnapGhost")
            controller.snap_to_ghost_button.pressed.connect(controller.on_snapGhostPressed)
            vbox.addWidget(controller.snap_to_ghost_button)
            controller.snap_to_current_button = QPushButton("SnapCurrent")
            controller.snap_to_current_button.pressed.connect(controller.on_snapCurrentPressed)
            vbox.addWidget(controller.snap_to_current_button)
            controller.apply_to_robot_button = QPushButton("ApplyRobot")
            controller.apply_to_robot_button.pressed.connect(controller.on_applyRobotPressed)
            vbox.addWidget(controller.apply_to_robot_button)
            controller.save_joints_to_file_button = QPushButton("SaveJoints")
            controller.save_joints_to_file_button.pressed.connect(controller.on_saveJointsPressed)
            vbox.addWidget(controller.save_joints_to_file_button)
            controller.undo_last_action_button = QPushButton("Undo Last")
            controller.undo_last_action_button.pressed.connect(controller.on_undoPressed)
            vbox.addWidget(controller.undo_last_action_button)


            for joint in controller.joints:
                label = QLabel()
                label.setText(joint.name)
                vbox.addWidget(label);

                robot_joint = robot.joints[joint_list[joint.name]]
                lower = robot_joint.limit.lower - (math.fabs(robot_joint.limit.upper)+math.fabs(robot_joint.limit.lower))*0.2
                upper = robot_joint.limit.upper + (math.fabs(robot_joint.limit.upper)+math.fabs(robot_joint.limit.lower))*0.2
                print "  ",joint.name, "  limits(", robot_joint.limit.lower,", ",robot_joint.limit.upper,") num"

                joint.slider = QSlider(Qt.Horizontal)
                joint.slider.setRange(int(lower*10000.0), int(upper*10000.0))
                joint.slider.setValue(int(lower*10000.0))
                joint.slider.setSingleStep((upper-lower)/20.0)
                joint.slider.valueChanged.connect(joint.on_sliderMoved)
                vbox.addWidget(joint.slider)
                joint.progress_bar = QProgressBar()
                joint.progress_bar.setRange(int(lower*10000.0), int(upper*10000.0))
                joint.progress_bar.setValue(int(lower*10000.0))
                vbox.addWidget(joint.progress_bar)

            vbox.addStretch()

            frame.setLayout(vbox)
            widget.addWidget(frame)

    def loadFile(self,fileName):
        if fileName[0]:
            print "Loading ",fileName
            loadFileptr = open(fileName, 'r')
            while True:
                line = loadFileptr.readline();
                if line != "":
                    line = line.strip()
                    splitLine = line.split(',')
                    if (splitLine[0][0] != '#'):
                        self.controllers.append(Controller(self,splitLine[0],splitLine[1]))
                        for joint_name in splitLine[2:]:
                            self.controllers[-1].joints.append(Joint(joint_name,self.controllers[-1]))
                            #self.controllers[-1].ghost_joints.append(Joint(joint_name,self.controllers[-1]))
                else:
                    break
                #print "Loaded ", i, " lines"
        else:
            print "Load cancelled."
            sys.exit(-1)

    def updateJointPositions(self,joint_states,first_time):
        if first_time:
            # initialize wbc command vector
            self.joint_commands = joint_states

        for i in range(len(joint_states.name)):
            # need to find the joint in our structures
            for controller in self.controllers:
                for joint in controller.joints:
                    # if joint is there
                    if joint.name == joint_states.name[i]:
                        joint.current_position = joint_states.position[i]
                        #joint.velocity         = joint_states.velocity[i]
                        #joint.effort           = joint_states.effort[i]
                        joint.progress_bar.setValue(int(joint.current_position*10000.0))
                        joint.progress_bar.setFormat("%.4f" % joint.position);

    def blockSignals(self, block):
        for controller in self.controllers:
            controller.blockSignals(block)

    def resetCurrentJointSliders(self):
        for controller in self.controllers:
            controller.blockSignals(True)
            controller.resetCurrentJointSliders()
            controller.blockSignals(False)

    def resetGhostJointSliders(self):
        for controller in self.controllers:
            controller.resetGhostJointSliders()


class NoLimitJointControlDialog(Plugin):
    updateStateSignal = Signal(object)
    updateGhostSignal = Signal(object)

    def __init__(self, context):
        super(NoLimitJointControlDialog, self).__init__(context)
        self.setObjectName('NoLimitJointControlDialog')
        self.updateStateSignal.connect(self.on_updateState)
        self.updateGhostSignal.connect(self.on_updateGhost)

        self.joint_states        = JointState()
        self.ghost_joint_states  = JointState()
        self._widget = QWidget()
        vbox = QVBoxLayout()

        # Define checkboxes
        radios = QWidget();
        hbox_radio = QHBoxLayout()
        self.radioGroup = QButtonGroup()
        self.radioGroup.setExclusive(True)
        self.radio_ghost_target = QRadioButton()
        self.radio_ghost_target.setText("Ghost")
        self.radioGroup.addButton(self.radio_ghost_target,0)
        self.radio_ghost_target.setChecked(True)
        self.radio_robot_target = QRadioButton()
        self.radio_robot_target.setText("Robot")
        self.radioGroup.addButton(self.radio_robot_target,1)
        hbox_radio.addStretch()
        hbox_radio.addWidget(self.radio_ghost_target)
        #hbox_radio.addWidget(QLabel("Ghost"))
        hbox_radio.addStretch()
        hbox_radio.addWidget(self.radio_robot_target)
        #hbox_radio.addWidget(QLabel("Robot"))
        hbox_radio.addStretch()
        radios.setLayout(hbox_radio)

        vbox.addWidget(radios)

        widget = QWidget()
        hbox = QHBoxLayout()

        # Left to right layout
        self.joint_control = NoLimitJointControl(self, roslib.packages.get_pkg_dir('vigir_rqt_no_limit_joint_control') + '/launch/joints.txt',hbox)

        widget.setLayout(hbox)

        vbox.addWidget(widget)

        print "Add buttons to apply all ..."
        all_widget = QWidget()
        all_box = QHBoxLayout()

        self.snap_to_ghost_button = QPushButton("SnapAllGhost")
        self.snap_to_ghost_button.pressed.connect(self.on_snapGhostPressed)
        all_box.addWidget(self.snap_to_ghost_button)
        self.snap_to_current_button = QPushButton("SnapAllCurrent")
        self.snap_to_current_button.pressed.connect(self.on_snapCurrentPressed)
        all_box.addWidget(self.snap_to_current_button)
        self.apply_to_robot_button = QPushButton("ApplyAllRobot")
        self.apply_to_robot_button.pressed.connect(self.on_applyRobotPressed)
        all_box.addWidget(self.apply_to_robot_button)
        self.apply_to_robot_button = QPushButton("Apply WBC Robot")
        self.apply_to_robot_button.pressed.connect(self.on_applyWBCRobotPressed)
        all_box.addWidget(self.apply_to_robot_button)

        all_widget.setLayout(all_box)

        vbox.addWidget(all_widget)

#        all_hbox = QHBoxLayout()
#        all_hbox.addStretch()
#        all_hbox.addWidget(all_widget)
#        all_hbox.addStretch()
#        bottom_widget=QWidget()
#        bottom_widget.setLayout(all_jbox)
#        vbox.addWidget(bottom_widget)

        vbox.addStretch()

        self._widget.setLayout(vbox)
        context.add_widget(self._widget)

        self.first_time = True

        self.stateSubscriber  = rospy.Subscriber('/atlas/joint_states', JointState, self.stateCallbackFnc)
        self.ghostSubscriber  = rospy.Subscriber('/flor/ghost/get_joint_states', JointState, self.ghostCallbackFnc)
        self.wbc_robot_pub    = rospy.Publisher('/flor/wbc_controller/joint_states',JointState, queue_size=10)





    def shutdown_plugin(self):
    #Just make sure to stop timers and publishers, unsubscribe from Topics etc in the shutdown_plugin method.
        print "Shutdown ..."
        #rospy.sleep(0.1)
        self.stateSubscriber.unregister()
        self.ghostSubscriber.unregister()
        self.wbc_robot_pub.unregister()
        print "Done!"

    def stateCallbackFnc(self, atlasState_msg):
        self.updateStateSignal.emit(atlasState_msg)

    def ghostCallbackFnc(self, ghost_joint_state_msg):
        self.updateGhostSignal.emit(ghost_joint_state_msg)

    def on_snapGhostPressed(self):
        print "Snap all joint values to current ghost joint positions"
        for controller in self.joint_control.controllers:
            controller.on_snapGhostPressed()

    def on_snapCurrentPressed(self):
        print "Snap all joint values to current joint positions"
        for controller in self.joint_control.controllers:
            controller.on_snapCurrentPressed()

    def on_applyRobotPressed(self):
        print "Send all latest joint values directly to robot"
        for controller in self.joint_control.controllers:
            controller.on_applyRobotPressed()

    def on_applyWBCRobotPressed(self):
        print "Send all latest joint values directly to robot as a WBC command"
        if self.first_time:
            print "Uninitialized !"
        else:
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name     =  copy.deepcopy(self.joint_states.name)
            joint_state.position =  list(copy.deepcopy(self.joint_states.position))
            joint_state.velocity =  list(copy.deepcopy(self.joint_states.velocity))

            #print "Positions: ",joint_state.position
            for i,name in enumerate(joint_state.name):
                # for each joint, retrieve the data from each controller to populate the WBC vectors
                for controller in self.joint_control.controllers:
                    for joint in controller.joints:
                        if joint.name == self.joint_states.name[i]:
                            print "     fetching value for "+name+" = "+str(joint.position)+"  at ndx="+str(i)
                            joint_state.position[i] = joint.position
                            joint_state.velocity[i] = joint.velocity
                            #joint_state.effort[i]   = joint.effort

            self.wbc_robot_pub.publish(joint_state)


    # this runs in the Qt GUI thread and can therefore update the GUI
    def on_updateState(self, atlasState_msg):
        self.joint_states = atlasState_msg
        self.joint_control.updateJointPositions(self.joint_states,self.first_time)
        if self.first_time:
            self.joint_control.resetCurrentJointSliders()
            self.first_time = False
            self.ghost_joint_states = copy.deepcopy(atlasState_msg);
            self.ghost_joint_states.position = list(self.ghost_joint_states.position)
            #print "Initial Ghost Stored:",self.ghost_joint_states

    # this runs in the Qt GUI thread and can therefore update the GUI
    def on_updateGhost(self, ghost_joint_state_msg):
        for i, new_joint_name in enumerate(list(ghost_joint_state_msg.name)):
            name_found = False
            for j,stored_joint_name in enumerate(self.ghost_joint_states.name):
                if (new_joint_name == stored_joint_name):
                    name_found = True
                    #print "ghost_joint_msg[",str(i),"]=",ghost_joint_state_msg.position[i]
                    #print "ghost_joint_states[",str(j),"]=",self.ghost_joint_states.position[i]
                    self.ghost_joint_states.position[j] = ghost_joint_state_msg.position[i]
                    #self.ghost_joint_states.velocity[j] = ghost_joint_state_msg.velocity[i]
            if (not name_found):
                # Update the list without regard to order
                self.ghost_joint_states.name.append(new_joint_name)
                self.ghost_joint_states.position.append(ghost_joint_state_msg.position[i])
                #self.ghost_joint_states.velocity.append(ghost_joint_state_msg.velocity[i])
        #print "Msg:",ghost_joint_state_msg
        #print "Stored:",self.ghost_joint_states


