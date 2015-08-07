import roslib
roslib.load_manifest('vigir_rqt_position_mode')

import rospy
import sys
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Signal, Slot, Qt, QObject
from python_qt_binding.QtGui import QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QFrame, QRadioButton, QButtonGroup #QDoubleSpinBox, QFileDialog

#from urdf_parser_py.urdf import URDF

#from atlas_msgs.msg  import AtlasCommand#, AtlasState
#from osrf_msgs.msg   import JointCommands
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointData(object):

    def __init__(self, parent, name, index, osrf):
        #self.parent   = parent
        self.name     = name
        self.index    = index
        self.osrf_ndx = osrf

    #def on_cmd_value(self, value):
    #    #print self.name,":",self.osrf_ndx," - on_cmd_value"
    #    self.main.robot_command.position[self.osrf_ndx] = value
    #    print "position[",self.osrf_ndx,"]=",self.main.robot_command.position[self.osrf_ndx]
    #    #print "position[]=",self.main.robot_command.position

class PoseData(object):

    def __init__(self, parent, positions, vbox):
        self.parent        = parent
        self.name          = positions[0]

        self.target_positions  = []
        for i,val in enumerate(positions):
            if (i>0):
                if (val != ""):
                    self.target_positions.append(float(val))

        title = positions[0]#parent.chain_name+":"+positions[0]

        #print "#"+title,":",self.target_positions

        apply_command = QPushButton(title)
        apply_command.clicked.connect(self.apply_command_callback)
        vbox.addWidget(apply_command)

    def apply_command_callback(self):

        print self.name,":",self.target_positions

        print " radioGroup checked is ",self.parent.main.radioGroup.checkedId()
        print " commandGroup checked is ",self.parent.main.commandGroup.checkedId()
        
        if (self.parent.main.radioGroup.checkedId() == 1):
            if (self.parent.main.commandGroup.checkedId() == 1):
                trajectory = JointTrajectory()
                trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.125) # in future
                for i,joint in enumerate(self.parent.chain):
                    trajectory.joint_names.append(joint.name)

                trajectory.points       = [JointTrajectoryPoint(),JointTrajectoryPoint()]
                trajectory.points[0].positions = len(self.target_positions)*[0.0]
                trajectory.points[1].positions = self.target_positions
                trajectory.points[0].velocities = len(self.target_positions)*[0.0]
                trajectory.points[1].velocities = len(self.target_positions)*[0.0]
                trajectory.points[0].time_from_start = rospy.Duration(0.0)
                trajectory.points[1].time_from_start = rospy.Duration(self.parent.main.time_factor)

                valid_trajectory = True
                for i,joint in enumerate(self.parent.chain):
                    osrf = joint.osrf_ndx
                    if (osrf < len(self.parent.main.joint_states.position)):
                        trajectory.points[0].positions[i] = self.parent.main.joint_states.position[osrf]
                        print "assign p=",trajectory.points[0].positions[i]," to (",i,") from osrf",osrf
                    else:
                        if (osrf >29) and (osrf < 36):
                            trajectory.points[0].positions[i] = self.parent.main.pelvis_states.position[osrf-30]
                            print "assign pelvis =",trajectory.points[0].positions[i]," to (",i,") from osrf",osrf
                        else:
                            print "missing joint value for osrf ndx = ",osrf
                            valid_trajectory = False

                if (valid_trajectory):
                    print "Publishing trajectory at ",trajectory.header.stamp," for ",self.name, " ", self.parent.chain_name
                    self.parent.trajectoryPublisher.publish(trajectory)
                else:
                    print "Ignore invalid trajectory!"
                print trajectory
            else:
                positions = JointState()
                positions.header.stamp = rospy.Time.now()
                positions.position = self.target_positions
                positions.velocity = len(self.target_positions)*[0.0]

                print "Publishing positions at ",positions.header.stamp," for ",self.name, " ", self.parent.chain_name
                self.parent.positionPublisher.publish(positions)
                print positions


            # Re-set to ghost so we don't accidently send anything to the real robot
            self.parent.main.radio_ghost_target.setChecked(True)

        else:
            positions = JointState()
            positions.header.stamp = rospy.Time.now()
            positions.position = self.target_positions
            positions.velocity = len(self.target_positions)*[0.0]

            if (self.parent.chain_name == "pelvis"):
                print "Cannot send pelvis command to ghost - not behavior!"
            else:
                # Must load names for the joint state vector
                for i,joint in enumerate(self.parent.chain):
                    positions.name.append(joint.name)

                print "Publishing positions at ",positions.header.stamp," for Ghost ",self.name, " ", self.parent.chain_name
                self.parent.ghostPublisher.publish(positions)
                print positions



class ChainData(object):
    def __init__(self, main, fileName, main_hbox):
        self.main       = main
        self.chain      = []
        self.poses      = []
        self.chain_name  = ""
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel);
        frame.setFrameShadow(QFrame.Raised);

        vbox = QVBoxLayout()

        label = QLabel()
        vbox.addWidget(label);

        self.load_file(fileName, vbox)
        self.trajectoryPublisher = rospy.Publisher('/trajectory_controllers/'+self.chain_name+'_traj_controller/trajectory'  ,JointTrajectory, queue_size=10)
        self.positionPublisher   = rospy.Publisher('/trajectory_controllers/'+self.chain_name+'_traj_controller/joint_states',JointState,      queue_size=10)
        self.ghostPublisher      = rospy.Publisher('/flor/ghost/set_joint_states',JointState, queue_size=10)

        label.setText(self.chain_name)
        vbox.addStretch(1)

        frame.setLayout(vbox)
        main_hbox.addWidget(frame)

    def load_file(self,fileName, vbox):

        if fileName[0]:
            print "Loading ",fileName
            loadFileptr = open(fileName, 'r')
            i = 0
            k = 0
            chain_joints= []
            chain_index = []
            while True:
                line = loadFileptr.readline();
                if line != "":# or line != '\n':
                    line = line.strip()
                    splitLine = line.split(',')
                    if (i == 0):
                        self.chain_name = splitLine[0]
                        k = 1
                        while k < len(splitLine):
                            chain_joints.append(splitLine[k])
                            k += 1
                        #print "Chain=",self.chain_name
                        #print chain_joints
                    elif (i == 1):
                        k = 1
                        while k < len(splitLine):
                            chain_index.append(int(splitLine[k]))
                            k += 1
                        #print "Chain=",self.chain_name
                        #print chain_joints
                        for j,joint in enumerate(chain_joints):
                            joint = joint.strip()
                            osrf_ndx = chain_index[j]
                            if (osrf_ndx < len(self.main.joint_names)):
                                if (joint == self.main.joint_names[ chain_index[j] ] ):
                                    print "joint=",joint, " i=",i," osrf=",chain_index[j]
                                    self.chain.append(JointData(self,joint,i,chain_index[j] ) )
                                else:
                                    if (osrf_ndx < 36):
                                        print "joint=",joint, " i=",i," osrf=",chain_index[j]
                                        self.chain.append(JointData(self,joint,i,chain_index[j] ) )
                                    else:
                                        print "Invalid match of osrf index ", chain_index[j], " name=<",joint,">=?=<",self.main.joint_names[chain_index[j]],">"
                            else:
                                print "Invalid joint index for robot - must be pelvis joint", osrf_ndx, " ", chain_joints[j]
                                self.chain.append(JointData(self,joint,i,osrf_ndx ) )
                    else:
                        if (len(splitLine) > 1):
                            self.poses.append(PoseData(self, splitLine, vbox))
                        #else:
                        #    print "Empty line:",line
                    i+=1
                else:
                    break
                #print "Loaded ", i, " lines"
        else:
            print "Load cancelled."
            sys.exit(-1)

    def __del__(self):
        print "Shutdown class for ", self.chain_name, " ... "
        self.trajectoryPublisher.unregister()
        self.positionPublisher.unregister()
        self.ghostPublisher.unregister()

class PositionModeWidget(QObject):
    updateStateSignal  = Signal(object)
    updatePelvisSignal = Signal(object)
    updateGhostSignal  = Signal(object)


    def __init__(self, context):
        #super(PositionModeDialog, self).__init__(context)
        #self.setObjectName('PositionModeDialog')
        super(PositionModeWidget, self).__init__()
        self.name = 'PositionModeWidget'

        self.updateStateSignal.connect(self.on_updateState)
        self.updatePelvisSignal.connect(self.on_updatePelvis)
        self.updateGhostSignal.connect(self.on_updateGhost)

        ## Process standalone plugin command-line arguments
        #from argparse import ArgumentParser
        #parser = ArgumentParser()
        ## Add argument(s) to the parser.
        #parser.add_argument("-f", "--file",
        #              dest="file",
        #              help="Input file name")
        #args, unknowns = parser.parse_known_args(context.argv())
        #print 'arguments: ', args
        #print 'unknowns: ', unknowns
        #
        #if ((args.file == "") or (args.file == None)):
        #    print "No file specified - abort!"
        #    sys.exit(-1)

        # Define how long the trajectories will take to execute
        self.time_factor  = rospy.get_param('~time_factor', 2.1)
        print "Using ",self.time_factor, " as the time factor for all trajectories"
        
        # Joint Name : Child Link :OSRF
        self.joint_names=[]
        self.joint_names.append('back_bkz')  #  :     ltorso :   0
        self.joint_names.append('back_bky')  #  :     mtorso :   1
        self.joint_names.append('back_bkx')  #  :     utorso :   2
        self.joint_names.append('neck_ry')   #  :       head :   3
        self.joint_names.append('l_leg_hpz') #  :    l_uglut :   4
        self.joint_names.append('l_leg_hpx') #  :    l_lglut :   5
        self.joint_names.append('l_leg_hpy') #  :     l_uleg :   6
        self.joint_names.append('l_leg_kny') #  :     l_lleg :   7
        self.joint_names.append('l_leg_aky') #  :    l_talus :   8
        self.joint_names.append('l_leg_akx') #  :     l_foot :   9
        self.joint_names.append('r_leg_hpz') #  :    r_uglut :  10
        self.joint_names.append('r_leg_hpx') #  :    r_lglut :  11
        self.joint_names.append('r_leg_hpy') #  :     r_uleg :  12
        self.joint_names.append('r_leg_kny') #  :     r_lleg :  13
        self.joint_names.append('r_leg_aky') #  :    r_talus :  14
        self.joint_names.append('r_leg_akx') #  :     r_foot :  15
        self.joint_names.append('l_arm_shz') #  :     l_clav :  16
        self.joint_names.append('l_arm_shx') #  :     l_scap :  17
        self.joint_names.append('l_arm_ely') #  :     l_uarm :  18
        self.joint_names.append('l_arm_elx') #  :     l_larm :  19
        self.joint_names.append('l_arm_wry') #  :     l_farm :  20
        self.joint_names.append('l_arm_wrx') #  :     l_hand :  21
        self.joint_names.append('l_arm_wry2') # :     l_farm :  22
        self.joint_names.append('r_arm_shz') #  :     r_clav :  23
        self.joint_names.append('r_arm_shx') #  :     r_scap :  24
        self.joint_names.append('r_arm_ely') #  :     r_uarm :  25
        self.joint_names.append('r_arm_elx') #  :     r_larm :  26
        self.joint_names.append('r_arm_wry') #  :     r_farm :  27
        self.joint_names.append('r_arm_wrx') #  :     r_hand :  28
        self.joint_names.append('r_arm_wry2') # :     r_farm :  29

        self.joint_states  = JointState()

        self.pelvis_names=[]
        self.joint_names.append('forward')  #  :     30
        self.joint_names.append('lateral')  #  :     31
        self.joint_names.append('height')   #  :     32
        self.joint_names.append('roll')     #  :     33
        self.joint_names.append('pitch')    #  :     34
        self.joint_names.append('yaw')      #  :     35

        self.pelvis_states  = JointState()
        self.pelvis_states.position = [0.0, 0.0, 0.85, 0.0, 0.0, 0.0] # default pelvis pose
        #self._widget = QWidget()
        self._widget = context
        vbox = QVBoxLayout()

        # Define checkboxes
        radios = QWidget();
        hbox_radio = QHBoxLayout()
        self.radioGroup = QButtonGroup()
        self.radioGroup.setExclusive(True)
        self.radio_ghost_target = QRadioButton()
        self.radio_ghost_target.setText("Ghost")
        self.radio_robot_target = QRadioButton()
        self.radio_robot_target.setText("Robot")
        self.radioGroup.addButton(self.radio_ghost_target,0)
        self.radioGroup.addButton(self.radio_robot_target,1)
        self.radio_ghost_target.setChecked(True)

        self.commandGroup = QButtonGroup()
        self.commandGroup.setExclusive(True)
        self.radio_position_command = QRadioButton()
        self.radio_position_command.setText("Position")
        self.commandGroup.addButton(self.radio_position_command,0)
        self.radio_trajectory_command = QRadioButton()
        self.radio_trajectory_command.setText("Trajectory")
        self.commandGroup.addButton(self.radio_trajectory_command,1)
        self.radio_trajectory_command.setChecked(True)

        #print "position   button is checked: ",self.radio_position_command.isChecked()
        #print "trajectory button is checked: ",self.radio_trajectory_command.isChecked()
        print "Default group   button checked is ",self.radioGroup.checkedId()
        print "Default command button checked is ",self.commandGroup.checkedId()
        
        hbox_radio.addStretch()
        hbox_radio.addWidget(self.radio_ghost_target)
        #hbox_radio.addWidget(QLabel("Ghost"))
        hbox_radio.addStretch()

        vbox_robot = QVBoxLayout()
        widget_robot = QWidget()

        widget_robot_sel = QWidget()
        hbox_sel = QHBoxLayout()
        hbox_radio.addStretch()
        hbox_sel.addWidget(self.radio_robot_target)
        #hbox_sel.addWidget(QLabel("Robot"))
        hbox_radio.addStretch()
        widget_robot_sel.setLayout(hbox_sel)
        vbox_robot.addWidget(widget_robot_sel);

        widget_cmd = QWidget()
        hbox_cmd = QHBoxLayout()
        hbox_radio.addStretch()
        hbox_cmd.addWidget(self.radio_position_command)
        #hbox_cmd.addWidget(QLabel("Position"))
        hbox_radio.addStretch()
        hbox_cmd.addWidget(self.radio_trajectory_command)
        #hbox_cmd.addWidget(QLabel("Trajectory"))
        hbox_radio.addStretch()
        widget_cmd.setLayout(hbox_cmd)
        vbox_robot.addWidget(widget_cmd);
        widget_robot.setLayout(vbox_robot)

        hbox_radio.addWidget(widget_robot)
        radios.setLayout(hbox_radio)

        vbox.addWidget(radios)


        positions_widget = QWidget()
        hbox = QHBoxLayout()

        #self.robot = URDF.from_parameter_server()
        self.chains=[]

        # Subscribe to Joint States update /atlas/atlas_state
        #print "Load parameters"
        #print rospy.get_param_names()
        #fileName = rospy.get_param('positions_file', '/opt/vigir/catkin_ws/src/vigir_ocs_eui/vigir_rqt/vigir_rqt_position_mode/launch/r_arm_positions.txt')
        #print "FileName:",fileName


        # Left to right layout
        numFiles = rospy.get_param("~num_files");
        for i in range(1,numFiles+1):
            path = "~position_files/file_" + str(i)
            foobar = str(rospy.get_param(path))
            self.chains.append(ChainData(self, foobar, hbox))

        #add stretch at end so all GUI elements are at left of dialog
        hbox.addStretch(1)

        positions_widget.setLayout(hbox)
        vbox.addWidget(positions_widget)

        vbox.addStretch(1)

        self._widget.setLayout(vbox)

        #context.add_widget(self._widget)

        self.stateSubscriber   = rospy.Subscriber('/atlas/joint_states', JointState, self.stateCallbackFnc)
        self.ghostSubscriber   = rospy.Subscriber('/flor/ghost/get_joint_states', JointState, self.ghostCallbackFnc)
        self.pelvisSubscriber  = rospy.Subscriber('/robot_controllers/pelvis_traj_controller/current_states',JointState, self.pelvisCallbackFnc)

    def shutdown_plugin(self):
    #Just make sure to stop timers and publishers, unsubscribe from Topics etc in the shutdown_plugin method.
    #TODO, is the above comment why the plugin gets stuck when we try to shutdown?
        print "Shutdown"
        rospy.sleep(0.1)
        self.stateSubscriber.unregister()
        self.ghostSubscriber.unregister()
        self.pelvisSubscriber.unregister()

    def stateCallbackFnc(self, jointState_msg):
        self.updateStateSignal.emit(jointState_msg)

    def on_updateState(self, jointState_msg):
        self.joint_states = jointState_msg

    def pelvisCallbackFnc(self, jointState_msg):
        self.updatePelvisSignal.emit(jointState_msg)

    def on_updatePelvis(self, jointState_msg):
        self.pelvis_states = jointState_msg;

    def ghostCallbackFnc(self, jointState_msg):
        self.updateGhostSignal.emit(jointState_msg)

    def on_updateGhost(self, ghost_joint_state_msg):
        self.ghost_joint_states = ghost_joint_state_msg
