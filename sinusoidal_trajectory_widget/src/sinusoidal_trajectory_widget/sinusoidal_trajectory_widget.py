import roslib

import rospy
import sys, copy
import yaml
import time
import numpy as np

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Signal, Slot, Qt
from python_qt_binding.QtGui import QWidget, QPushButton, QCheckBox, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QFrame, QFileDialog, QComboBox, QPalette

from urdf_parser_py.urdf import URDF
import rosparam

from trajectory_msgs.msg    import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg        import JointState

class JointData(object):

    def __init__(self, main, name):
        self.main        = main
        self.name        = name
        self.position    = 0.0
        self.amplitude   = 0.0
        self.frequency   = 0.0
        self.iterations  = 0
        self.lower_limit = -6
        self.upper_limit =  6

    def on_position_value(self, value):
        self.position                             = copy.deepcopy(value)
        print "position[",self.name,"]=",self.position

    def on_amplitude_value(self, value):
        self.amplitude = copy.deepcopy(value)

    def __str__(self):
        return str(self.name) + ' - position: ' + str(self.position) + ' amplitude: ' +  str(self.amplitude) +  ' frequency: ' + str(self.frequency)

class SinusoidalTrajectoryDialog(Plugin):
    updateStateSignal = Signal(object)

    def __init__(self, context):
        super(SinusoidalTrajectoryDialog, self).__init__(context)
        self.setObjectName('SinusoidalTrajectoryDialog')
        #self.updateStateSignal = Signal(object)
        self.updateStateSignal.connect(self.on_updateState)

        self.robot = URDF.from_parameter_server()
        self.joint_list = {}

    	for ndx,jnt in enumerate(self.robot.joints):
            self.joint_list[jnt.name] = ndx
            #print jnt.name, " ",self.joint_list[jnt.name]

        self.chain=[]
        self.chain_file = rospy.get_param("chain_file")
        self.chain_name = rospy.get_param("chain_name")

        yaml_file = self.chain_file+self.chain_name+"_chain.yaml"
        print yaml_file

        #define structures
        self.robot_state = JointState()
        self.robot_command = JointTrajectory()

        stream = open(yaml_file, "r")
        jointChain = yaml.load_all(stream)
        print '\n\n'
        for ndx, data in enumerate(jointChain):
            print ndx," : ", data
            self.delay_time          = data["delay_time"]
            self.amplitude           = data["amplitude"]
            self.frequency           = data["frequency"]
            self.frequency_limit     = data["frequency_limit"]
            self.iterations          = data["iterations"]
            self.joint_state_topic   = data["joint_state_topic"]
            self.trajectory_topic    = data["trajectory_topic"]

            joints = rospy.get_param(data["chain_param_name"])
            for joint in joints:
                print joint
                self.robot_state.name.append(joint)
                self.chain.append(JointData(self, joint) )
        self.robot_command.joint_names = self.robot_state.name

        stream.close()


        self.robot_state.position = [0.0]*len(self.robot_state.name)
        self.robot_state.velocity = [0.0]*len(self.robot_state.name)
        self.robot_state.effort   = [0.0]*len(self.robot_state.name)
        self.robot_joint_state = JointState()

        print "delay_time  =",self.delay_time
        print "amplitude   =",self.amplitude
        print "frequency   =",self.frequency
        print "iterations  =",self.iterations

        print "Robot State Structure",self.robot_state
        print "Robot Command Structure",self.robot_command

        # initialize structure to hold widget handles
        self.cur_position_spinbox=[]
        self.amplitude_spinbox=[]
        self.frequency_spinbox=[]
        self.iterations_spinbox=[]

        self._widget = QWidget()
        vbox = QVBoxLayout()

        # Push buttons
        hbox = QHBoxLayout()

        snap_command = QPushButton("Snap Position")
        snap_command.clicked.connect(self.snap_current_callback)
        hbox.addWidget(snap_command)

        check_limits = QPushButton("Check Limits Gains")
        check_limits.clicked.connect(self.check_limits_callback)
        hbox.addWidget(check_limits)

        apply_command = QPushButton("Send Trajectory")
        apply_command.clicked.connect(self.apply_command_callback)
        hbox.addWidget(apply_command)

        save_trajectory = QPushButton("Save Trajectory")
        save_trajectory.clicked.connect(self.save_trajectory_callback)
        hbox.addWidget(save_trajectory)

        zero_ramp = QPushButton("Zero Values")
        zero_ramp.clicked.connect(self.zero_values_callback)
        hbox.addWidget(zero_ramp)

        vbox.addLayout(hbox)

        time_hbox = QHBoxLayout()

        vbox_frequqency = QVBoxLayout()
        vbox_frequqency.addWidget(QLabel("Frequency"))
        self.frequency_spinbox = QDoubleSpinBox()
        self.frequency_spinbox.setDecimals(5)
        self.frequency_spinbox.setRange(0, self.frequency_limit)
        self.frequency_spinbox.setSingleStep(0.05)
        self.frequency_spinbox.valueChanged.connect(self.on_frequency_value)
        self.frequency_spinbox.setValue(self.frequency)
        vbox_frequqency.addWidget(self.frequency_spinbox)
        time_hbox.addLayout(vbox_frequqency)

        vbox_iterations = QVBoxLayout()
        vbox_iterations.addWidget(QLabel("Iterations"))
        self.iterations_spinbox = QDoubleSpinBox()
        self.iterations_spinbox.setDecimals(5)
        self.iterations_spinbox.setRange(0, 10)
        self.iterations_spinbox.setSingleStep(1)
        self.iterations_spinbox.valueChanged.connect(self.on_iterations_value)
        self.iterations_spinbox.setValue(self.iterations)
        vbox_iterations.addWidget(self.iterations_spinbox)
        time_hbox.addLayout(vbox_iterations)

        vbox.addLayout(time_hbox)

        # Joints title
        title_frame = QFrame()
        title_frame.setFrameShape(QFrame.StyledPanel);
        title_frame.setFrameShadow(QFrame.Raised);

        title_hbox = QHBoxLayout()
        title_hbox.addWidget(QLabel("Joints"))
        title_hbox.addWidget(QLabel("Current Position"))
        title_hbox.addWidget(QLabel("Amplitude"))
        title_frame.setLayout(title_hbox)
        vbox.addWidget(title_frame)


        # Define the widgets for each joint
        for i,joint in enumerate(self.chain):
            #print i,",",joint
            self.joint_widget( vbox, i)


        #add stretch at end so all GUI elements are at top of dialog
        vbox.addStretch(1)

        self._widget.setLayout(vbox)


        # Define the connections to the outside world
        self.jointSubscriber  = rospy.Subscriber(self.joint_state_topic, JointState, self.stateCallbackFnc)
        self.commandPublisher = rospy.Publisher(self.trajectory_topic  , JointTrajectory, queue_size=10)

        # Add the widget
        context.add_widget(self._widget)

    @Slot()
    def snap_current_callback(self):
        self.blockSignals(True)
        print "Snapping positions to actual values"
        for index, joint in enumerate(self.chain):
            for index_state, name_state in enumerate(self.robot_joint_state.name):
                if (name_state == joint.name):
                    joint.position = copy.deepcopy(self.robot_joint_state.position[index_state])
                    self.cur_position_spinbox[index].setValue(joint.position)
                    break

        self.blockSignals(False)

    @Slot()
    def check_limits_callback(self):
        self.blockSignals(True)
        print "Check limits callback ..."
        valid = True
        for index, joint in enumerate(self.chain):

            ramp_up   = joint.position + joint.amplitude
            ramp_down = joint.position - joint.amplitude

            p = self.amplitude_spinbox[index].palette()
            if (ramp_up > joint.upper_limit) or (ramp_up < joint.lower_limit):
                print "Joint ",joint.name, " is beyond upper limit!"
                valid=False
                p.setColor(QPalette.Window, Qt.red)  # <<<<<<<<<<<----------------- This isn't working as intended
            else:
                p.setColor(QPalette.Window, Qt.white)# <<<<<<<<<<<----------------- This isn't working as intended
                #print joint.lower_limit," < ", ramp_up, " < ",joint.upper_limit

            self.amplitude_spinbox[index].setPalette(p)

            if (ramp_down > joint.upper_limit) or (ramp_down < joint.lower_limit):
                print "Joint ",joint.name, " is beyond lower limit!"
                valid=False
                p.setColor(QPalette.Window, Qt.red)  # <<<<<<<<<<<----------------- This isn't working as intended
            else:
                p.setColor(QPalette.Window, Qt.white)# <<<<<<<<<<<----------------- This isn't working as intended
                #print joint.lower_limit," < ", ramp_down, " < ",joint.upper_limit

        if (self.frequency > self.frequency_limit or self.frequency <= 0):
            print "invalid frequency. must be between 0 and ", self.frequency_limit, ". value is: ", self.frequency
            valid = False

        if (not valid):
            print "ERROR: Invalid input!"
        else:
            print "   Valid values!"

        self.blockSignals(False)
        return valid

    @Slot()
    def zero_values_callback(self):
        self.blockSignals(True)
        print "Zero ramps callback ..."
        for index, joint in enumerate(self.chain):
            self.amplitude_spinbox[index].setValue(0.0)
        self.blockSignals(False)

    @Slot()
    def apply_command_callback(self):
        self.blockSignals(True)
        print "Send trajectory"

        if self.calculate_trajectory():
            #print self.robot_command;
            self.commandPublisher.publish(self.robot_command)
        else:
            print "Trajectory calculation failure - do not publish!"

        self.blockSignals(False)

    @Slot()
    def save_trajectory_callback(self):
        self.blockSignals(True)
        print "Save gains"
        # Save data to file that we can read in
        # TODO - invalid file names

        fileName = QFileDialog.getSaveFileName()

        #if fileName[0] checks to ensure that the file dialog has not been canceled
        if fileName[0]:
            if self.calculate_trajectory():
                print self.robot_command;
                newFileptr = open(fileName[0], 'w')
                # Suggested format for files to make it easy to combine different outputs
                newFileptr.write('# Trajectory \n')
                newFileptr.write(self.robot_command)
                newFileptr.write('\n')
                newFileptr.close()
            else:
                print "Trajectory calculation failure - do not save!"
        else:
            print "Save cancelled."

        newFilePtr.close()

        self.blockSignals(False)

    #
    @Slot()
    def stateCallbackFnc(self, atlasState_msg):
        # Handle the state message for actual positions
        time = atlasState_msg.header.stamp.to_sec()
        if ((time - self.prior_time) > 0.02):
            # Only update at 50hz
            # relay the ROS messages through a Qt Signal to switch to the GUI thread
            self.updateStateSignal.emit(atlasState_msg)
            self.prior_time = time

    # this runs in the Qt GUI thread and can therefore update the GUI
    def on_updateState(self, joint_state_msg):
        #print joint_state_msg
        self.robot_joint_state = joint_state_msg

    def on_delay_time_value(self, value):
        self.delay_time = copy.deepcopy(value)

    def on_frequency_value(self, value):
        self.frequency = copy.deepcopy(value)

    def on_iterations_value(self, value):
        self.iterations = copy.deepcopy(value)


    def calculate_trajectory(self):

        if not self.check_limits_callback():
            print "Invalid limits for trajectory!"
            return False

        knot_points = 8*self.iterations
        if (knot_points < 2):
            print "Invalid trajectory - knot_points = ",knot_points
            return False

        #print "Knot points=",knot_points
        self.robot_command.points = []
        self.robot_command.points.append(JointTrajectoryPoint())

        ndx = 0
        self.robot_command.points[ndx].time_from_start = rospy.Duration(0.0)
        for jnt, joint in enumerate(self.chain):
            self.robot_command.points[ndx].positions.append(joint.position)
            self.robot_command.points[ndx].velocities.append(0.0)
            self.robot_command.points[ndx].accelerations.append(0.0)
        ndx += 1


        dt = np.pi / (4.0 * self.frequency)
        time_offset = dt

        print "dt = ", dt
        while ndx < knot_points-1:
            self.robot_command.points.append(JointTrajectoryPoint())
            time_offset += dt
            print " time = ", time_offset
            self.robot_command.points[ndx].time_from_start = rospy.Duration(time_offset)
            for jnt, joint in enumerate(self.chain):
                self.robot_command.points[ndx].positions.append(joint.position + joint.amplitude * np.sin(self.frequency * time_offset))
                self.robot_command.points[ndx].velocities.append(self.frequency * joint.amplitude * np.cos(self.frequency * time_offset))
                self.robot_command.points[ndx].accelerations.append(-self.frequency * self.frequency * joint.amplitude * np.sin(self.frequency * time_offset))
            ndx += 1

            
        #end position
        self.robot_command.points.append(JointTrajectoryPoint())
        time_offset += dt
        self.robot_command.points[ndx].time_from_start = rospy.Duration(time_offset)
        for jnt, joint in enumerate(self.chain):
            self.robot_command.points[ndx].positions.append(joint.position)
            self.robot_command.points[ndx].velocities.append(0.0)
            self.robot_command.points[ndx].accelerations.append(0.0)

        print self.robot_command
        print "Ndx=",ndx, " of ",knot_points, "=",len(self.robot_command.points)
        if (ndx != len(self.robot_command.points)-1) or (len(self.robot_command.points) != knot_points):
            print "Invalid number of knot points - ignore trajectory"
            print self.robot_command
            return False

        self.robot_command.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        return True;



    def shutdown_plugin(self):
    #Just make sure to stop timers and publishers, unsubscribe from Topics etc in the shutdown_plugin method.
    #TODO, is the above comment why the plugin gets stuck when we try to shutdown?
        print "Shutdown ..."
        rospy.sleep(0.1)
        self.jointSubscriber.unregister()
        self.commandPublisher.unregister()
        rospy.sleep(0.5)
        print "Done!"


    # Define collection of widgets for joint group
    def joint_widget( self, main_vbox, index):
        joint = self.chain[index]
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel);
        frame.setFrameShadow(QFrame.Raised);

        hbox = QHBoxLayout()
        #hbox.addWidget(frame)

        self.prior_time = 0.0

        robot_joint = self.robot.joints[self.joint_list[joint.name]]
        joint.lower_limit = robot_joint.limit.lower
        joint.upper_limit = robot_joint.limit.upper
        amplitude_range = (robot_joint.limit.upper-robot_joint.limit.lower)
        frequency_range = self.frequency_limit
        iterations_range = 10000

        print "  ",joint.name, "  limits(", joint.lower_limit,", ",joint.upper_limit,") range=",amplitude_range

        self.cur_position_spinbox.append(QDoubleSpinBox())
        self.cur_position_spinbox[index].setDecimals(5)
        self.cur_position_spinbox[index].setRange(joint.lower_limit, joint.upper_limit)
        self.cur_position_spinbox[index].setSingleStep((robot_joint.limit.upper-robot_joint.limit.lower)/50.0)
        self.cur_position_spinbox[index].valueChanged.connect(joint.on_position_value)

        self.amplitude_spinbox.append(QDoubleSpinBox())
        self.amplitude_spinbox[index].setDecimals(5)
        self.amplitude_spinbox[index].setRange(-amplitude_range, amplitude_range)
        self.amplitude_spinbox[index].setSingleStep(amplitude_range/50.0)
        self.amplitude_spinbox[index].valueChanged.connect(joint.on_amplitude_value)
        self.amplitude_spinbox[index].setValue(joint.amplitude)

        hbox.addWidget(QLabel(joint.name))
        hbox.addWidget(self.cur_position_spinbox[index])
        hbox.addWidget(self.amplitude_spinbox[index])

        # Add horizontal layout to frame for this joint group
        frame.setLayout(hbox)

        # Add frame to main vertical layout
        main_vbox.addWidget(frame)

