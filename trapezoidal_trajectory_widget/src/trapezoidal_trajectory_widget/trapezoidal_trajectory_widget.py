import roslib
roslib.load_manifest('trapezoidal_trajectory_widget')

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
        self.main     = main
        self.name     = name
        self.position = 0.0
        self.ramp_up  = 0.0
        self.ramp_down= 0.0
        self.lower_limit = -6
        self.upper_limit =  6

    def on_cmd_value(self, value):
        self.position                             = copy.deepcopy(value)
        print "position[",self.name,"]=",self.position

    def on_ramp_up_value(self, value):
        self.ramp_up = copy.deepcopy(value)

    def on_ramp_down_value(self, value):
        self.ramp_down = copy.deepcopy(value)

class TrapezoidalTrajectoryDialog(Plugin):
    updateStateSignal = Signal(object)

    def __init__(self, context):
        super(TrapezoidalTrajectoryDialog, self).__init__(context)
        self.setObjectName('TrapezoidalTrajectoryDialog')
        #self.updateStateSignal = Signal(object)
        self.updateStateSignal.connect(self.on_updateState)

        self.robot = URDF.from_parameter_server()
        self.joint_list = {}

        for ndx,jnt in enumerate(self.robot.joints):
            self.joint_list[jnt.name] = ndx

        self.chain=[]
        self.chain_file = rospy.get_param("~chain_file")
        self.chain_name = rospy.get_param("~chain_name")

        print
        print "Define order of splines used to approximate trapezoid"
        self.spline_order = rospy.get_param("~spline_order", 5) # 1 # 3 # 5 # linear, cubic, quintic
        if (self.spline_order == 1) or (self.spline_order == 3) or (self.spline_order == 5):
            print "Spline order=",self.spline_order
        else:
            print "Invalid spline order!  Must be 1, 3, or 5"
            print "Spline order=",self.spline_order
            sys.exit(-1)


        yaml_file = self.chain_file+self.chain_name+"_chain.yaml"
        print "Chain definition file:"
        print yaml_file
        print

        #define structures
        self.robot_state = JointState()
        self.robot_command = JointTrajectory()


        stream = open(yaml_file, "r")
        jointChain = yaml.load_all(stream)

        for ndx, data in enumerate(jointChain):
            print ndx," : ", data
            self.delay_time     = data["delay_time"]
            self.ramp_up_time   = data["ramp_up_time"]
            self.dwell_time     = data["dwell_time"]
            self.ramp_down_time = data["ramp_down_time"]
            self.hold_time      = data["hold_time"]
            self.ramp_start_fraction = data["ramp_start_fraction"]
            self.ramp_end_fraction   = data["ramp_end_fraction"]

            self.joint_state_topic = data["joint_state_topic"]
            self.trajectory_topic = data["trajectory_topic"]

            if (rospy.search_param(data["chain_param_name"])):
                print "Found ",data["chain_param_name"]
            else:
                print "Failed to find the ",data["chain_param_name"]," in the parameter server!"
                sys.exit(-1)
                
            joint_names = rospy.get_param(data["chain_param_name"])
            for joint in joint_names:
                print joint
                self.robot_state.name.append(joint)
                self.chain.append(JointData(self, joint ) )

        self.robot_command.joint_names = self.robot_state.name

        stream.close()


        self.robot_state.position = [0.0]*len(self.robot_state.name)
        self.robot_state.velocity = [0.0]*len(self.robot_state.name)
        self.robot_state.effort   = [0.0]*len(self.robot_state.name)
        self.robot_joint_state = JointState()

        print "delay_time    =",self.delay_time
        print "ramp_up_time  =",self.ramp_up_time
        print "dwell_time    =",self.dwell_time
        print "ramp_down_time=",self.ramp_down_time
        print "hold_time     =",self.hold_time
        print "spline order  =",self.spline_order


        if ((self.ramp_start_fraction < 0.001) or (self.ramp_end_fraction > 0.999) or (self.ramp_start_fraction >= self.ramp_end_fraction)):
            print "Invalid ramp fractions - abort!"
            print "0.0 < ",self.ramp_start_fraction, " < ",self.ramp_end_fraction , " < 1.0"
            return



        print "Robot State Structure",self.robot_state
        print "Robot Command Structure",self.robot_command

        # initialize structure to hold widget handles
        self.cmd_spinbox=[]
        self.ramp_up_spinbox=[]
        self.ramp_down_spinbox=[]

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

        zero_ramp = QPushButton("Zero Ramps")
        zero_ramp.clicked.connect(self.zero_ramp_callback)
        hbox.addWidget(zero_ramp)

        vbox.addLayout(hbox)

        time_hbox = QHBoxLayout()

        vbox_delay = QVBoxLayout()
        vbox_delay.addWidget(QLabel("Delay"))
        self.delay_time_spinbox = QDoubleSpinBox()
        self.delay_time_spinbox.setDecimals(5)
        self.delay_time_spinbox.setRange(0, 10.0)
        self.delay_time_spinbox.setSingleStep(0.1)
        self.delay_time_spinbox.valueChanged.connect(self.on_delay_time_value)
        self.delay_time_spinbox.setValue(self.delay_time)
        vbox_delay.addWidget(self.delay_time_spinbox)
        time_hbox.addLayout(vbox_delay)

        vbox_ramp_up = QVBoxLayout()
        vbox_ramp_up.addWidget(QLabel("Ramp Up"))
        self.ramp_up_time_spinbox = QDoubleSpinBox()
        self.ramp_up_time_spinbox.setDecimals(5)
        self.ramp_up_time_spinbox.setRange(0, 10.0)
        self.ramp_up_time_spinbox.setSingleStep(0.1)
        self.ramp_up_time_spinbox.valueChanged.connect(self.on_ramp_up_time_value)
        self.ramp_up_time_spinbox.setValue(self.ramp_up_time)
        vbox_ramp_up.addWidget(self.ramp_up_time_spinbox)
        time_hbox.addLayout(vbox_ramp_up)

        #
        vbox_dwell = QVBoxLayout()
        vbox_dwell.addWidget(QLabel("Dwell"))
        self.dwell_time_spinbox = QDoubleSpinBox()
        self.dwell_time_spinbox.setDecimals(5)
        self.dwell_time_spinbox.setRange(0, 10.0)
        self.dwell_time_spinbox.setSingleStep(0.1)
        self.dwell_time_spinbox.valueChanged.connect(self.on_dwell_time_value)
        self.dwell_time_spinbox.setValue(self.dwell_time)
        vbox_dwell.addWidget(self.dwell_time_spinbox)
        time_hbox.addLayout(vbox_dwell)

        vbox_ramp_down = QVBoxLayout()
        vbox_ramp_down.addWidget(QLabel("Down"))
        self.ramp_down_time_spinbox = QDoubleSpinBox()
        self.ramp_down_time_spinbox.setDecimals(5)
        self.ramp_down_time_spinbox.setRange(0, 10.0)
        self.ramp_down_time_spinbox.setSingleStep(0.1)
        self.ramp_down_time_spinbox.valueChanged.connect(self.on_ramp_down_time_value)
        self.ramp_down_time_spinbox.setValue(self.ramp_down_time)
        vbox_ramp_down.addWidget(self.ramp_down_time_spinbox)
        time_hbox.addLayout(vbox_ramp_down)

        vbox_hold = QVBoxLayout()
        vbox_hold.addWidget(QLabel("Hold"))
        self.hold_time_spinbox = QDoubleSpinBox()
        self.hold_time_spinbox.setDecimals(5)
        self.hold_time_spinbox.setRange(0, 10.0)
        self.hold_time_spinbox.setSingleStep(0.1)
        self.hold_time_spinbox.valueChanged.connect(self.on_hold_time_value)
        self.hold_time_spinbox.setValue(self.hold_time)
        vbox_hold.addWidget(self.hold_time_spinbox)
        time_hbox.addLayout(vbox_hold)


        vbox.addLayout(time_hbox)

        # Joints title
        title_frame = QFrame()
        title_frame.setFrameShape(QFrame.StyledPanel);
        title_frame.setFrameShadow(QFrame.Raised);

        title_hbox = QHBoxLayout()
        title_hbox.addWidget(QLabel("Joints"))
        title_hbox.addWidget(QLabel("Start"))
        title_hbox.addWidget(QLabel("Ramp Up"))
        title_hbox.addWidget(QLabel("Ramp Down"))
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
                    self.cmd_spinbox[index].setValue(joint.position)
                    break

        self.blockSignals(False)

    @Slot()
    def check_limits_callback(self):
        self.blockSignals(True)
        print "Check limits callback ..."
        valid = True
        for index, joint in enumerate(self.chain):

            ramp_up   = joint.position + joint.ramp_up
            ramp_down = joint.ramp_down + ramp_up

            p = self.ramp_up_spinbox[index].palette()
            if (ramp_up > joint.upper_limit) or (ramp_up < joint.lower_limit):
                print "Joint ",joint.name, " is over limit on ramp up!"
                valid=False
                p.setColor(QPalette.Window, Qt.red)  # <<<<<<<<<<<----------------- This isn't working as intended
            else:
                p.setColor(QPalette.Window, Qt.white)# <<<<<<<<<<<----------------- This isn't working as intended
                #print joint.lower_limit," < ", ramp_up, " < ",joint.upper_limit

            self.ramp_up_spinbox[index].setPalette(p)

            if (ramp_down > joint.upper_limit) or (ramp_down < joint.lower_limit):
                print "Joint ",joint.name, " is over limit on ramp down!"
                valid=False
                p.setColor(QPalette.Window, Qt.red)  # <<<<<<<<<<<----------------- This isn't working as intended
            else:
                p.setColor(QPalette.Window, Qt.white)# <<<<<<<<<<<----------------- This isn't working as intended
                #print joint.lower_limit," < ", ramp_down, " < ",joint.upper_limit

        if (not valid):
            print "Invalid joint limits on ramp!"
        else:
            print "   Valid ramp!"

        self.blockSignals(False)
        return valid

    @Slot()
    def zero_ramp_callback(self):
        self.blockSignals(True)
        print "Zero ramps callback ..."
        for index, joint in enumerate(self.chain):
            self.ramp_up_spinbox[index].setValue(0.0)
            self.ramp_down_spinbox[index].setValue(0.0)
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

    def on_ramp_up_time_value(self, value):
        self.ramp_up_time = copy.deepcopy(value)

    def on_ramp_down_time_value(self, value):
        self.ramp_down_time = copy.deepcopy(value)

    def on_dwell_time_value(self, value):
        self.dwell_time = copy.deepcopy(value)

    def on_hold_time_value(self, value):
        self.hold_time = copy.deepcopy(value)



    def calculate_trajectory(self):

        if not self.check_limits_callback():
            print "Invalid limits for trajectory!"
            return False


        knot_points = 1
        if (self.delay_time > 0.002):
            knot_points += 1

        if (self.ramp_up_time > 0.002):
            knot_points += 1

        if (self.dwell_time > 0.002):
            knot_points += 1

        if (self.ramp_down_time > 0.002):
            knot_points += 1

        if (self.hold_time > 0.002):
            knot_points += 1

        if (knot_points < 2):
            print "Invalid trajectory - knot_points = ",knot_points
            return False

        #print "Minimum knot points=",knot_points
        self.robot_command.points = []
        self.robot_command.points.append(JointTrajectoryPoint())

        ros_time_offset = rospy.Duration(0.0)
        ndx = 0
        self.robot_command.points[ndx].time_from_start = ros_time_offset
        for jnt, joint in enumerate(self.chain):
            self.robot_command.points[ndx].positions.append(joint.position)
            if (self.spline_order > 1):
                self.robot_command.points[ndx].velocities.append(0.0)
            if (self.spline_order > 3):
                self.robot_command.points[ndx].accelerations.append(0.0)
        ndx += 1

        if (self.delay_time > 0.002):
            ros_time_offset += rospy.Duration(self.delay_time)
            self.robot_command.points.append(copy.deepcopy(self.robot_command.points[ndx-1]))
            self.robot_command.points[ndx].time_from_start = ros_time_offset
            ndx += 1

        if (self.ramp_up_time > 0.002):
            ramp_up_count = self.calculate_ramp_up_section(ndx)
            if (ramp_up_count):
                ndx += ramp_up_count
                ros_time_offset += rospy.Duration(self.ramp_up_time)
            else:
                return False # Invalid ramp calculation

        if (self.dwell_time > 0.002):
            ros_time_offset += rospy.Duration(self.dwell_time)
            self.robot_command.points.append(copy.deepcopy(self.robot_command.points[ndx-1]))
            self.robot_command.points[ndx].time_from_start = ros_time_offset
            ndx += 1

        if (self.ramp_down_time > 0.002):
            ramp_dn_count = self.calculate_ramp_down_section(ndx)
            if (ramp_dn_count > 0):
                ndx += ramp_dn_count
                ros_time_offset += rospy.Duration(self.ramp_down_time)
            else:
                return False # Invalid ramp calculation

        if (self.hold_time > 0.002):
            ros_time_offset += rospy.Duration(self.hold_time)
            self.robot_command.points.append(copy.deepcopy(self.robot_command.points[ndx-1]))
            self.robot_command.points[ndx].time_from_start = ros_time_offset
            ndx += 1

        #print self.robot_command
        #print "Ndx=",ndx, " of ",knot_points, "=",len(self.robot_command.points)

        if (ndx != len(self.robot_command.points)):
            print "Invalid number of knot points - ignore trajectory"
            print "Ndx=",ndx, " of ",len(self.robot_command.points)," = ",knot_points
            print self.robot_command
            return False

        self.robot_command.header.stamp = rospy.Time.now() + rospy.Duration(0.1)

        return True;

    def calculate_ramp_up_section(self, ndx):
        prior = self.robot_command.points[ndx-1]

        # Create next 3 knot points
        self.robot_command.points.append(copy.deepcopy(prior))
        if (self.spline_order > 1):
            # Add transition points
            self.robot_command.points.append(copy.deepcopy(prior))
            self.robot_command.points.append(copy.deepcopy(prior))
            dT0 = self.ramp_start_fraction*self.ramp_up_time
            dT1 = (self.ramp_end_fraction - self.ramp_start_fraction)*self.ramp_up_time
            dT2 = (1.0 - self.ramp_end_fraction)*self.ramp_up_time
            self.robot_command.points[ndx].time_from_start = prior.time_from_start + rospy.Duration(dT0)
            self.robot_command.points[ndx+1].time_from_start = self.robot_command.points[ndx].time_from_start + rospy.Duration(dT1)
            self.robot_command.points[ndx+2].time_from_start = self.robot_command.points[ndx+1].time_from_start + rospy.Duration(dT2)
        else:
            dT0 = self.ramp_up_time
            self.robot_command.points[ndx].time_from_start = prior.time_from_start + rospy.Duration(dT0)

        # Now calculate the interior values for each joint individually
        for jnt, joint in enumerate(self.chain):
            if (joint.ramp_up != 0.0):
                starting_position = copy.deepcopy(prior.positions[jnt])
                ending_position   = starting_position + joint.ramp_up

                print "calculating ramp up for ",joint.name, " from ",starting_position," to ",ending_position," over ",self.ramp_up_time," seconds"

                if (self.spline_order > 1):
                    # Either quintic or cubic - either way we have transition -> linear -> transition
                    # for quintic,
                    #  Ax=b  where x=[a0 b0 c0 d0 e0 f0 e1 f1 a2 b2 c2 d2 e2 f2] with p=a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t +f for first and last, and p=e*t + f for middle
                    #  6 + 2 + 6 unknowns
                    x = self.solve_ramp_matrices(dT0, dT1, dT2, starting_position, ending_position)
                    # first interior point
                    self.robot_command.points[ndx].positions[jnt]     = copy.deepcopy(x[7][0])
                    self.robot_command.points[ndx].velocities[jnt]    = copy.deepcopy(x[6][0])
                    # linear segment
                    self.robot_command.points[ndx+1].positions[jnt]   = copy.deepcopy(x[13][0])
                    self.robot_command.points[ndx+1].velocities[jnt]  = copy.deepcopy(x[6][0])
                    # last interior point
                    self.robot_command.points[ndx+2].positions[jnt]   = ending_position
                    self.robot_command.points[ndx+2].velocities[jnt]  =  0.0
                else:
                    # Piecewise linear
                    self.robot_command.points[ndx].positions[jnt]     = copy.deepcopy(ending_position)

                if (self.spline_order > 3):
                    # Quintic spline
                    self.robot_command.points[ndx].accelerations[jnt]   =  0.0
                    self.robot_command.points[ndx+1].accelerations[jnt] =  0.0
                    self.robot_command.points[ndx+2].accelerations[jnt] =  0.0

        if (self.spline_order < 3):
            return 1

        return 3 # At least cubic and has 3 transition segments

    def calculate_ramp_down_section(self, ndx):
        prior = self.robot_command.points[ndx-1]

        # Create next knot points
        self.robot_command.points.append(copy.deepcopy(prior))
        if (self.spline_order > 1):
            # Add transition points
            self.robot_command.points.append(copy.deepcopy(prior))
            self.robot_command.points.append(copy.deepcopy(prior))
            dT0 = self.ramp_start_fraction*self.ramp_down_time
            dT1 = (self.ramp_end_fraction - self.ramp_start_fraction)*self.ramp_down_time
            dT2 = (1.0 - self.ramp_end_fraction)*self.ramp_down_time
            self.robot_command.points[ndx].time_from_start = prior.time_from_start + rospy.Duration(dT0)
            self.robot_command.points[ndx+1].time_from_start = self.robot_command.points[ndx].time_from_start + rospy.Duration(dT1)
            self.robot_command.points[ndx+2].time_from_start = self.robot_command.points[ndx+1].time_from_start + rospy.Duration(dT2)
        else:
            dT0 = self.ramp_down_time
            self.robot_command.points[ndx].time_from_start = prior.time_from_start + rospy.Duration(dT0)

        # Now calculate the interior values for each joint individually
        for jnt, joint in enumerate(self.chain):
            if (joint.ramp_up != 0.0):
                starting_position = copy.deepcopy(prior.positions[jnt])
                ending_position   = starting_position + joint.ramp_down

                print "calculating ramp down for ",joint.name, " from ",starting_position," to ",ending_position," over ",self.ramp_down_time," seconds"

                if (self.spline_order > 1):
                    # Ax=b  where x=[a0 b0 c0 d0 e0 f0 e1 f1 a2 b2 c2 d2 e2 f2] with p=a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t +f for first and last, and p=e*t + f for middle
                    #  6 + 2 + 6 unknowns
                    x = self.solve_ramp_matrices(dT0, dT1, dT2, starting_position, ending_position)
                    # first interior point
                    self.robot_command.points[ndx].positions[jnt]     = copy.deepcopy(x[7][0])
                    self.robot_command.points[ndx].velocities[jnt]    = copy.deepcopy(x[6][0])
                    # linear segment
                    self.robot_command.points[ndx+1].positions[jnt]   = copy.deepcopy(x[13][0])
                    self.robot_command.points[ndx+1].velocities[jnt]  = copy.deepcopy(x[6][0])
                    # last interior point
                    self.robot_command.points[ndx+2].positions[jnt]   = ending_position
                    self.robot_command.points[ndx+2].velocities[jnt]  =  0.0
                else:
                    self.robot_command.points[ndx].positions[jnt]     = copy.deepcopy(ending_position)

                if (self.spline_order > 3):
                    self.robot_command.points[ndx+2].accelerations[jnt] =  0.0
                    self.robot_command.points[ndx].accelerations[jnt]   =  0.0
                    self.robot_command.points[ndx+1].accelerations[jnt] =  0.0

        if (self.spline_order < 3):
            return 1

        return 3 # At least cubic and has 3 transition segments

    def solve_ramp_matrices(self, dT0, dT1, dT2, starting_position, ending_position):

        if (self.spline_order < 4):
            # assume cubic spline
            return self.solve_ramp_cubic_matrices(dT0, dT1, dT2, starting_position, ending_position)


        # This is the quintic version
        # Ax=b  where x=[a0 b0 c0 d0 e0 f0 e1 f1 a2 b2 c2 d2 e2 f2] with p=a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t +f for first and last, and p=e*t + f for middle
        #  6 + 2 + 6 = 14 unknowns
        A = np.zeros((14,14))
        b = np.zeros((14,1))

        #Eqn 1  - Known starting position
        A[0][5] = 1
        b[0][0] = starting_position

        #Eqn 2  - Zero velocity at start
        A[1][4] = 1.0

        #Eqn 3  - Zero acceleration at start
        A[2][3] = 2.0

        #Eqn 4  - Continuous velocity at first interior knot point
        A[3][0] = 5*dT0**4
        A[3][1] = 4*dT0**3
        A[3][2] = 3*dT0**2
        A[3][3] = 2*dT0
        A[3][4] = 1.0
        A[3][5] = 0.0
        A[3][6] = -1.0

        #Eqn 5  - Zero acceleration at first interior knot point
        A[4][0] = 20*dT0**3
        A[4][1] = 12*dT0**2
        A[4][2] = 6*dT0
        A[4][3] = 2
        A[4][4] = 0.0
        A[4][5] = 0.0
        A[4][6] = 0.0

        #Eqn 6  - Zero jerk at first interior knot point
        A[5][0] = 60*dT0**2
        A[5][1] = 24*dT0
        A[5][2] = 6
        A[5][3] = 0.0
        A[5][4] = 0.0
        A[5][5] = 0.0
        A[5][6] = 0.0


        #Eqn 7  - Continuous velocity at second interior knot point
        A[6][ 8] = 0.0
        A[6][ 9] = 0.0
        A[6][10] = 0.0
        A[6][11] = 0.0
        A[6][12] = 1.0
        A[6][13] = 0.0
        A[6][ 6] = -1.0

        #Eqn 8  -  Zero acceleration at second interior knot point
        A[7][ 8] = 0.0
        A[7][ 9] = 0.0
        A[7][10] = 0.0
        A[7][11] = 2
        A[7][12] = 0.0
        A[7][13] = 0.0
        A[7][ 6] = 0.0

        #Eqn 9  - Zero jerk at second interior knot point
        A[8][ 8] = 0.0
        A[8][ 9] = 0.0
        A[8][10] = 6
        A[8][11] = 0.0
        A[8][12] = 0.0
        A[8][13] = 0.0
        A[8][ 6] = 0.0


        #Eqn 10 - Equal (but unknown) positions at first interior knot point
        A[9][0] = dT0**5
        A[9][1] = dT0**4
        A[9][2] = dT0**3
        A[9][3] = dT0**2
        A[9][4] = dT0
        A[9][5] = 1.0
        A[9][7] = -1.0


        #Eqn 11 - Equal (but unknown) positions at second interior knot point
        A[10][13] = 1.0
        A[10][6]  = -dT1
        A[10][7]  = -1.0

        #Eqn 12 - Known final position
        A[11][ 8] = dT2**5
        A[11][ 9] = dT2**4
        A[11][10] = dT2**3
        A[11][11] = dT2**2
        A[11][12] = dT2
        A[11][13] = 1.0
        b[11][0]  = ending_position

        #Eqn 13 - Known final velocity = 0
        A[12][ 8] = 5*dT2**4
        A[12][ 9] = 4*dT2**3
        A[12][10] = 3*dT2**2
        A[12][11] = 2*dT2
        A[12][12] = 1.0
        A[12][13] = 0.0

        #Eqn 14 - Known final acceleration = 0
        A[13][ 8] = 20*dT2**3
        A[13][ 9] = 12*dT2**2
        A[13][10] =  6*dT2
        A[13][11] =  2.0
        A[13][12] =  0.0
        A[13][13] =  0.0

        params = np.linalg.solve(A,b)

        #np.set_printoptions(precision=9, suppress=True, linewidth=250)
        #print "A=",A
        #print "b=",b
        #print "condition number(A)=",np.linalg.cond(A)
        #print "x=",params

        return params


    def solve_ramp_cubic_matrices(self, dT0, dT1, dT2, starting_position, ending_position):


        # This is the cubic version
        # Quintic version: Ax=b  where x=[a0 b0 c0 d0 e0 f0 e1 f1 a2 b2 c2 d2 e2 f2] with p=a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t +f for first and last, and p=e*t + f for middle
        # Cubic   version: Ax=b  where x=[c0 d0 e0 f0 e1 f1 c2 d2 e2 f2] with p= c*t^3 + d*t^2 + e*t +f for first and last, and p=e*t + f for middle
        #
        #  4 + 2 + 4 = 10 unknowns
        A = np.zeros((10,10))
        b = np.zeros((10,1))

        #Eqn 1  - Known starting position
        A[0][3] = 1
        b[0][0] = starting_position

        #Eqn 2  - Zero velocity at start
        A[1][2] = 1.0

        #Eqn 3  - Continuous velocity at first interior knot point
        A[2][0] = 3*dT0**2
        A[2][1] = 2*dT0
        A[2][2] = 1.0
        A[2][3] = 0.0
        A[2][4] = -1.0

        #Eqn 4  - Zero acceleration at first interior knot point
        A[3][0] = 6*dT0
        A[3][1] = 2
        A[3][2] = 0.0
        A[3][3] = 0.0

        #Eqn 5  - Continuous velocity at second interior knot point
        A[4][6] =  0.0
        A[4][7] =  0.0
        A[4][8] =  1.0
        A[4][9] =  0.0
        A[4][4] = -1.0

        #Eqn 6  -  Zero acceleration at second interior knot point
        A[5][6] = 0.0
        A[5][7] = 2.0
        A[5][8] = 0.0
        A[5][9] = 0.0

        #Eqn 7 - Equal (but unknown) positions at first interior knot point
        A[6][0] = dT0**3
        A[6][1] = dT0**2
        A[6][2] = dT0
        A[6][3] = 1.0
        A[6][5] = -1.0


        #Eqn 8 - Equal (but unknown) positions at second interior knot point
        A[7][9] =  1.0
        A[7][4] = -dT1
        A[7][5] = -1.0

        #Eqn 9 - Known final position
        A[8][6] = dT2**3
        A[8][7] = dT2**2
        A[8][8] = dT2
        A[8][9] = 1.0
        b[8][0] = ending_position

        #Eqn 10 - Known final velocity = 0
        A[9][6] = 3*dT2**2
        A[9][7] = 2*dT2
        A[9][8] = 1.0
        A[9][9] = 0.0

        params = np.linalg.solve(A,b)

        quintic_params = np.zeros((14,1))
        quintic_params[2:8] = params[0:6]
        quintic_params[10:14] = params[6:10]

        np.set_printoptions(precision=9, suppress=True, linewidth=250)
        print "A=",A
        print "b=",b
        print "condition number(A)=",np.linalg.cond(A)
        print "x=",params
        #print "-----------------------------------------------------------------------"
        #print "--------- Cubic solve -------------------------------------------------"
        #print "cubic params=",params
        #print "quintic params = ", quintic_params
        #print "-----------------------------------------------------------------------"
        return quintic_params


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
        ramp_range = (robot_joint.limit.upper-robot_joint.limit.lower)

        print "  ",joint.name, "  limits(", joint.lower_limit,", ",joint.upper_limit,") range=",ramp_range

        self.cmd_spinbox.append(QDoubleSpinBox())
        self.cmd_spinbox[index].setDecimals(5)
        self.cmd_spinbox[index].setRange(joint.lower_limit, joint.upper_limit)
        self.cmd_spinbox[index].setSingleStep((robot_joint.limit.upper-robot_joint.limit.lower)/50.0)
        self.cmd_spinbox[index].valueChanged.connect(joint.on_cmd_value)


        self.ramp_up_spinbox.append(QDoubleSpinBox())
        self.ramp_up_spinbox[index].setDecimals(5)
        self.ramp_up_spinbox[index].setRange(-ramp_range, ramp_range)
        self.ramp_up_spinbox[index].setSingleStep(ramp_range/50.0)
        self.ramp_up_spinbox[index].valueChanged.connect(joint.on_ramp_up_value)

        self.ramp_down_spinbox.append(QDoubleSpinBox())
        self.ramp_down_spinbox[index].setDecimals(5)
        self.ramp_down_spinbox[index].setRange(-ramp_range, ramp_range)
        self.ramp_down_spinbox[index].setSingleStep(ramp_range/50.0)
        self.ramp_down_spinbox[index].valueChanged.connect(joint.on_ramp_down_value)

        hbox.addWidget(QLabel(joint.name))
        hbox.addWidget(self.cmd_spinbox[index])
        hbox.addWidget(self.ramp_up_spinbox[index])
        hbox.addWidget(self.ramp_down_spinbox[index])

        # Add horizontal layout to frame for this joint group
        frame.setLayout(hbox)

        # Add frame to main vertical layout
        main_vbox.addWidget(frame)

