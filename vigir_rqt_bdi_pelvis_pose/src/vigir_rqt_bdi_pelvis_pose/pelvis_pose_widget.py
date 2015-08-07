import roslib
roslib.load_manifest('vigir_rqt_bdi_pelvis_pose')
from time import strftime, localtime
import rospy
import tf
import sys, copy, math
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Signal, Slot, QObject, Qt
from python_qt_binding.QtGui import QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QFrame, QSlider, QProgressBar, QSpacerItem, QCheckBox

from urdf_parser_py.urdf import URDF

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

class BDIPelvisPoseWidget(QObject):
    updateStateSignal = Signal(object)

    def __init__(self, context):
        #super(BDIPelvisPoseWidget, self).__init__(context)
        #self.setObjectName('BDIPelvisPoseWidget')
        super(BDIPelvisPoseWidget, self).__init__()
        self.name = 'BDIPelvisPoseWidget'

        self.updateStateSignal.connect(self.on_updateState)

        #self._widget = QWidget()
        self._widget = context
        vbox = QVBoxLayout()

        self.forward_position =   0.0
        self.lateral_position =   0.0
        self.height_position  =   0.91
        self.roll_position    =   0.0
        self.pitch_position   =   0.0
        self.yaw_position     =   0.0

        self.currentForward   =   0.0
        self.currentLateral   =   0.0
        self.currentHeight    =   0.91
        self.currentRoll      =   0.0
        self.currentPitch     =   0.0
        self.currentYaw       =   0.0
        # Define checkboxes
        vbox = QVBoxLayout()
        label = QLabel()
        label.setText("BDI Pelvis Height (Manipulate Mode Only)") # todo - connect controller mode
        vbox.addWidget(label);

        self.enable_checkbox = QCheckBox("Enable")
        self.enable_checkbox.stateChanged.connect(self.on_enable_check)
        vbox.addWidget(self.enable_checkbox)

        self.snap_to_current_button = QPushButton("Snap to Current")
        self.snap_to_current_button.pressed.connect(self.on_snapCurrentPressed)
        vbox.addWidget(self.snap_to_current_button)

        self.roll_slider = QSlider(Qt.Horizontal)
        self.roll_label = QLabel()
        self.roll_label.setText("Roll")
        vbox.addWidget(self.roll_label)


        self.roll_slider.setRange(int(-100), int(101))
        self.roll_slider.setValue(int(0))
        self.roll_slider.setSingleStep((200)/50)
        self.roll_slider.setTickInterval(25)
        self.roll_slider.valueChanged.connect(self.on_rollSliderMoved)
        vbox.addWidget(self.roll_slider)

        self.roll_progress_bar = QProgressBar()
        self.roll_progress_bar.setRange(int(-100),int(101))
        self.roll_progress_bar.setValue((6.0/math.pi)*self.currentRoll*100)
        self.roll_progress_bar.setFormat("%.6f" % self.currentRoll)
        vbox.addWidget(self.roll_progress_bar)

        self.pitch_slider = QSlider(Qt.Horizontal)
        self.pitch_label = QLabel()
        self.pitch_label.setText("Pitch")
        vbox.addWidget(self.pitch_label)

        self.pitch_slider.setRange(int(-100), int(101))
        self.pitch_slider.setValue(int(0))
        self.pitch_slider.setSingleStep((200)/50)
        self.pitch_slider.setTickInterval(25)
        self.pitch_slider.valueChanged.connect(self.on_pitchSliderMoved)
        vbox.addWidget(self.pitch_slider)

        self.pitch_progress_bar = QProgressBar()
        self.pitch_progress_bar.setRange(int(-100),int(101))
        self.pitch_progress_bar.setValue((6.0/math.pi)*self.currentPitch*100)
        self.pitch_progress_bar.setFormat("%.6f" % self.currentPitch)
        vbox.addWidget(self.pitch_progress_bar)

        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_label = QLabel()
        self.yaw_label.setText("Yaw")
        vbox.addWidget(self.yaw_label)
        
        self.yaw_slider.setRange(int(-100), int(101))
        self.yaw_slider.setValue(int(0))
        self.yaw_slider.setSingleStep((200)/50)
        self.yaw_slider.setTickInterval(25)
        self.yaw_slider.valueChanged.connect(self.on_yawSliderMoved)
        vbox.addWidget(self.yaw_slider)

        self.yaw_progress_bar = QProgressBar()
        self.yaw_progress_bar.setRange(int(-100),int(101))
        self.yaw_progress_bar.setValue((4.0/math.pi)*self.currentYaw*100)
        self.yaw_progress_bar.setFormat("%.6f" % self.currentYaw)
        vbox.addWidget(self.yaw_progress_bar)

        self.forward_label = QLabel()
        self.forward_label.setText("Forward")
        vbox.addWidget(self.forward_label)

        widget = QWidget()
        hbox   = QHBoxLayout()
        hbox.addStretch()
        self.forward_slider = QSlider(Qt.Vertical)
        #self.forward_slider.setText("Height")
        self.forward_slider.setRange(int(-101), int(100))
        self.forward_slider.setValue(int(0))
        self.forward_slider.setSingleStep(1)
        self.forward_slider.setTickInterval(10)
        self.forward_slider.valueChanged.connect(self.on_forwardSliderMoved)
        hbox.addWidget(self.forward_slider)

        self.forward_progress_bar = QProgressBar()
        self.forward_progress_bar.setOrientation(Qt.Vertical)
        self.forward_progress_bar.setRange(int(-100),int(101))
        self.forward_progress_bar.setValue((self.currentForward/0.075)*100)
        self.forward_progress_bar.setTextVisible(False)
        hbox.addWidget(self.forward_progress_bar)

        self.forward_progress_bar_label = QLabel()
        self.forward_progress_bar_label.setText("%.6f" % self.currentForward)
        hbox.addWidget(self.forward_progress_bar_label)
        hbox.addStretch()
        widget.setLayout(hbox)
        vbox.addWidget(widget)


        self.lateral_label = QLabel()
        self.lateral_label.setText("Lateral")
        vbox.addWidget(self.lateral_label)

        self.lateral_slider = QSlider(Qt.Horizontal)
        #self.lateral_slider.setText("Lateral")
        self.lateral_slider.setRange(int(-100), int(101))
        self.lateral_slider.setValue(int(0))
        self.lateral_slider.setSingleStep((200)/50)
        self.lateral_slider.setTickInterval(25)
        self.lateral_slider.valueChanged.connect(self.on_lateralSliderMoved)
        vbox.addWidget(self.lateral_slider)

        self.lateral_progress_bar = QProgressBar()
        self.lateral_progress_bar.setRange(int(-100),int(101))
        self.lateral_progress_bar.setValue((self.currentLateral/0.15)*100)
        self.lateral_progress_bar.setFormat("%.6f" % self.currentLateral)
        vbox.addWidget(self.lateral_progress_bar)

        self.height_label = QLabel()
        self.height_label.setText("Height")
        vbox.addWidget(self.height_label)

        widget = QWidget()
        hbox   = QHBoxLayout()
        hbox.addStretch()
        self.height_slider = QSlider(Qt.Vertical)
        #self.height_slider.setText("Height")
        self.height_slider.setRange(int(55), int(105))
        self.height_slider.setValue(int(91))
        self.height_slider.setSingleStep(2)
        self.height_slider.setTickInterval(10)
        self.height_slider.valueChanged.connect(self.on_heightSliderMoved)
        hbox.addWidget(self.height_slider)

        self.height_progress_bar = QProgressBar()
        self.height_progress_bar.setOrientation(Qt.Vertical)
        self.height_progress_bar.setRange(int(55),int(105))
        self.height_progress_bar.setValue(self.currentHeight*100)
        self.height_progress_bar.setTextVisible(False)
        hbox.addWidget(self.height_progress_bar)

        self.height_progress_bar_label = QLabel()
        self.height_progress_bar_label.setText("%.6f" % self.currentHeight)
        hbox.addWidget(self.height_progress_bar_label)
        hbox.addStretch()
        widget.setLayout(hbox)
        vbox.addWidget(widget)
        vbox.addStretch()

        self._widget.setLayout(vbox)
        #context.add_widget(self._widget)
        self.height_position  = 0.91
        self.lateral_position = 0.0
        self.yaw_position     = 0.0
        self.first_time = True
        self.enable_checkbox.setChecked(False)
        self.yaw_slider.setEnabled(False)
        self.roll_slider.setEnabled(False)
        self.pitch_slider.setEnabled(False)
        self.forward_slider.setEnabled(False)
        self.lateral_slider.setEnabled(False)
        self.height_slider.setEnabled(False)

        self.pub_robot       = rospy.Publisher('/flor/controller/bdi_desired_pelvis_pose',PoseStamped, queue_size=10)
        #self.stateSubscriber = rospy.Subscriber('/flor/pelvis_controller/current_states',JointState, self.stateCallbackFnc)
        #self.pub_bdi_pelvis  = rospy.Publisher('/bdi_manipulate_pelvis_pose_rpy',PoseStamped)
        self.pelvis_trajectory_pub = rospy.Publisher('/robot_controllers/pelvis_traj_controller/command', JointTrajectory, queue_size=10)
        self.stateSubscriber = rospy.Subscriber('/robot_controllers/pelvis_traj_controller/state', JointTrajectoryControllerState, self.stateCallbackFnc)

    def stateCallbackFnc(self, jointState_msg):
        self.updateStateSignal.emit(jointState_msg)

    def shutdown_plugin(self):
    #Just make sure to stop timers and publishers, unsubscribe from Topics etc in the shutdown_plugin method.
        print "Shutdown BDI pelvis height "
        self.pub_robot.unregister()
        self.stateSubscriber.unregister()

    def publishRobotPelvisPose(self):
        print "publishing new pelvis pose ..."
        bdi_pelvis_pose = PoseStamped()
        bdi_pelvis_pose.header.stamp = rospy.Time.now()
        bdi_pelvis_pose.pose.position.x   = self.forward_position
        bdi_pelvis_pose.pose.position.y   = self.lateral_position
        bdi_pelvis_pose.pose.position.z   = self.height_position

        # Use BDI yaw*roll*pitch concatenation
        xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
        Rx = tf.transformations.rotation_matrix(self.roll_position, xaxis)
        Ry = tf.transformations.rotation_matrix(self.pitch_position, yaxis)
        Rz = tf.transformations.rotation_matrix(self.yaw_position, zaxis)
        R =  tf.transformations.concatenate_matrices(Rz, Rx, Ry)
        q = tf.transformations.quaternion_from_matrix(R)

        bdi_pelvis_pose.pose.orientation.w = q[3]
        bdi_pelvis_pose.pose.orientation.x = q[0]
        bdi_pelvis_pose.pose.orientation.y = q[1]
        bdi_pelvis_pose.pose.orientation.z = q[2]

        #w   = math.cos(self.yaw_position*0.5)
        #bdi_pelvis_pose.pose.orientation.x   = 0.0
        #bdi_pelvis_pose.pose.orientation.y   = 0.0
        #bdi_pelvis_pose.pose.orientation.z   = math.sin(self.yaw_position*0.5)

        print bdi_pelvis_pose
        print q
        euler = tf.transformations.euler_from_quaternion(q)
        print euler
        self.pub_robot.publish(bdi_pelvis_pose)

        # Now publish the trajectory form for the new controllers
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = [ "com_v1",      "com_v0",        "pelvis_height",
                                   "pelvis_roll", "pelvis_pitch",  "pelvis_yaw"]



        trajectory.points = [JointTrajectoryPoint()]
        trajectory.points[0].positions = [self.lateral_position, self.forward_position, self.height_position,
                                          self.roll_position,    self.pitch_position,   self.yaw_position]
        trajectory.points[0].velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        trajectory.points[0].time_from_start = rospy.Duration(0.75)
        self.pelvis_trajectory_pub.publish(trajectory)


###################################
    def on_snapCurrentPressed(self):
        print "Snap ",self.name," values to current pelvis positions"
        self.blockSignals(True)
        self.resetCurrentPelvisSliders()
        self.blockSignals(False)

    def blockSignals(self, block):
        self.yaw_slider.blockSignals(block)
        self.roll_slider.blockSignals(block)
        self.pitch_slider.blockSignals(block)
        self.forward_slider.blockSignals(block)
        self.lateral_slider.blockSignals(block)
        self.height_slider.blockSignals(block)

    def resetCurrentPelvisSliders(self):
        self.yaw_slider.setValue((4.0/math.pi)*self.currentYaw*100)
        self.roll_slider.setValue((6.0/math.pi)*self.currentRoll*100)
        self.pitch_slider.setValue((6.0/math.pi)*self.currentPitch*100)
        self.forward_slider.setValue((self.currentForward/0.075)*100)
        self.lateral_slider.setValue((self.currentLateral/0.15)*100)
        self.height_slider.setValue(self.currentHeight*100)
        self.yaw_label.setText("Yaw: "+str(self.currentYaw))
        self.roll_label.setText("Roll: "+str(self.currentRoll))
        self.pitch_label.setText("Pitch: "+str(self.currentPitch))
        self.forward_label.setText("Forward: "+str(self.currentForward))
        self.lateral_label.setText("Lateral: "+str(self.currentLateral))
        self.height_label.setText("Height: "+str(self.currentHeight))
        self.forward_position = self.currentForward
        self.lateral_position = self.currentLateral
        self.height_position  = self.currentHeight
        self.roll_position    = self.currentRoll
        self.pitch_position   = self.currentPitch
        self.yaw_position     = self.currentYaw

    def on_updateState(self, jointState_msg):

        self.currentLateral= jointState_msg.actual.positions[0]
        self.currentForward= jointState_msg.actual.positions[1]
        self.currentHeight = jointState_msg.actual.positions[2]
        self.currentRoll   = jointState_msg.actual.positions[3]
        self.currentPitch  = jointState_msg.actual.positions[4]
        self.currentYaw    = jointState_msg.actual.positions[5]

        self.yaw_progress_bar.setValue((4.0/math.pi)*self.currentYaw*100)
        self.yaw_progress_bar.setFormat("%.6f" % self.currentYaw)

        self.roll_progress_bar.setValue((6.0/math.pi)*self.currentRoll*100)
        self.roll_progress_bar.setFormat("%.6f" % self.currentRoll)

        self.pitch_progress_bar.setValue((6.0/math.pi)*self.currentPitch*100)
        self.pitch_progress_bar.setFormat("%.6f" % self.currentPitch)

        self.forward_progress_bar.setValue((self.currentForward/0.075)*100)
        self.forward_progress_bar_label.setText("%.6f" % self.currentForward)

        self.lateral_progress_bar.setValue((self.currentLateral/0.15)*100)
        self.lateral_progress_bar.setFormat("%.6f" % self.currentYaw)

        self.height_progress_bar.setValue(self.currentHeight*100)
        self.height_progress_bar_label.setText("%.6f" % self.currentHeight)

####################################

    def on_enable_check(self,value):        
        print "Toggle the enabling checkbox - current state is ",value
        self.yaw_slider.setEnabled(self.enable_checkbox.isChecked())
        self.roll_slider.setEnabled(self.enable_checkbox.isChecked())
        self.pitch_slider.setEnabled(self.enable_checkbox.isChecked())
        self.forward_slider.setEnabled(self.enable_checkbox.isChecked())
        self.lateral_slider.setEnabled(self.enable_checkbox.isChecked())
        self.height_slider.setEnabled(self.enable_checkbox.isChecked())

    def on_yawSliderMoved(self, value):
        self.yaw_position = (value/100.0)*math.pi/4.0
        #print "New yaw=",self.yaw_position
        self.yaw_label.setText("Yaw: "+str(self.yaw_position))

        if (self.enable_checkbox.isChecked()):
            self.publishRobotPelvisPose()

    def on_rollSliderMoved(self, value):
        self.roll_position = (value/100.0)*math.pi/6.0
        self.roll_label.setText("Roll: "+str(self.roll_position))

        if (self.enable_checkbox.isChecked()):
            self.publishRobotPelvisPose()

    def on_pitchSliderMoved(self, value):
        self.pitch_position = (value/100.0)*math.pi/6.0
        self.pitch_label.setText("Pitch: "+str(self.pitch_position))

        if (self.enable_checkbox.isChecked()):
            self.publishRobotPelvisPose()

    def on_forwardSliderMoved(self, value):
        self.forward_position = (value/100.0)*0.075
        #print "New forward=",self.forward_position
        self.forward_label.setText("Forward: "+str(self.forward_position))

        if (self.enable_checkbox.isChecked()):
            self.publishRobotPelvisPose()

    def on_lateralSliderMoved(self, value):
        self.lateral_position = (value/100.0)*0.15
        #print "New lateral=",self.lateral_position
        self.lateral_label.setText("Lateral: "+str(self.lateral_position))

        if (self.enable_checkbox.isChecked()):
            self.publishRobotPelvisPose()

    def on_heightSliderMoved(self, value):
        self.height_position = (value/100.0)
        #print "New height=",self.height_position
        self.height_label.setText("Height: "+str(self.height_position))

        if (self.enable_checkbox.isChecked()):
            self.publishRobotPelvisPose()
