import roslib
roslib.load_manifest('vigir_rqt_head_control')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QCheckBox
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_msgs.msg import Float64

class HeadControlDialog(Plugin):

    def __init__(self, context):
        super(HeadControlDialog, self).__init__(context)
        self.setObjectName('HeadControlDialog')

        self.head_max_pitch_deg =  65.0
        self.head_min_pitch_deg = -60.0
        self.head_pitch_range_deg = self.head_max_pitch_deg - self.head_min_pitch_deg
        self.scan_period_secs   = 20.0
        self.head_pitch_cmd_deg =  0.0
        self.scan_offset        = rospy.get_rostime()
        self.last_publish_time  = self.scan_offset
        self._timer = None

        self._widget = QWidget()
        vbox = QVBoxLayout()

        spindle_speed_hbox = QHBoxLayout()

        #Add input for setting the spindle speed
        spindle_label = QLabel("Spindle Speed [deg/s]")
        spindle_speed_hbox.addWidget(spindle_label)

        #Add a spinbox for setting the spindle speed
        spindle_speed_spinbox = QDoubleSpinBox()
        spindle_speed_spinbox.setRange(-360 * 4, 360 * 4)
        spindle_speed_spinbox.valueChanged.connect(self.handle_spindle_speed)
        self.spindle_speed_pub = rospy.Publisher('/multisense/set_spindle_speed', Float64, queue_size=10)
        spindle_speed_hbox.addWidget(spindle_speed_spinbox)

        vbox.addLayout(spindle_speed_hbox)

        #Add input for directly setting the head pitch
        head_pitch_hbox = QHBoxLayout()

        head_pitch_label = QLabel("Head Pitch [deg]")
        head_pitch_hbox.addWidget(head_pitch_label)

        #Add a spinbox for setting the head pitch directly
        self.head_pitch_spinbox = QDoubleSpinBox()
        self.head_pitch_spinbox.setRange(self.head_min_pitch_deg, self.head_max_pitch_deg)
        self.head_pitch_spinbox.valueChanged.connect(self.handle_head_pitch)
        self.head_pitch_pub = rospy.Publisher('/atlas/pos_cmd/neck_ry', Float64, queue_size=10)
        head_pitch_hbox.addWidget(self.head_pitch_spinbox)

        vbox.addLayout(head_pitch_hbox)

        #Publisher for head trajectory
        self.head_trajectory_pub = rospy.Publisher('/trajectory_controllers/neck_traj_controller/command', JointTrajectory, queue_size=10)

        #Add checkbox for invoking scanning behavior
        self.head_scan_chkbox = QCheckBox("Scanning behavior")
        self.head_scan_chkbox.stateChanged.connect(self.handle_scan_chg)
        vbox.addWidget(self.head_scan_chkbox)

        #Add input for setting the minimum head pitch
        head_min_pitch_hbox = QHBoxLayout()
        head_min_pitch_label = QLabel("Min Head Pitch [deg] (raise head)")
        head_min_pitch_hbox.addWidget(head_min_pitch_label)
        head_min_pitch_spinbox = QDoubleSpinBox()
        head_min_pitch_spinbox.setRange(self.head_min_pitch_deg, self.head_max_pitch_deg)
        head_min_pitch_spinbox.setValue(self.head_min_pitch_deg)
        head_min_pitch_spinbox.valueChanged.connect(self.handle_head_min_pitch)
        head_min_pitch_hbox.addWidget(head_min_pitch_spinbox)
        vbox.addLayout(head_min_pitch_hbox)

        #Add input for setting the maximum head pitch
        head_max_pitch_hbox = QHBoxLayout()
        head_max_pitch_label = QLabel("Max Head Pitch [deg] (lower head)")
        head_max_pitch_hbox.addWidget(head_max_pitch_label)
        head_max_pitch_spinbox = QDoubleSpinBox()
        head_max_pitch_spinbox.setRange(self.head_min_pitch_deg, self.head_max_pitch_deg)
        head_max_pitch_spinbox.setValue(self.head_max_pitch_deg)
        head_max_pitch_spinbox.valueChanged.connect(self.handle_head_max_pitch)
        head_max_pitch_hbox.addWidget(head_max_pitch_spinbox)
        vbox.addLayout(head_max_pitch_hbox)

        #Add input for setting the scan period
        head_period_hbox = QHBoxLayout()
        head_period_label = QLabel("Scanning Period [secs]")
        head_period_hbox.addWidget(head_period_label)
        head_period_spinbox = QDoubleSpinBox()
        head_period_spinbox.setRange(0.01, 60.0)
        head_period_spinbox.setValue(self.scan_period_secs)
        head_period_spinbox.valueChanged.connect(self.handle_head_period)
        head_period_hbox.addWidget(head_period_spinbox)
        vbox.addLayout(head_period_hbox)

        #add stretch at end so all GUI elements are at top of dialog
        vbox.addStretch(1)

        self._widget.setLayout(vbox)

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        print "Shutdown ..."
        if self._timer is not None:
            self._timer.shutdown()
            rospy.sleep(0.1)
        self.spindle_speed_pub.unregister()
        self.head_pitch_pub.unregister()
        self.head_trajectory_pub.unregister()
        print "Done!"

    #Slot for setting the spindle speed
    def handle_spindle_speed(self, degree_per_sec):
        self.spindle_speed_pub.publish(data=math.radians(degree_per_sec))

    #Slot for setting the head pitch state
    def handle_head_pitch(self, pitch_degree):
        self.head_pitch_cmd_deg = pitch_degree
        self.head_pitch_pub.publish(data=math.radians(pitch_degree))
        #Publish neck trajectory
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = ['neck_ry']
        trajectory.points = [JointTrajectoryPoint()]
        trajectory.points[0].positions = [math.radians(pitch_degree)]
        trajectory.points[0].velocities = [0.0]
        trajectory.points[0].time_from_start = rospy.Duration(0.75)
        self.head_trajectory_pub.publish(trajectory)

    #Slot for setting the head max pitch state
    def handle_head_max_pitch(self, value):
        self.head_max_pitch_deg = value
        self.head_pitch_range_deg = self.head_max_pitch_deg - self.head_min_pitch_deg
        if (self.head_pitch_range_deg < 0.0001):
            print 'invalid pitch limits - fix range!'
            self.head_pitch_range_deg = 0.01

    #Slot for setting the head min pitch state
    def handle_head_min_pitch(self, value):
        self.head_min_pitch_deg = value
        self.head_pitch_range_deg = self.head_max_pitch_deg - self.head_min_pitch_deg
        if (self.head_pitch_range_deg < 0.0001):
            print 'invalid pitch limits - fix range!'
            self.head_pitch_range_deg = 0.01

    def handle_head_period(self, value):
        self.scan_period_secs = value

    @Slot(bool)
    def handle_scan_chg(self, checked):
        ros_time = rospy.get_rostime()
        if checked:
            print 'Start scanning ...'
            if ((self.head_pitch_cmd_deg >= self.head_min_pitch_deg) and (self.head_pitch_cmd_deg <= self.head_max_pitch_deg)):
                # adjust offset so that scanning command starts from current angle
                # to provide bumpless transfer

                # cos value corresponding to the current head pitch command
                tmp = ((self.head_pitch_cmd_deg - self.head_min_pitch_deg) / self.head_pitch_range_deg - 0.5) * 2.0

                # angle to give the same cosine value as current command
                tmp_rad = math.acos(tmp)
                tmp_time = tmp_rad * self.scan_period_secs / (math.pi * 2.0)

                #Use integer math to avoid issues with UTC time having large numbers
                self.scan_offset.secs = int(math.floor(tmp_time))
                self.scan_offset.nsecs = int(math.floor((tmp_time - self.scan_offset.secs) * 1000000000))
                self.scan_offset.secs -= ros_time.secs
                self.scan_offset.nsecs -= ros_time.nsecs
                if (self.scan_offset.nsecs < 0):
                    self.scan_offset.secs -= 1
                    self.scan_offset.nsecs += 1000000000

            else:
                print 'outside of range - ignore offset '
                self.scan_offset = ros_time

            self._timer = rospy.Timer(rospy.Duration(0.01), self.time_callback)

        else:
            print 'Stop scanning!'
            if self._timer is not None:
                self._timer.shutdown()

    #Update the time
    def time_callback(self, timer_event):
        ros_time = rospy.get_rostime()
        # Test and only publish at 100Hz so often to avoid spamming at 1khz
        delta_time = ros_time - self.last_publish_time
        if (delta_time.secs > 0) or (delta_time.nsecs > 10000000):
            self.last_publish_time = ros_time
            float_time = float(ros_time.secs + self.scan_offset.secs) + float(ros_time.nsecs+ self.scan_offset.nsecs) * 1e-9
            rad = (float_time) * (math.pi * 2.0 / self.scan_period_secs)
            cos_rad = math.cos(rad)
            frac_range = 0.5 * cos_rad + 0.5

            tmp_deg = frac_range * self.head_pitch_range_deg + self.head_min_pitch_deg

            # Set value and publish inside spinbox callback
            self.head_pitch_spinbox.setValue(tmp_deg)
