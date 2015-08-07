import roslib
roslib.load_manifest('vigir_rqt_head_control')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QFrame, QCheckBox

from sensor_msgs.msg import JointState
from atlas_msgs.msg  import AtlasSimInterfaceState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Joint(object):
    def __init__(self, name, sensor_bias, control_bias, gearing_bias, controller):
        self.name             = name
        self.sensor_bias      = sensor_bias
        self.control_bias     = control_bias
        self.controller       = controller
        self.gearing_bias     = gearing_bias

        print "Initial data for joint ",self.name, " biases=(",self.sensor_bias,", ",self.control_bias,")"
        self.max_sensor_bias  =  3.5
        self.min_sensor_bias  = -3.5
        self.range_sensor_bias = self.max_sensor_bias - self.min_sensor_bias
        self.max_control_bias =  0.3
        self.min_control_bias = -0.3
        self.max_gearing_bias =  1.3
        self.min_gearing_bias =  0.7
        self.range_gearing_bias = self.max_gearing_bias - self.min_gearing_bias

        self.range_control_bias = self.max_control_bias - self.min_control_bias
        self.sensor_bias_spinbox = QDoubleSpinBox()
        self.sensor_bias_spinbox.setDecimals(4);
        self.sensor_bias_spinbox.setRange(self.min_sensor_bias, self.max_sensor_bias)
        self.sensor_bias_spinbox.setValue(self.sensor_bias)
        self.sensor_bias_spinbox.setSingleStep(self.range_sensor_bias/500)
        self.sensor_bias_spinbox.valueChanged.connect(self.on_biasChanged)

        self.control_bias_spinbox = QDoubleSpinBox()
        self.control_bias_spinbox.setDecimals(4);
        self.control_bias_spinbox.setRange(self.min_control_bias, self.max_control_bias)
        self.control_bias_spinbox.setSingleStep(self.range_control_bias/50)
        self.control_bias_spinbox.setValue(self.control_bias)
        self.control_bias_spinbox.valueChanged.connect(self.on_biasChanged)

        self.gearing_bias_spinbox = QDoubleSpinBox()
        self.gearing_bias_spinbox.setDecimals(4);
        self.gearing_bias_spinbox.setRange(self.min_gearing_bias, self.max_gearing_bias)
        self.gearing_bias_spinbox.setSingleStep(self.range_gearing_bias/50)
        self.gearing_bias_spinbox.setValue(self.gearing_bias)
        self.gearing_bias_spinbox.valueChanged.connect(self.on_biasChanged)

    def on_biasChanged(self, value):

        # grab both if either change
        self.sensor_bias  = self.sensor_bias_spinbox.value()
        self.control_bias = self.control_bias_spinbox.value()
        self.gearing_bias = self.gearing_bias_spinbox.value()

        joint_state = JointState()
        joint_state.name = [self.name];
        joint_state.position = [self.sensor_bias];
        joint_state.effort   = [self.control_bias];
        joint_state.velocity   = [self.gearing_bias];

        print "Set biases:\n ",joint_state
        self.controller.parent.parent.bias_pub.publish(joint_state)

    def saveData(self, saveFilePtr):
        saveFilePtr.write(","+self.name+"/"+str(self.sensor_bias)+"/"+str(self.control_bias)+"/"+str(self.gearing_bias))

class Controller(object):
    def __init__(self, parent, label, name):
        self.label    = label
        self.name     = name
        self.parent   = parent
        self.joints   = []
    def saveData(self, saveFilePtr):
        saveFilePtr.write(self.label+","+self.name)
        for joint in self.joints:
            joint.saveData(saveFilePtr)
        saveFilePtr.write("\n")

class JointBias(object):
    def __init__(self, parent, fileName, top_widget_layout):

        self.controllers = []
        self.parent      = parent

        self.loadFile(fileName)


        print "Initialize controllers..."
        for controller in self.controllers:
            frame = QFrame()
            frame.setFrameShape(QFrame.StyledPanel);
            frame.setFrameShadow(QFrame.Raised);

            vbox = QVBoxLayout()
            label = QLabel()
            label.setText(controller.label)
            vbox.addWidget(label);

            print controller.name

            for joint in controller.joints:
                label = QLabel()
                label.setText(joint.name)
                vbox.addWidget(label);

                #Add input for setting the biases
                widget = QWidget()
                hbox = QHBoxLayout()

                hbox.addWidget(joint.sensor_bias_spinbox)
                hbox.addWidget(joint.control_bias_spinbox)
                hbox.addWidget(joint.gearing_bias_spinbox)

                widget.setLayout(hbox)
                vbox.addWidget(widget)

            label = QLabel()
            label.setText("      Sensor           Control           Gearing")
            vbox.addWidget(label);
            vbox.addStretch()

            frame.setLayout(vbox)
            top_widget_layout.addWidget(frame)
        print "Done loading controllers"

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
                        for joint_data in splitLine[2:]:
                            splitData = joint_data.split('/')
                            joint_name   = splitData[0]
                            sensor_bias  = float(splitData[1])
                            control_bias = float(splitData[2])
                            gearing_bias = float(splitData[3])
                            self.controllers[-1].joints.append(Joint(joint_name, sensor_bias, control_bias, gearing_bias, self.controllers[-1]))
                else:
                    break
            print "Finished loading file!"
        else:
            print "Load cancelled."
            sys.exit(-1)
    def saveData(self, fileName):
        if fileName[0]:
            print "Saving ",fileName
            saveFilePtr = open(fileName, 'w')
            if (saveFilePtr):
                saveFilePtr.write("#Group, joint_0/sensor_bias/control_bias,joint_1/sensor_bias/control_bias, ...\n")
                for controller in self.controllers:
                    controller.saveData(saveFilePtr)

class BiasCalibrationDialog(Plugin):

    def __init__(self, context):
        super(BiasCalibrationDialog, self).__init__(context)
        self.setObjectName('BiasCalibrationDialog')

        self._widget = QWidget()
        vbox = QVBoxLayout()
        controller_widget = QWidget()

        hbox = QHBoxLayout()

        # Left to right layout
        self.joint_control = JointBias(self, roslib.packages.get_pkg_dir('vigir_rqt_bias_calibration') + '/launch/joints.txt',hbox)

        controller_widget.setLayout(hbox)

        self.marker = Marker()
        self.marker.header.frame_id = "/world"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 14.0
        self.marker.scale.y = 14.0
        self.marker.scale.z = 0.02
        self.marker.color.a = 0.25
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        #self.marker.points  = [[0.0, 0.0, 0.0], [7.0, -4.0, 0.0], [7.0, 4.0, 0.0]]
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0

        self.bias_pub  = rospy.Publisher('/flor/controller/atlas_biases', JointState, queue_size=10)
        self.flor_marker_pub  = rospy.Publisher('/flor_plane_marker', Marker, queue_size=10)
        self.feet_state_sub = rospy.Subscriber('/atlas/atlas_sim_interface_state', AtlasSimInterfaceState, self.stateCallbackFnc)

        vbox.addWidget(controller_widget)

        self.save_button = QPushButton("Save Biases")
        self.save_button.pressed.connect(self.on_savePressed)
        vbox.addWidget(self.save_button)

        self._widget.setLayout(vbox)

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        print "Shutdown ..."
        self.bias_pub.unregister()
        self.flor_marker_pub.unregister()
        self.feet_state_sub.unregister()
        print "Done!"

    def stateCallbackFnc(self, atlasState_msg):
        self.marker.pose.position.x = (atlasState_msg.foot_pos_est[0].position.x + atlasState_msg.foot_pos_est[1].position.x)*0.5
        self.marker.pose.position.y = (atlasState_msg.foot_pos_est[0].position.y + atlasState_msg.foot_pos_est[1].position.y)*0.5
        self.marker.pose.position.z = (atlasState_msg.foot_pos_est[0].position.z + atlasState_msg.foot_pos_est[1].position.z)*0.5
        #self.marker.pose.orientation = atlasState_msg.foot_pos_est[0].orientation
        self.flor_marker_pub.publish(self.marker)

    def on_savePressed(self):
        print "Save data to file..."
        self.joint_control.saveData(roslib.packages.get_pkg_dir('vigir_rqt_bias_calibration') + '/launch/new_biases.txt')
