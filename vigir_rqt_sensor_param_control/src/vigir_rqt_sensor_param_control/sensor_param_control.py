import roslib
roslib.load_manifest('vigir_rqt_sensor_param_control')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QCheckBox, QComboBox, QTextEdit, QGroupBox, QPushButton, QSlider

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension

class SensorParamControlDialog(Plugin):

    def __init__(self, context):
        super(SensorParamControlDialog, self).__init__(context)
        self.setObjectName('SensorParamControlDialog')
        
        self._widget = QWidget()
        vbox = QVBoxLayout()


        ### Multisense ###

        ms_groupbox = QGroupBox( "Multisense" )
        
        ms_vbox = QVBoxLayout()
        
        ms_gain_hbox = QHBoxLayout()
       
        self.ms_gain_label = QLabel("Image Gain [1, 1, 8]")
        ms_gain_hbox.addWidget( self.ms_gain_label )
        
        self.ms_gain = QDoubleSpinBox()
        self.ms_gain.setSingleStep(.01)
        self.ms_gain.setRange(1,8)

        ms_gain_hbox.addWidget( self.ms_gain ) 
    
        ms_vbox.addLayout( ms_gain_hbox )       

        
        ms_exp_hbox = QHBoxLayout()
        
        self.ms_exp_auto = QCheckBox("Image Exposure [.03, 0.5]")
        ms_exp_hbox.addWidget( self.ms_exp_auto )
        
        self.ms_exp = QDoubleSpinBox()
        self.ms_exp.setSingleStep( .001 )
        self.ms_exp.setRange( .025,.5 ) 

        ms_exp_hbox.addWidget( self.ms_exp ) 
    
        ms_vbox.addLayout( ms_exp_hbox )       
        
        ms_spindle_hbox = QHBoxLayout()
        
        ms_spindle_label = QLabel("Spindle Speed [0, 5.2]")        
        
        ms_spindle_hbox.addWidget( ms_spindle_label )

        self.ms_spindle = QDoubleSpinBox()
        self.ms_spindle.setSingleStep(.01)
        self.ms_spindle.setRange( 0,15.2 )

        ms_spindle_hbox.addWidget( self.ms_spindle ) 
    
        ms_vbox.addLayout( ms_spindle_hbox )       

        ms_light_hbox = QHBoxLayout()
        
        ms_light_label = QLabel("Light Brightness")
        ms_light_hbox.addWidget(ms_light_label)

        self.ms_light = QSlider(Qt.Horizontal)
        self.ms_light.setRange(0,100)
    
        ms_light_hbox.addWidget( self.ms_light )

        ms_vbox.addLayout( ms_light_hbox )

        ms_button_hbox = QHBoxLayout()

        ms_button_hbox.addStretch(1)        

        ms_button_get = QPushButton("Get Settings")
        ms_button_get.pressed.connect(self.ms_get_callback)
        #ms_button_hbox.addWidget( ms_button_get )
        
        ms_button_set = QPushButton("Set Settings")
        ms_button_set.pressed.connect(self.ms_set_callback)
        ms_button_hbox.addWidget( ms_button_set )

        ms_vbox.addLayout( ms_button_hbox ) 

        ms_groupbox.setLayout( ms_vbox )

        vbox.addWidget( ms_groupbox )


        ### Left SA ###

        sa_left_groupbox = QGroupBox( "Left SA Camera" )
        
        sa_left_vbox = QVBoxLayout()
        
        sa_left_gain_hbox = QHBoxLayout()
        
        sa_left_gain_label = QLabel("Image Gain [0, 0, 25]")        
        
        sa_left_gain_hbox.addWidget( sa_left_gain_label )

        self.sa_left_gain = QDoubleSpinBox()
        self.sa_left_gain.setSingleStep(.01)
        self.sa_left_gain.setRange( 0, 25 )

        sa_left_gain_hbox.addWidget( self.sa_left_gain ) 
    
        sa_left_vbox.addLayout( sa_left_gain_hbox )       

        
        sa_left_exp_hbox = QHBoxLayout()
        
        sa_left_exp_label = QLabel("Image Shutter [0, 5.5, 25]")        
        
        sa_left_exp_hbox.addWidget( sa_left_exp_label )

        self.sa_left_exp = QDoubleSpinBox()
        self.sa_left_exp.setSingleStep(.01)
        self.sa_left_exp.setRange( 0, 25 )

        sa_left_exp_hbox.addWidget( self.sa_left_exp ) 
    
        sa_left_vbox.addLayout( sa_left_exp_hbox )       
        
        sa_left_button_hbox = QHBoxLayout()

        sa_left_button_hbox.addStretch(1)        

        sa_left_button_get = QPushButton("Get Settings")
        sa_left_button_get.pressed.connect(self.sa_left_get_callback)
        #sa_left_button_hbox.addWidget( sa_left_button_get )
        
        sa_left_button_set = QPushButton("Set Settings")
        sa_left_button_set.pressed.connect(self.sa_left_set_callback)
        sa_left_button_hbox.addWidget( sa_left_button_set )

        sa_left_vbox.addLayout( sa_left_button_hbox ) 

        sa_left_groupbox.setLayout( sa_left_vbox )

        vbox.addWidget(sa_left_groupbox)
        
        ### Right SA ###

        sa_right_groupbox = QGroupBox( "Right SA Camera" )
        
        sa_right_vbox = QVBoxLayout()
        
        sa_right_gain_hbox = QHBoxLayout()
        
        sa_right_gain_label = QLabel("Image Gain [0, 0, 25]")        
        
        sa_right_gain_hbox.addWidget( sa_right_gain_label )

        self.sa_right_gain = QDoubleSpinBox()
        self.sa_right_gain.setSingleStep(.01)
        self.sa_right_gain.setRange( 0, 25 )

        sa_right_gain_hbox.addWidget( self.sa_right_gain ) 
    
        sa_right_vbox.addLayout( sa_right_gain_hbox )       

        
        sa_right_exp_hbox = QHBoxLayout()
        
        sa_right_exp_label = QLabel("Image Shutter [0, 5.5, 25]")        
        
        sa_right_exp_hbox.addWidget( sa_right_exp_label )

        self.sa_right_exp = QDoubleSpinBox()
        self.sa_right_exp.setSingleStep(.01)
        self.sa_right_exp.setRange( 0, 25 )    
    
        sa_right_exp_hbox.addWidget( self.sa_right_exp ) 
    
        sa_right_vbox.addLayout( sa_right_exp_hbox )       
        
        sa_right_button_hbox = QHBoxLayout()

        sa_right_button_hbox.addStretch(1)        

        sa_right_button_get = QPushButton("Get Settings")
        sa_right_button_get.pressed.connect(self.sa_right_get_callback)
        #sa_right_button_hbox.addWidget( sa_right_button_get )
        
        sa_right_button_set = QPushButton("Set Settings")
        sa_right_button_set.pressed.connect(self.sa_right_set_callback)
        sa_right_button_hbox.addWidget( sa_right_button_set )

        sa_right_vbox.addLayout( sa_right_button_hbox ) 

        sa_right_groupbox.setLayout( sa_right_vbox )

        vbox.addWidget(sa_right_groupbox)

        vbox.addStretch(1)

        self._widget.setLayout(vbox)

        context.add_widget(self._widget)

        # publishers and subscribers

        self.ms_cam_pub = rospy.Publisher( '/multisense_sl/set_cam_parameters', Float64MultiArray, queue_size=10)
        self.ms_light_pub = rospy.Publisher( '/multisense_sl/set_light_brightness', Float64, queue_size=10)
        self.ms_spindle_pub = rospy.Publisher( '/multisense_sl/set_spindle_speed', Float64, queue_size=10)

        self.sa_left_cam_pub = rospy.Publisher( '/sa/left/set_cam_parameters', Float64MultiArray, queue_size=10)
        self.sa_right_cam_pub = rospy.Publisher( '/sa/right/set_cam_parameters', Float64MultiArray, queue_size=10)


 #       self.param_sub = self.stateSubscriber   = rospy.Subscriber('/ros_footstep_planner/params', FootstepPlannerParams, self.getParamCallbackFcn)


    def shutdown_plugin(self):
        print "Shutdown ..." 
#        self.param_pub.unregister()
        print "Done!"

    def ms_get_callback(self):
        print "fill values"

    def ms_set_callback(self):
        ms_cam = Float64MultiArray()
        
        ms_cam_dim_h = MultiArrayDimension()
        ms_cam_dim_h.label = "height"
        ms_cam_dim_h.size = 1
        ms_cam_dim_h.stride  = 2        

        ms_cam.layout.dim.append( ms_cam_dim_h )
        
        ms_cam_dim_w = MultiArrayDimension()
        ms_cam_dim_w.label = "width"
        ms_cam_dim_w.size = 2
        ms_cam_dim_w.stride  = 2        
        
        ms_cam.layout.dim.append(ms_cam_dim_w)

        ms_cam.data.append( self.ms_gain.value() )

        if self.ms_exp_auto.isChecked() :
            ms_cam.data.append( 1 )   
        else:
            ms_cam.data.append( self.ms_exp.value() )
        
        self.ms_cam_pub.publish( ms_cam )
    

        light  = Float64()
        light.data = self.ms_light.value()/100.0

        self.ms_light_pub.publish( light )

        spindle_speed = Float64()
        spindle_speed.data = self.ms_spindle.value()

        self.ms_spindle_pub.publish( spindle_speed )
        
    def sa_left_get_callback(self):
        print "fill values"

    def sa_left_set_callback(self):
        sa_left_cam = Float64MultiArray()
        
        sa_left_cam_dim_h = MultiArrayDimension()
        sa_left_cam_dim_h.label = "height"
        sa_left_cam_dim_h.size = 1
        sa_left_cam_dim_h.stride  = 2        

        sa_left_cam.layout.dim.append( sa_left_cam_dim_h )
        
        sa_left_cam_dim_w = MultiArrayDimension()
        sa_left_cam_dim_w.label = "width"
        sa_left_cam_dim_w.size = 2
        sa_left_cam_dim_w.stride  = 2        
        
        sa_left_cam.layout.dim.append(sa_left_cam_dim_w)

        sa_left_cam.data.append( self.sa_left_gain.value() )
        sa_left_cam.data.append( self.sa_left_exp.value() )
        
        self.sa_left_cam_pub.publish( sa_left_cam )
    
    def sa_right_get_callback(self):
        print "fill values"

    def sa_right_set_callback(self):
        sa_right_cam = Float64MultiArray()
        
        sa_right_cam_dim_h = MultiArrayDimension()
        sa_right_cam_dim_h.label = "height"
        sa_right_cam_dim_h.size = 1
        sa_right_cam_dim_h.stride  = 2        

        sa_right_cam.layout.dim.append( sa_right_cam_dim_h )
        
        sa_right_cam_dim_w = MultiArrayDimension()
        sa_right_cam_dim_w.label = "width"
        sa_right_cam_dim_w.size = 2
        sa_right_cam_dim_w.stride  = 2        
        
        sa_right_cam.layout.dim.append(sa_right_cam_dim_w)

        sa_right_cam.data.append( self.sa_right_gain.value() )
        sa_right_cam.data.append( self.sa_right_exp.value() )
        
        self.sa_right_cam_pub.publish( sa_right_cam )
