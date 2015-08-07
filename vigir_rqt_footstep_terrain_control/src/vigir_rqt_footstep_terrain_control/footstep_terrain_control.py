import roslib
roslib.load_manifest('vigir_rqt_footstep_terrain_control')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QCheckBox, QComboBox, QTextEdit, QGroupBox, QPushButton, QLineEdit

from std_msgs.msg import Float64
from flor_footstep_planner_msgs.msg import FootstepPlannerParams
from flor_terrain_classifier.msg import TerrainModelRequest
from vigir_perception_msgs.msg import EnvironmentRegionRequest

class FootstepTerrainControlDialog(Plugin):

    def __init__(self, context):
        super(FootstepTerrainControlDialog, self).__init__(context)
        self.setObjectName('FootstepTerrainControlDialog')
       
        self.request_seq = 0 
        self._widget = QWidget()
        vbox = QVBoxLayout()
        
        self.cube_vbox = QVBoxLayout()
        
        self.cube_groupbox = QGroupBox( "Terrain Box" )

        self.cube_min_hbox = QHBoxLayout()

        self.cube_min_label = QLabel("Bottom Left")
        self.cube_min_hbox.addWidget( self.cube_min_label )

        self.cube_min_x = QDoubleSpinBox()
        self.cube_min_x.setSingleStep(.01)
        self.cube_min_x.setRange(-10.0, 10.0)
        self.cube_min_x.setValue(-3.0)
        self.cube_min_hbox.addWidget(self.cube_min_x)

        self.cube_min_y = QDoubleSpinBox()
        self.cube_min_y.setSingleStep(.01)
        self.cube_min_y.setRange(-10.0, 10.0)
        self.cube_min_y.setValue(-3.0)
        self.cube_min_hbox.addWidget(self.cube_min_y)

        self.cube_min_z = QDoubleSpinBox()
        self.cube_min_z.setSingleStep(.01)
        self.cube_min_z.setRange(-10.0, 10.0)
        self.cube_min_z.setValue(-1.0)
        self.cube_min_hbox.addWidget(self.cube_min_z)

        self.cube_vbox.addLayout( self.cube_min_hbox )
        
        self.cube_max_hbox = QHBoxLayout()

        self.cube_max_label = QLabel("Top Right")
        self.cube_max_hbox.addWidget( self.cube_max_label )

        self.cube_max_x = QDoubleSpinBox()
        self.cube_max_x.setSingleStep(.01)
        self.cube_max_x.setRange(-10.0, 10.0)
        self.cube_max_x.setValue(3.0)
        self.cube_max_hbox.addWidget(self.cube_max_x)

        self.cube_max_y = QDoubleSpinBox()
        self.cube_max_y.setSingleStep(.01)
        self.cube_max_y.setRange(-10.0, 10.0)
        self.cube_max_y.setValue(3.0)
        self.cube_max_hbox.addWidget(self.cube_max_y)

        self.cube_max_z = QDoubleSpinBox()
        self.cube_max_z.setSingleStep(.01)
        self.cube_max_z.setRange(-10.0, 10.0)
        self.cube_max_z.setValue(1.0)
        self.cube_max_hbox.addWidget(self.cube_max_z)

        self.cube_vbox.addLayout( self.cube_max_hbox )
    
        self.cube_groupbox.setLayout( self.cube_vbox )
        vbox.addWidget( self.cube_groupbox )

        self.type_hbox = QHBoxLayout()
  
#        self.type_label = QLabel( "Type" )
#        self.type_hbox.addWidget( self.type_label )
#
#        self.type_combobox = QComboBox()
#
#        self.type_combobox.addItem( "Type 1" )
#        self.type_combobox.addItem( "Type 2" )       
#        self.type_combobox.addItem( "Type 3" ) 
#        
#        self.type_hbox.addWidget( self.type_combobox )
#       
#        vbox.addLayout( self.type_hbox )

        self.scan_number_hbox = QHBoxLayout()
        
        self.scan_number_label = QLabel("Number of Scans Used")
        self.scan_number_hbox.addWidget( self.scan_number_label )

        self.scan_number_val = QDoubleSpinBox()
        self.scan_number_val.setDecimals(0)
        self.scan_number_val.setRange(0,10000)
        self.scan_number_val.setValue(2000)
        self.scan_number_hbox.addWidget(self.scan_number_val)

        vbox.addLayout( self.scan_number_hbox )

#       self.use_terrain_checkbox = QCheckBox( "Use Terrain" )
#       vbox.addWidget( self.use_terrain_checkbox )


        button_hbox = QHBoxLayout()
        
#        button_get = QPushButton("Get Current Values")
#        button_hbox.addWidget( button_get )

        button_submit = QPushButton("Send Values")
        button_hbox.addWidget( button_submit)

        vbox.addLayout( button_hbox )

        vbox.addStretch(1)

        self._widget.setLayout(vbox)

        context.add_widget(self._widget)

        # publishers and subscribers
        self.terrain_pub = rospy.Publisher("/flor/terrain_classifier/generate_terrain_model", TerrainModelRequest, queue_size=10 )
        button_submit.pressed.connect(self.sendParams)

#        self.terrain_sub = self.stateSubscriber   = rospy.Subscriber('/ros_footstep_planner/terrains', FootstepPlannerParams, self.getParamCallbackFcn)
#        button_submit.pressed.connect(self.getParams)


    def shutdown_plugin(self):
        print "Shutdown ..." 
        print "Done!"
    
#    def getParams( self ):
#        msg = FootstepPlannerParams()
#
#        
#        self.terrain_pub.publish( msg ) 

    def sendParams( self ):
        msg = TerrainModelRequest()

        msg.use_default_region_request = False

        rr = EnvironmentRegionRequest()

        rr.header.seq = self.request_seq
        self.request_seq = self.request_seq+1

        rr.header.stamp = rospy.get_rostime()
        rr.header.frame_id = "/world"

        rr.bounding_box_min.x = self.cube_min_x.value()
        rr.bounding_box_min.y = self.cube_min_y.value()
        rr.bounding_box_min.z = self.cube_min_z.value()
        
        rr.bounding_box_max.x = self.cube_max_x.value()
        rr.bounding_box_max.y = self.cube_max_y.value()
        rr.bounding_box_max.z = self.cube_max_z.value()

        rr.resolution = 0
        rr.request_augment = 0

        msg.region_req = rr

        msg.aggregation_size = self.scan_number_val.value() 

        print msg
        self.terrain_pub.publish( msg ) 


    def getParamCallbackFcn(self, msg):
        print "Not Defined Yet"  
