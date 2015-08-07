import roslib
roslib.load_manifest('vigir_rqt_footstep_param_control')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, QObject
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QCheckBox, QComboBox, QTextEdit, QGroupBox, QPushButton

from std_msgs.msg import Float64
from flor_footstep_planner_msgs.msg import FootstepPlannerParams

class FootstepParamControlWidget(QObject):

    def __init__(self, context):
        #super(FootstepParamControlDialog, self).__init__(context)
        #self.setObjectName('FootstepParamControlDialog')
        super(FootstepParamControlWidget, self).__init__()
        self.name = 'FootstepParamControlWidget'
        
        #self._widget = QWidget()
        self._widget = context
        vbox = QVBoxLayout()

        ### STEP_COST_ESTIMATOR ########
        self.step_cost_vbox = QVBoxLayout()
   
        self.step_cost_groupbox = QGroupBox( "Step Cost Estimator" )
        self.step_cost_groupbox.setCheckable( True )
        self.step_cost_groupbox.setChecked(False)

        self.step_cost_combobox = QComboBox()

        self.step_cost_combobox.addItem( "Euclidean" )
        self.step_cost_combobox.addItem( "GPR" )       
        self.step_cost_combobox.addItem( "Map" ) 
        self.step_cost_combobox.addItem( "Boundary" )
        self.step_cost_combobox.addItem( "Dynamics" )
        
        self.step_cost_vbox.addWidget( self.step_cost_combobox )
       
        self.step_cost_groupbox.setLayout( self.step_cost_vbox )
 
        vbox.addWidget( self.step_cost_groupbox )

# #HARD       ### FOOTSTEP_SET ########
#        # parameters for discret footstep planning mode
#        vigir_atlas_control_msgs/VigirBehaviorStepData[] footstep_set            # set of footsteps (displacement vectors (in meter / rad))
#        float32[] footstep_cost                                         # cost for each footstep given in footstep set
#
# #HARD       ### LOAD_GPR_STEP_COST ########
#        # map step cost file
#        std_msgs/String map_step_cost_file
#
# #HARD       ### LOAD_MAP_STEP_COST ########
#        # destination of gpr file
#        std_msgs/String gpr_step_cost_file

        ### COLLISION_CHECK_TYPE ########
        self.collision_check_groupbox = QGroupBox( "Collision Check Type" )
        self.collision_check_groupbox.setCheckable( True )
        self.collision_check_groupbox.setChecked( False)

        self.collision_check_vbox = QVBoxLayout()
   
        self.collision_check_feet_checkbox = QCheckBox( "Feet" )      
        self.collision_check_vbox.addWidget( self.collision_check_feet_checkbox  )
        
        self.collision_check_ub_checkbox = QCheckBox( "Upper Body" )
        self.collision_check_vbox.addWidget( self.collision_check_ub_checkbox  )
        
        self.collision_check_fc_checkbox = QCheckBox( "Foot Contact Support" ) 
        self.collision_check_vbox.addWidget( self.collision_check_fc_checkbox  )

        self.collision_check_groupbox.setLayout( self.collision_check_vbox )
        
        vbox.addWidget( self.collision_check_groupbox )

        ### FOOT_SIZE ########
        self.foot_size_vbox = QVBoxLayout()
        
        self.foot_size_groupbox = QGroupBox( "Foot Size" )
        self.foot_size_groupbox.setCheckable( True )
        self.foot_size_groupbox.setChecked( False )

        self.foot_size_hbox = QHBoxLayout()

        self.foot_size_label = QLabel("Foot Size")
        self.foot_size_hbox.addWidget( self.foot_size_label )

        self.foot_size_x = QDoubleSpinBox()
        self.foot_size_x.setSingleStep(.01)
        self.foot_size_hbox.addWidget(self.foot_size_x)

        self.foot_size_y = QDoubleSpinBox()
        self.foot_size_y.setSingleStep(.01)
        self.foot_size_hbox.addWidget(self.foot_size_y)

        self.foot_size_z = QDoubleSpinBox()
        self.foot_size_z.setSingleStep(.01)
        self.foot_size_hbox.addWidget(self.foot_size_z)

        self.foot_size_vbox.addLayout( self.foot_size_hbox )
        
        self.foot_shift_hbox = QHBoxLayout()

        self.foot_shift_label = QLabel("Foot Origin Shift")
        self.foot_shift_hbox.addWidget( self.foot_shift_label )

        self.foot_shift_x = QDoubleSpinBox()
        self.foot_shift_x.setRange(-1.0, 1.0)
        self.foot_shift_x.setSingleStep(.01)
        self.foot_shift_hbox.addWidget(self.foot_shift_x)

        self.foot_shift_y = QDoubleSpinBox()
        self.foot_shift_y.setRange(-1.0, 1.0)
        self.foot_shift_y.setSingleStep(.01)
        self.foot_shift_hbox.addWidget(self.foot_shift_y)

        self.foot_shift_z = QDoubleSpinBox()
        self.foot_shift_z.setRange(-1.0, 1.0)
        self.foot_shift_z.setSingleStep(.01)
        self.foot_shift_hbox.addWidget(self.foot_shift_z)

        self.foot_size_vbox.addLayout( self.foot_shift_hbox )
    
        self.foot_size_groupbox.setLayout( self.foot_size_vbox )
        vbox.addWidget( self.foot_size_groupbox )

        ### UPPER_BODY_SIZE ########
            
        self.upper_vbox = QVBoxLayout()
   
        self.upper_groupbox = QGroupBox( "Upper Body Size" )
        self.upper_groupbox.setCheckable( True )
        self.upper_groupbox.setChecked( False )
        
        self.upper_size_hbox = QHBoxLayout()

        self.upper_size_label = QLabel("Upper Body Size")
        self.upper_size_hbox.addWidget( self.upper_size_label )

        self.upper_size_x = QDoubleSpinBox()
        self.upper_size_x.setSingleStep(.01)
        self.upper_size_hbox.addWidget(self.upper_size_x)

        self.upper_size_y = QDoubleSpinBox()
        self.upper_size_y.setSingleStep(.01)
        self.upper_size_hbox.addWidget(self.upper_size_y)

        self.upper_size_z = QDoubleSpinBox()
        self.upper_size_z.setSingleStep(.01)
        self.upper_size_hbox.addWidget(self.upper_size_z)

        self.upper_vbox.addLayout( self.upper_size_hbox )
        
        self.upper_origin_hbox = QHBoxLayout()

        self.upper_origin_label = QLabel("Upper Body Origin")
        self.upper_origin_hbox.addWidget( self.upper_origin_label )

        self.upper_origin_x = QDoubleSpinBox()
        self.upper_origin_x.setRange(-1.0, 1.0)
        self.upper_origin_x.setSingleStep(.01)
        self.upper_origin_hbox.addWidget(self.upper_origin_x)

        self.upper_origin_y = QDoubleSpinBox()
        self.upper_origin_y.setRange(-1.0, 1.0)
        self.upper_origin_y.setSingleStep(.01)
        self.upper_origin_hbox.addWidget(self.upper_origin_y)

        self.upper_origin_z = QDoubleSpinBox()
        self.upper_origin_z.setRange(-1.0, 1.0)
        self.upper_origin_z.setSingleStep(.01)
        self.upper_origin_hbox.addWidget(self.upper_origin_z)

        self.upper_vbox.addLayout( self.upper_origin_hbox )
       
        self.upper_groupbox.setLayout( self.upper_vbox ) 
        vbox.addWidget( self.upper_groupbox )
    
        ### TERRAIN_MODEL ########
        self.terrain_model_groupbox = QGroupBox( "Terrain Model" )
        self.terrain_model_groupbox.setCheckable( True )
        self.terrain_model_groupbox.setChecked( False )

        self.terrain_model_vbox = QVBoxLayout()
        self.terrain_model_checkbox = QCheckBox( "Use Terrain Model" )      
        self.terrain_model_vbox.addWidget( self.terrain_model_checkbox )
        
        self.terrain_min_ssx_hbox = QHBoxLayout()
        
        self.terrain_min_ssx_label = QLabel("Min Sampling Steps x")
        self.terrain_min_ssx_hbox.addWidget( self.terrain_min_ssx_label )

        self.terrain_min_ssx_val = QDoubleSpinBox()
        self.terrain_min_ssx_val.setSingleStep(1)
        self.terrain_min_ssx_val.setRange(0,100)
        self.terrain_min_ssx_hbox.addWidget( self.terrain_min_ssx_val )

        self.terrain_model_vbox.addLayout( self.terrain_min_ssx_hbox )
    
        self.terrain_min_ssy_hbox = QHBoxLayout()
        
        self.terrain_min_ssy_label = QLabel("Min Sampling Steps y")
        self.terrain_min_ssy_hbox.addWidget( self.terrain_min_ssy_label )

        self.terrain_min_ssy_val = QDoubleSpinBox()
        self.terrain_min_ssy_val.setSingleStep(1)
        self.terrain_min_ssy_val.setRange(0,100)
        self.terrain_min_ssy_hbox.addWidget( self.terrain_min_ssy_val )

        self.terrain_model_vbox.addLayout( self.terrain_min_ssy_hbox )
        
        self.terrain_max_ssx_hbox = QHBoxLayout()
        
        self.terrain_max_ssx_label = QLabel("Max Sampling Steps x")
        self.terrain_max_ssx_hbox.addWidget( self.terrain_max_ssx_label )

        self.terrain_max_ssx_val = QDoubleSpinBox()
        self.terrain_max_ssx_val.setSingleStep(1)
        self.terrain_max_ssx_val.setRange(0,100)
        self.terrain_max_ssx_hbox.addWidget( self.terrain_max_ssx_val )

        self.terrain_model_vbox.addLayout( self.terrain_max_ssx_hbox )
    
        self.terrain_max_ssy_hbox = QHBoxLayout()
        
        self.terrain_max_ssy_label = QLabel("Max Sampling Steps y")
        self.terrain_max_ssy_hbox.addWidget( self.terrain_max_ssy_label )

        self.terrain_max_ssy_val = QDoubleSpinBox()
        self.terrain_max_ssy_val.setSingleStep(1)
        self.terrain_max_ssy_val.setRange(0,100)
        self.terrain_max_ssy_hbox.addWidget( self.terrain_max_ssy_val )

        self.terrain_model_vbox.addLayout( self.terrain_max_ssy_hbox )

        self.terrain_max_iz_hbox = QHBoxLayout()
        
        self.terrain_max_iz_label = QLabel("Max Intrusion z")
        self.terrain_max_iz_hbox.addWidget( self.terrain_max_iz_label )

        self.terrain_max_iz_val = QDoubleSpinBox()
        self.terrain_max_iz_val.setDecimals(4)
        self.terrain_max_iz_val.setSingleStep(.0001)
        self.terrain_max_iz_val.setRange(0,.25)
        self.terrain_max_iz_hbox.addWidget( self.terrain_max_iz_val )

        self.terrain_model_vbox.addLayout( self.terrain_max_iz_hbox )

        self.terrain_max_gc_hbox = QHBoxLayout()
        
        self.terrain_max_gc_label = QLabel("Max Ground Clearance")
        self.terrain_max_gc_hbox.addWidget( self.terrain_max_gc_label )

        self.terrain_max_gc_val = QDoubleSpinBox()
        self.terrain_max_gc_val.setDecimals(4)
        self.terrain_max_gc_val.setSingleStep(.0001)
        self.terrain_max_gc_val.setRange(0,.25)
        self.terrain_max_gc_hbox.addWidget( self.terrain_max_gc_val )

        self.terrain_model_vbox.addLayout( self.terrain_max_gc_hbox )

        self.terrain_ms_hbox = QHBoxLayout()
        
        self.terrain_ms_label = QLabel("Minimal Support")
        self.terrain_ms_hbox.addWidget( self.terrain_ms_label )

        self.terrain_ms_val = QDoubleSpinBox()
        self.terrain_ms_val.setDecimals(2)
        self.terrain_ms_val.setSingleStep(.01)
        self.terrain_ms_val.setRange(0,1)
        self.terrain_ms_hbox.addWidget( self.terrain_ms_val )

        self.terrain_model_vbox.addLayout( self.terrain_ms_hbox )

        self.terrain_model_groupbox.setLayout( self.terrain_model_vbox )
        
        vbox.addWidget( self.terrain_model_groupbox ) 

        ### STANDARD_STEP_PARAMS ########

        self.std_step_vbox = QVBoxLayout()

        self.std_step_groupbox = QGroupBox( "Standard Step Parameters" )
        self.std_step_groupbox.setCheckable( True )
        self.std_step_groupbox.setChecked( False )

        self.foot_sep_hbox = QHBoxLayout()
        
        self.foot_sep_label = QLabel("Foot Separation")
        self.foot_sep_hbox.addWidget( self.foot_sep_label )

        self.foot_sep_val = QDoubleSpinBox()
        self.foot_sep_val.setSingleStep(.01)
        self.foot_sep_hbox.addWidget(self.foot_sep_val)

        self.std_step_vbox.addLayout( self.foot_sep_hbox )

        self.std_step_step_duration_hbox = QHBoxLayout()
        
        self.std_step_step_duration_label = QLabel("Step Duration")
        self.std_step_step_duration_hbox.addWidget( self.std_step_step_duration_label )

        self.std_step_step_duration_val = QDoubleSpinBox()
        self.std_step_step_duration_val.setSingleStep(.01)
        self.std_step_step_duration_hbox.addWidget( self.std_step_step_duration_val )

        self.std_step_vbox.addLayout( self.std_step_step_duration_hbox )

        self.std_step_sway_duration_hbox = QHBoxLayout()
        
        self.std_step_sway_duration_label = QLabel("Sway Duration")
        self.std_step_sway_duration_hbox.addWidget( self.std_step_sway_duration_label )

        self.std_step_sway_duration_val = QDoubleSpinBox()
        self.std_step_sway_duration_val.setSingleStep(.01)
        self.std_step_sway_duration_hbox.addWidget( self.std_step_sway_duration_val )

        self.std_step_vbox.addLayout( self.std_step_sway_duration_hbox )

        self.std_step_swing_height_hbox = QHBoxLayout()
        
        self.std_step_swing_height_label = QLabel("Swing Height")
        self.std_step_swing_height_hbox.addWidget( self.std_step_swing_height_label )

        self.std_step_swing_height_val = QDoubleSpinBox()
        self.std_step_swing_height_val.setSingleStep(.01)
        self.std_step_swing_height_hbox.addWidget( self.std_step_swing_height_val )

        self.std_step_vbox.addLayout( self.std_step_swing_height_hbox )

        self.std_step_lift_height_hbox = QHBoxLayout()
        
        self.std_step_lift_height_label = QLabel("Lift Height")
        self.std_step_lift_height_hbox.addWidget( self.std_step_lift_height_label )

        self.std_step_lift_height_val = QDoubleSpinBox()
        self.std_step_lift_height_val.setSingleStep(.01)
        self.std_step_lift_height_hbox.addWidget( self.std_step_lift_height_val )

        self.std_step_vbox.addLayout( self.std_step_lift_height_hbox )
       
        self.std_step_groupbox.setLayout( self.std_step_vbox ) 
        vbox.addWidget( self.std_step_groupbox )

        button_hbox = QHBoxLayout()
        
        button_get = QPushButton("Get Current Values")
        button_hbox.addWidget( button_get )

        button_submit = QPushButton("Send Values")
        button_hbox.addWidget( button_submit)

        vbox.addLayout( button_hbox )

        vbox.addStretch(1)

        self._widget.setLayout(vbox)

        #context.add_widget(self._widget)

        # publishers and subscribers

        self.param_pub = rospy.Publisher('/flor/footstep_planner/set_params', FootstepPlannerParams, queue_size=10)
        button_submit.pressed.connect(self.sendParams)

        self.param_sub = self.stateSubscriber   = rospy.Subscriber('/ros_footstep_planner/params', FootstepPlannerParams, self.getParamCallbackFcn)
        button_get.pressed.connect(self.getParams)


    def shutdown_plugin(self):
        print "Shutdown ..." 
        self.param_pub.unregister()
        print "Done!"
    
    def getParams( self ):
        msg = FootstepPlannerParams()

        msg.change_mask = 0
        
        self.param_pub.publish( msg ) 

    def sendParams( self ):
        msg = FootstepPlannerParams()

        msg.change_mask = 0
        
        if self.step_cost_groupbox.isChecked():
            msg.change_mask = msg.change_mask | 0x001

        if self.collision_check_groupbox.isChecked():
            msg.change_mask = msg.change_mask | 0x010

        if self.foot_size_groupbox.isChecked():
            msg.change_mask = msg.change_mask | 0x020

        if self.upper_groupbox.isChecked():
            msg.change_mask = msg.change_mask | 0x040

        if self.std_step_groupbox.isChecked():
            msg.change_mask = msg.change_mask | 0x080
        
        if self.terrain_model_groupbox.isChecked():
            msg.change_mask = msg.change_mask | 0x100

        ### STEP_COST_ESTIMATOR ########
        msg.step_cost_type = self.step_cost_combobox.currentIndex()

# #HARD       ### FOOTSTEP_SET ########
#        # parameters for discret footstep planning mode
#        vigir_atlas_control_msgs/VigirBehaviorStepData[] footstep_set            # set of footsteps (displacement vectors (in meter / rad))
#        float32[] footstep_cost                                         # cost for each footstep given in footstep set
#
# #HARD       ### LOAD_GPR_STEP_COST ########
#        # map step cost file
#        std_msgs/String map_step_cost_file
#
# #HARD       ### LOAD_MAP_STEP_COST ########
#        # destination of gpr file
#        std_msgs/String gpr_step_cost_file

        ### COLLISION_CHECK_TYPE ########
        msg.collision_check_type = 0

        if self.collision_check_feet_checkbox.isChecked():
            msg.collision_check_type = msg.collision_check_type | 0x01
        
        if self.collision_check_ub_checkbox.isChecked():
            msg.collision_check_type = msg.collision_check_type | 0x02

        if self.collision_check_fc_checkbox.isChecked():
            msg.collision_check_type = msg.collision_check_type | 0x04
        
        ### FOOT_SIZE ########
        msg.foot_size.x = self.foot_size_x.value()
        msg.foot_size.y = self.foot_size_y.value()
        msg.foot_size.z = self.foot_size_z.value()

        msg.foot_origin_shift.x = self.foot_shift_x.value() 
        msg.foot_origin_shift.y = self.foot_shift_y.value() 
        msg.foot_origin_shift.z = self.foot_shift_z.value() 

        ### UPPER_BODY_SIZE ########
        msg.upper_body_size.x = self.upper_size_x.value()
        msg.upper_body_size.y = self.upper_size_y.value()
        msg.upper_body_size.z = self.upper_size_z.value()
        
        msg.upper_body_origin_shift.x = self.upper_origin_x.value()
        msg.upper_body_origin_shift.y = self.upper_origin_y.value()
        msg.upper_body_origin_shift.z = self.upper_origin_z.value()
    
        ### TERRAIN_MODEL ########
        msg.use_terrain_model = self.terrain_model_checkbox.isChecked()
        msg.min_sampling_steps_x = self.terrain_min_ssx_val.value()
        msg.min_sampling_steps_y = self.terrain_min_ssy_val.value()
        msg.max_sampling_steps_x = self.terrain_max_ssx_val.value()
        msg.max_sampling_steps_y = self.terrain_max_ssy_val.value()
        msg.max_intrusion_z      = self.terrain_max_iz_val.value()
        msg.max_ground_clearance = self.terrain_max_gc_val.value()
        msg.minimal_support      = self.terrain_ms_val.value()

        ### STANDARD_STEP_PARAMS ########
        msg.foot_seperation = self.foot_sep_val.value() 
        msg.step_duration = self.std_step_step_duration_val.value() 
        msg.sway_duration = self.std_step_sway_duration_val.value() 
        msg.swing_height = self.std_step_swing_height_val.value()
        msg.lift_height =  self.std_step_lift_height_val.value() 

        print msg

        self.param_pub.publish( msg ) 


    def getParamCallbackFcn(self, msg):
     
        ### STEP_COST_ESTIMATOR ########
        if msg.change_mask & 0x001:
            self.step_cost_combobox.setCurrentIndex( msg.step_cost_type )

# #HARD       ### FOOTSTEP_SET ########
#        # parameters for discret footstep planning mode
#        vigir_atlas_control_msgs/VigirBehaviorStepData[] footstep_set            # set of footsteps (displacement vectors (in meter / rad))
#        float32[] footstep_cost                                         # cost for each footstep given in footstep set
#
# #HARD       ### LOAD_GPR_STEP_COST ########
#        # map step cost file
#        std_msgs/String map_step_cost_file
#
# #HARD       ### LOAD_MAP_STEP_COST ########
#        # destination of gpr file
#        std_msgs/String gpr_step_cost_file

        ### COLLISION_CHECK_TYPE ########
        if msg.change_mask & 0x010:
            self.collision_check_feet_checkbox.setChecked( msg.collision_check_type & 0x01 )
            self.collision_check_ub_checkbox.setChecked( msg.collision_check_type & 0x02 )
            self.collision_check_fc_checkbox.setChecked( msg.collision_check_type & 0x04 )

        ### FOOT_SIZE ########
        if msg.change_mask & 0x020:
            self.foot_size_x.setValue( msg.foot_size.x )
            self.foot_size_y.setValue( msg.foot_size.y )
            self.foot_size_z.setValue( msg.foot_size.z )

            self.foot_shift_x.setValue( msg.foot_origin_shift.x )
            self.foot_shift_y.setValue( msg.foot_origin_shift.y )
            self.foot_shift_z.setValue( msg.foot_origin_shift.z )
           
        ### UPPER_BODY_SIZE ########
        if msg.change_mask & 0x040:
            self.upper_size_x.setValue( msg.upper_body_size.x )
            self.upper_size_y.setValue( msg.upper_body_size.y )
            self.upper_size_z.setValue( msg.upper_body_size.z )
            
            self.upper_origin_x.setValue( msg.upper_body_origin_shift.x )
            self.upper_origin_y.setValue( msg.upper_body_origin_shift.y )
            self.upper_origin_z.setValue( msg.upper_body_origin_shift.z )

        ### STANDARD_STEP_PARAMS ########
        if msg.change_mask & 0x080:
            self.foot_sep_val.setValue( msg.foot_seperation ) 
            self.std_step_step_duration_val.setValue( msg.step_duration ) 
            self.std_step_sway_duration_val.setValue( msg.sway_duration ) 
            self.std_step_swing_height_val.setValue( msg.swing_height ) 
            self.std_step_lift_height_val.setValue( msg.lift_height ) 

        ### TERRAIN_MODEL ########
        if msg.change_mask & 0x100:
            self.terrain_model_checkbox.setChecked( msg.use_terrain_model )
            self.terrain_min_ssx_val.setValue(   msg.min_sampling_steps_x  )
            self.terrain_min_ssy_val.setValue(   msg.min_sampling_steps_y  )
            self.terrain_max_ssx_val.setValue(   msg.max_sampling_steps_x  )
            self.terrain_max_ssy_val.setValue(   msg.max_sampling_steps_y  )
            self.terrain_max_iz_val.setValue(  msg.max_intrusion_z     )  
            self.terrain_max_gc_val.setValue(  msg.max_ground_clearance )
            self.terrain_ms_val.setValue(    msg.minimal_support )
