import roslib
roslib.load_manifest('vigir_rqt_steering_interface')

import rospy
import tf.transformations
import math
import numpy as np

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, QAbstractListModel
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton

import PyKDL
from tf_conversions import posemath

from flor_atlas_nav_msgs.msg import Stepping
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vigir_grasp_msgs.msg import TemplateSelection

# using this class as an enum
class RightLegState:
	# just so we can refer to things by name
	FAST 		= 0
	SLOW 		= 1
	NEUTRAL 	= 2
	BRAKE 		= 3
	STOP 		= 4
	NUM_STATES 	= 5
	
	JOINT_STATE = 20
	
	def __init__(self):
		self._c_T_target_positions = []
		self.joint_positions = []		 # list of list of joint positions
		self.current_joint_positions = None # list of current joint positions
		self.current_state = RightLegState.JOINT_STATE  # last pose we sent to the controller
		
		# Joint Name : Child Link :OSRF
		self.joint_to_osrf = {}
		self.joint_to_osrf['r_leg_hpz'] = 10
		self.joint_to_osrf['r_leg_hpx'] = 11
		self.joint_to_osrf['r_leg_hpy'] = 12
		self.joint_to_osrf['r_leg_kny'] = 13
		self.joint_to_osrf['r_leg_aky'] = 14
		self.joint_to_osrf['r_leg_akx'] = 15
		
		self.colors = []
		for i in range(RightLegState.NUM_STATES):
			self.colors.append(ColorRGBA())
			self.colors[i].a = 0.6

		self.colors[RightLegState.FAST].r = 0.1
		self.colors[RightLegState.FAST].g = 0.9
		self.colors[RightLegState.FAST].b = 0.1
		
		self.colors[RightLegState.SLOW].r = 0.2
		self.colors[RightLegState.SLOW].g = 0.7
		self.colors[RightLegState.SLOW].b = 0.2
		
		self.colors[RightLegState.NEUTRAL].r = 0.2
		self.colors[RightLegState.NEUTRAL].g = 0.3
		self.colors[RightLegState.NEUTRAL].b = 0.8
		
		self.colors[RightLegState.BRAKE].r = 0.7
		self.colors[RightLegState.BRAKE].g = 0.2
		self.colors[RightLegState.BRAKE].b = 0.2
		
		self.colors[RightLegState.STOP].r = 0.9
		self.colors[RightLegState.STOP].g = 0.1
		self.colors[RightLegState.STOP].b = 0.1
		
		
	def get_trajectory_points(self, final_state):
		# take current JointState as the initial state and add intermediate states if needed
		self.current_state = final_state
		if ((self.current_state < RightLegState.NEUTRAL and final_state > RightLegState.NEUTRAL) or (self.current_state > RightLegState.NEUTRAL and final_state < RightLegState.NEUTRAL)):
			return [self.current_joint_positions,self.joint_positions[RightLegState.NEUTRAL],self.joint_positions[final_state]]
		else:
			return [self.current_joint_positions,self.joint_positions[final_state]]

class RightArmState:
	# just so we can refer to things by name
	HLEFT 		= 0
	LEFT 		= 1
	FWD 		= 2
	RIGHT 		= 3
	HRIGHT 		= 4
	PRE_FORWARD = 5
	FORWARD 	= 6
	REVERSE 	= 7
	PRE_PARKING = 8
	PARKING		= 9
	NUM_STATES 	= 10
	
	JOINT_STATE = 20
	
	def __init__(self):
		self._c_T_target_positions = []
		self.joint_positions = []		 # list of list of joint positions
		self.current_joint_positions = None # list of current joint positions
		self.current_state = RightArmState.JOINT_STATE  # last pose we sent to the controller
		# Joint Name : Child Link :OSRF
		self.joint_to_osrf = {}
		self.joint_to_osrf['r_arm_shz'] = 22
		self.joint_to_osrf['r_arm_shx'] = 23
		self.joint_to_osrf['r_arm_ely'] = 24
		self.joint_to_osrf['r_arm_elx'] = 25
		self.joint_to_osrf['r_arm_wry'] = 26
		self.joint_to_osrf['r_arm_wrx'] = 27
		
		self.colors = []
		for i in range(RightArmState.NUM_STATES):
			self.colors.append(ColorRGBA())
			self.colors[i].a = 0.6

		self.colors[RightArmState.PRE_FORWARD].r = 0.1
		self.colors[RightArmState.PRE_FORWARD].g = 0.2
		self.colors[RightArmState.PRE_FORWARD].b = 0.8
		
		self.colors[RightArmState.FORWARD].r = 0.1
		self.colors[RightArmState.FORWARD].g = 0.9
		self.colors[RightArmState.FORWARD].b = 0.1
		
		self.colors[RightArmState.REVERSE].r = 0.9
		self.colors[RightArmState.REVERSE].g = 0.1
		self.colors[RightArmState.REVERSE].b = 0.1
		
		self.colors[RightArmState.PRE_PARKING].r = 1.0
		self.colors[RightArmState.PRE_PARKING].g = 0.5
		self.colors[RightArmState.PRE_PARKING].b = 0.0
		
		self.colors[RightArmState.PARKING].r = 0.9
		self.colors[RightArmState.PARKING].g = 0.1
		self.colors[RightArmState.PARKING].b = 0.1
	
	def get_trajectory_points(self, final_state):
		# take current JointState as the initial state and add intermediate states if needed
		if self.current_state == RightArmState.JOINT_STATE:
			return [self.current_joint_positions,self.joint_positions[final_state]]
		elif final_state == RightArmState.PARKING:
			return [self.current_joint_positions,self.joint_positions[RightArmState.PARKING],self.joint_positions[final_state]]
		elif final_state == RightArmState.FORWARD or final_state == RightArmState.REVERSE:
			return [self.current_joint_positions,self.joint_positions[RightArmState.PRE_FORWARD],self.joint_positions[final_state]]
		else:
			if self.current_state < final_state:
				self.current_state = final_state
				print 'Setting the right arm joint positions: current+',range(self.current_state,final_state+1)
				return [self.current_joint_positions]+[self.joint_positions[state] for state in range(self.current_state,final_state+1)]
			elif self.current_state > final_state:
				self.current_state = final_state
				print 'Setting the right arm joint positions: current+',range(self.current_state,final_state-1,-1)
				return [self.current_joint_positions]+[self.joint_positions[state] for state in range(self.current_state,final_state-1,-1)]

class SteeringInterfaceDialog(Plugin):
	def __init__(self, context):
		super(SteeringInterfaceDialog, self).__init__(context)
		self.setObjectName('SteeringInterfaceDialog')

		self.time_step  = rospy.get_param('~time_step', 1.0)
		
		self.listener = tf.TransformListener()

		# Assuming right arm control for now
		self.r_arm_pub  = rospy.Publisher('/flor/r_arm_controller/trajectory',JointTrajectory, None, False, True, None, queue_size=10)
		self.r_leg_pub  = rospy.Publisher('/flor/r_leg_controller/trajectory',JointTrajectory, None, False, True, None, queue_size=10)
		
		self.markers_pub = rospy.Publisher('/flor_footstep_planner/footsteps_array',MarkerArray, None, False, True, None, queue_size=10)

		self.r_leg_state = RightLegState()
		self.r_arm_state = RightArmState()

		# External data coming in (current joint states, and template pose (utility vehicle model)
		self.joint_data		 = JointState()
		self.template_selection = TemplateSelection()
		self.joints_sub		 = rospy.Subscriber('/atlas/joint_states',JointState, self.joint_states_callback )
		self.template_sub	   = rospy.Subscriber('/template/template_selected',TemplateSelection, self.template_selection_callback )

		# ------------------------------------------------------------------------------------
		# TODO--- Define these transforms relative to the origin of the large polaris mesh

		#Name 					X 		Y 		Z
		#Parking Break 			-0.07 	0.48 	-0.87
		#Parking Break Free 	-0.07 	0.53 	-0.85
		#FNR Switch 			0.02 	0.56 	-0.90
		#FNR Switch F 			0.02 	0.60 	-0.90
		#FNR Switch R 			0.02 	0.55 	-0.95
		#Gas Pedal 				-0.10 	0.60 	-1.50
		#Gas Pedal Half 		-0.10 	0.62 	-1.53
		#Gas Pedal Full 		-0.10 	0.65 	-1.56
		#Break Pedal 			-0.27 	0.60 	-1.50
		#Break Pedal Slow 		-0.27 	0.63 	-1.53
		#Break Pedal Stop 		-0.27 	0.66 	-1.55

		# Transform brake and throttle to car template (static transforms relative to the car)
		# leg
		self._c_T_b	 	  = [-0.27, 0.60,-1.50] # (b)rake
		self._c_T_b_slow  = [-0.27, 0.63,-1.53]
		self._c_T_b_stop  = [-0.27, 0.66,-1.55]
		self._c_T_t	 	  = [-0.10, 0.60,-1.50] # (t)hrottle
		self._c_T_t_slow  = [-0.10, 0.62,-1.53]
		self._c_T_t_fast  = [-0.10, 0.65,-1.56]
		self._c_T_n	  	  = [-0.185,0.60,-1.50] # (n)eutral // in between neutral gas/brake
		# need to add them in order for the transforms to work
		#FAST = 0		self._c_T_t_fast  = [-0.10, 0.65,-1.56]
		#SLOW = 1		self._c_T_t_slow  = [-0.10, 0.62,-1.53]
		#NEUTRAL = 2	self._c_T_n		  = [-0.185,0.60,-1.50] # (n)eutral // in between neutral gas/brake
		#BRAKE = 3 		self._c_T_b_slow  = [-0.27, 0.63,-1.53]
		#STOP = 4  		self._c_T_b_stop  = [-0.27, 0.66,-1.55]
		self.r_leg_state._c_T_target_positions.append(self._c_T_t_fast)
		self.r_leg_state._c_T_target_positions.append(self._c_T_t_slow)
		self.r_leg_state._c_T_target_positions.append(self._c_T_n)
		self.r_leg_state._c_T_target_positions.append(self._c_T_b_slow)
		self.r_leg_state._c_T_target_positions.append(self._c_T_b_stop)
		
		
		# arm
		self._c_T_hl	  = [ 0.00, 0.00, 0.00] # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< NEED THESE POSITIONS
		self._c_T_l	      = [ 0.00, 0.00, 0.00] # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< NEED THESE POSITIONS
		self._c_T_fwd	  = [ 0.00, 0.00, 0.00] # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< NEED THESE POSITIONS
		self._c_T_r	      = [ 0.00, 0.00, 0.00] # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< NEED THESE POSITIONS
		self._c_T_hr	  = [ 0.00, 0.00, 0.00] # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< NEED THESE POSITIONS
		self._c_T_p	 	  = [-0.07, 0.48,-0.87] # (p)arking brake
		self._c_T_p_free  = [-0.07, 0.53,-0.85]
		self._c_T_f	 	  = [ 0.02, 0.56,-0.90] # (f)wd_button
		self._c_T_f_f	  = [ 0.02, 0.60,-0.90]
		self._c_T_f_r	  = [ 0.02, 0.55,-0.95]
		# need to add them in order
		#HLEFT = 0  
		#LEFT = 1
		#FWD = 2
		#RIGHT = 3
		#HRIGHT = 4
		#PRE_FORWARD = 5	self._c_T_f	 	  = [ 0.02, 0.56,-0.90] # (f)wd_button
		#FORWARD = 6		self._c_T_f_f	  = [ 0.02, 0.60,-0.90]
		#REVERSE = 7		self._c_T_f_r	  = [ 0.02, 0.55,-0.95]
		#PRE_PARKING = 8	self._c_T_p	      = [-0.07, 0.48,-0.87] # (p)arking brake
		#PARKING = 9		self._c_T_p_free  = [-0.07, 0.53,-0.85]
		self.r_arm_state._c_T_target_positions.append(self._c_T_hl)
		self.r_arm_state._c_T_target_positions.append(self._c_T_l)
		self.r_arm_state._c_T_target_positions.append(self._c_T_fwd)
		self.r_arm_state._c_T_target_positions.append(self._c_T_r)
		self.r_arm_state._c_T_target_positions.append(self._c_T_hr)
		self.r_arm_state._c_T_target_positions.append(self._c_T_f)
		self.r_arm_state._c_T_target_positions.append(self._c_T_f_f)
		self.r_arm_state._c_T_target_positions.append(self._c_T_f_r)
		self.r_arm_state._c_T_target_positions.append(self._c_T_p)
		self.r_arm_state._c_T_target_positions.append(self._c_T_p_free)

		# ---------------------------------------------------------------------------------

		# Widget layout stuff
		# -----------------------------------------------------------------------
		self._widget = QWidget()
		vbox		= QVBoxLayout()
		top_widget  = QWidget()
		top_hbox	= QHBoxLayout()

		# top row
		hard_left_command = QPushButton("H L")
		hard_left_command.clicked.connect(self.hard_left_command_callback)
		top_hbox.addWidget(hard_left_command)

		left_command = QPushButton("Left")
		left_command.clicked.connect(self.left_command_callback)
		top_hbox.addWidget(left_command)

		straight_command = QPushButton("Fwd")
		straight_command.clicked.connect(self.straight_command_callback)
		top_hbox.addWidget(straight_command)

		right_command = QPushButton("Right")
		right_command.clicked.connect(self.right_command_callback)
		top_hbox.addWidget(right_command)

		hard_right_command = QPushButton("H R")
		hard_right_command.clicked.connect(self.hard_right_command_callback)
		top_hbox.addWidget(hard_right_command)
		top_widget.setLayout(top_hbox)
		vbox.addWidget(top_widget)

		# Center column
		bottom_widget   = QWidget()
		bottom_hbox	 = QHBoxLayout()

		calc_widget = QWidget()
		calc_vbox   = QVBoxLayout()
		calc_vbox.addStretch()

		calc_command = QPushButton("Calc")
		calc_command.clicked.connect(self.calc_command_callback)
		calc_vbox.addWidget(calc_command)
		calc_widget.setLayout(calc_vbox)

		bottom_hbox.addWidget(calc_widget)

		throttle_widget = QWidget()
		throttle_vbox   = QVBoxLayout()

		fast_command = QPushButton("Fast")
		fast_command.clicked.connect(self.fast_command_callback)
		throttle_vbox.addWidget(fast_command)

		slow_command = QPushButton("Slow")
		slow_command.clicked.connect(self.slow_command_callback)
		throttle_vbox.addWidget(slow_command)

		neutral_command = QPushButton("Neutral")
		neutral_command.clicked.connect(self.neutral_command_callback)
		throttle_vbox.addWidget(neutral_command)

		slow_command = QPushButton("Brake")
		slow_command.clicked.connect(self.slow_command_callback)
		throttle_vbox.addWidget(slow_command)

		stop_command = QPushButton("Stop")
		stop_command.clicked.connect(self.stop_command_callback)
		throttle_vbox.addWidget(stop_command)
		throttle_widget.setLayout(throttle_vbox)

		bottom_hbox.addWidget(throttle_widget)

		right_widget = QWidget()
		right_vbox   = QVBoxLayout()
		right_vbox.addStretch()
		parking_command = QPushButton("Parking")
		parking_command.clicked.connect(self.parking_command_callback)
		right_vbox.addWidget(parking_command)

		go_button_command = QPushButton("Fwd")
		go_button_command.clicked.connect(self.go_button_command_callback)
		right_vbox.addWidget(go_button_command)
		reverse_button_command = QPushButton("R")
		reverse_button_command.clicked.connect(self.reverse_button_command_callback)
		right_vbox.addWidget(reverse_button_command)

		right_widget.setLayout(right_vbox)

		bottom_hbox.addWidget(right_widget)

		bottom_widget.setLayout(bottom_hbox)

		vbox.addWidget(bottom_widget)

		self._widget.setLayout(vbox)
		# --------------------------------------------------- End Widget stuff -------------------------------

		context.add_widget(self._widget)

	def shutdown_plugin(self):
		print "Shutting down ..."
		self.r_arm_pub.unregister()
		self.r_leg_pub.unregister()
		self.joints_sub.unregister()
		self.template_sub.unregister()
		print "Done!"

	def joint_states_callback(self,data):
		# Store the latest joint data
		self.joint_states = data

		# Update structures of the state machines
		self.r_leg_state.current_joint_positions = []
		for k in self.r_leg_state.joint_to_osrf.keys():
			self.r_leg_state.current_joint_positions.append(self.joint_states.position[self.r_leg_state.joint_to_osrf[k]])
	
		self.r_arm_state.current_joint_positions = []
		for k in self.r_arm_state.joint_to_osrf.keys():
			self.r_arm_state.current_joint_positions.append(self.joint_states.position[self.r_arm_state.joint_to_osrf[k]])

		#print "Updated joint state data at ",self.joint_states.header.stamp.to_sec()

	def template_selection_callback(self, template):
		self.template_selection = template
		
		print "Updated template selection with ",self.template_selection.pose

	def publish_trajectory(self, pub, joint_traj_positions):
		trajectory = JointTrajectory()
		trajectory.header.stamp = rospy.Time.now()
		trajectory.points		= [JointTrajectoryPoint() for p in joint_traj_positions]
		for i,position in enumerate(joint_traj_positions):
			trajectory.points[i].positions = joint_traj_positions[i]
			trajectory.points[i].velocities = 0.0
			trajectory.points[i].time_from_start = rospy.Duration(i*self.time_step)

		print "Publishing trajectory at ",trajectory.header.stamp
		pub.publish(trajectory)

	# Define system command strings
	def hard_left_command_callback(self):
		# Define trajectory from current point positions to hard right target
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightArmState.HLEFT)
		self.publish_trajectory(self.r_arm_pub, trajectory_positions)
		print "Send hard left command:",trajectory_positions

	def left_command_callback(self):
		# Define trajectory from current point positions to left target
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightArmState.LEFT)
		self.publish_trajectory(self.r_arm_pub, trajectory_positions)
		print "Send left command:",trajectory_positions

	def straight_command_callback(self):
		# Define trajectory from current point positions to straight steering target
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightArmState.FWD)
		self.publish_trajectory(self.r_arm_pub, trajectory_positions)
		print "Send forward command:",trajectory_positions

	def right_command_callback(self):
		# Define trajectory from current point positions to right target
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightArmState.RIGHT)
		self.publish_trajectory(self.r_arm_pub, trajectory_positions)
		print "Send right command:",trajectory_positions

	def hard_right_command_callback(self):
		# Define trajectory from current point positions to hard right target
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightArmState.HRIGHT)
		self.publish_trajectory(self.r_arm_pub, trajectory_positions)
		print "Send hard right command:",trajectory_positions

	def fast_command_callback(self):
		# This defines trajectory for moving leg through neutral position
		# to apply light throttle
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightLegState.FAST)
		self.publish_trajectory(self.r_leg_pub, trajectory_positions)
		print "Send fast command:",trajectory_positions

	def slow_command_callback(self):
		# This defines trajectory for moving leg through neutral position
		# to apply light throttle
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightLegState.SLOW)
		self.publish_trajectory(self.r_leg_pub, trajectory_positions)
		print "Send slow command:",trajectory_positions

	def neutral_command_callback(self):
		# This defines trajectory for moving leg to neutral position
		# just needs two points, current and neutral position
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightLegState.NEUTRAL)
		self.publish_trajectory(self.r_leg_pub, trajectory_positions)
		print "Send neutral command:",trajectory_positions

	def brake_command_callback(self):
		# This slight braking trajectory if not currently applying brake
		# if currently in throttle, move joints to neutral position, then to foot on brake position
		# generate a 3 point trajectory for right leg from current --> neutral --> braking
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightLegState.BRAKE)
		self.publish_trajectory(self.r_leg_pub, trajectory_positions)
		print "Send brake command:",trajectory_positions

	def stop_command_callback(self):
		# This defines stomp on brake trajectory if not currently applying brake
		# if currently in throttle, move joints to neutral position, then full stop position
		# generate a 3 point trajectory for right leg from current --> neutral --> full stop
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightLegState.STOP)
		self.publish_trajectory(self.r_leg_pub, trajectory_positions)
		print "Send fast command:",trajectory_positions

	def go_button_command_callback(self):
		# this should publish the pose of go button as target for moveit planner
		# can plan online
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightArmState.FORWARD)
		self.publish_trajectory(self.r_arm_pub, trajectory_positions)
		print "Send go button command:",trajectory_positions
		
	def reverse_button_command_callback(self):
		# this should publish the pose of go button as target for moveit planner
		# can plan online
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightArmState.REVERSE)
		self.publish_trajectory(self.r_arm_pub, trajectory_positions)
		print "Send go button command:",trajectory_positions

	def parking_command_callback(self):
		# this should publish the pose of parking brake as target for grasping/moveit planner
		# can plan online
		trajectory_positions = self.r_arm_state.get_trajectory_points(RightArmState.PARKING)
		self.publish_trajectory(self.r_arm_pub, trajectory_positions)
		print "Send parking command:",trajectory_positions

	def calc_command_callback(self):

		print "Calculate joint targets = "
		
		# Get robot pose in world (
		# wTr =
		#now = rospy.Time.now()
		#self.listener.waitForTransform("/world", "/pelvis", now, rospy.Duration(4.0))
		#w_T_r = self.listener.lookupTransform("/world", "/pelvis", now)
		#print w_T_r

		# Get vehicle pose in world from Template selection message
		#  wTc =
                # this is already in tf
                #Should be:
                #self.listener.waitForTransform("/world", "/car", now, rospy.Duration(4.0))  #TODO change /car with the right frame name
                #w_T_c = self.listener.lookupTransform("/world", "/car", now)
                #print w_T_c
			
		# initialize joint positions structure
		print "setting the transforms for moveit"
		self.r_leg_state.joint_positions = RightLegState.NUM_STATES*[len(self.r_leg_state.joint_to_osrf.keys())*[0.0]]
		self.r_arm_state.joint_positions = RightArmState.NUM_STATES*[len(self.r_arm_state.joint_to_osrf.keys())*[0.0]]
		
		# For each transform, call to moveit to generate
		#  Joint positions for each target relative to current robot pose
		#
		
		# EXAMPLE
		# Brake position in world
		# wTb = wTc * cTb (static pose)
		# ATTENTION: THIS WILL ONLY WORK IF THERE IS A TEMPLATE IN THE OCS AND THE TEMPLATE NODELET IS RUNNING
		
		#RightLegState:
		#FAST = 0		self._c_T_t_fast  = [-0.10, 0.65,-1.56]
		#SLOW = 1		self._c_T_t_slow  = [-0.10, 0.62,-1.53]
		#NEUTRAL = 2	self._c_T_n		  = [-0.185,0.60,-1.50] # (n)eutral // in between neutral gas/brake
		#BRAKE = 3 		self._c_T_b_slow  = [-0.27, 0.63,-1.53]
		#STOP = 4  		self._c_T_b_stop  = [-0.27, 0.66,-1.55]
	
		cube = Point()
		cube.x=cube.y=cube.z=0.045
		
		markers = MarkerArray()
			
		marker = Marker()
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = "/world"
		marker.action = 0#Marker.ADD
		marker.scale = cube
		marker.type = 6#Marker.CUBE_LIST
		marker.ns = "test" 

		wTc = posemath.fromMsg(self.template_selection.pose.pose)

		for i,position in enumerate(self.r_leg_state._c_T_target_positions):
			tmp = PoseStamped()
			tmp.header.stamp = rospy.Time.now()
			#tmp.header.frame_id = "/template_tf_"+str(self.template_selection.template_id.data)
			tmp.pose.position.x = position[0]
			tmp.pose.position.y = position[1]
			tmp.pose.position.z = position[2]
			tmp.pose.orientation.w = 1.0
			tmp.pose.orientation.x = 0.0
			tmp.pose.orientation.y = 0.0
			tmp.pose.orientation.z = 0.0
			#transformed_pose = self.listener.transformPose("/world",tmp)
			
			f1 = posemath.fromMsg(tmp.pose)
			#f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(w_T_r[1][0],w_T_r[1][1],w_T_r[1][2],w_T_r[1][3]),PyKDL.Vector(w_T_r[0][0],w_T_r[0][1],w_T_r[0][2]))
			#Should be:
			#f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(w_T_c[1][0],w_T_c[1][1],w_T_c[1][2],w_T_c[1][3]),PyKDL.Vector(w_T_c[0][0],w_T_c[0][1],w_T_c[0][2]))
			#f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(w_T_r[1][0],w_T_r[1][1],w_T_r[1][2],w_T_r[1][3]),PyKDL.Vector(0.98,0.0,1.10)) #TOREMOVE this is just an example
			#We want w_T_b = w_T_c * c_T_b
			f = wTc * f1
			
			tmp.pose = posemath.toMsg(f)
			
			marker.points.append(tmp.pose.position)
			marker.colors.append(self.r_leg_state.colors[i])
			# call moveit with transformed_pose
			#joint_positions = moveit
			#self.r_leg_state.joint_positions.append(joint_positions)
	
		#markers.markers.append(marker)
		
		#RightArmState:
		#HLEFT = 0
		#LEFT = 1
		#FWD = 2
		#RIGHT = 3
		#HRIGHT = 4
		#PRE_FORWARD = 5	self._c_T_f	 	  = [ 0.02, 0.56,-0.90] # (f)wd_button
		#FORWARD = 6		self._c_T_f_f	  = [ 0.02, 0.60,-0.90]
		#REVERSE = 7		self._c_T_f_r	  = [ 0.02, 0.55,-0.95]
		#PRE_PARKING = 8	self._c_T_p	      = [-0.07, 0.48,-0.87] # (p)arking brake
		#PARKING = 9		self._c_T_p_free  = [-0.07, 0.53,-0.85]
			
		#marker = Marker()
		#marker.header.stamp = rospy.Time.now()
		#marker.header.frame_id = "/world"
		#marker.action = 0#Marker.ADD
		#marker.scale = cube
		#marker.color = green
		#marker.type = 6#Marker.CUBE_LIST
		#marker.ns = "test" 

		for i,position in enumerate(self.r_arm_state._c_T_target_positions):
			tmp = PoseStamped()
			tmp.header.stamp = rospy.Time.now()
			#tmp.header.frame_id = "/template_tf_"+str(self.template_selection.template_id.data)
			tmp.pose.position.x = position[0]
			tmp.pose.position.y = position[1]
			tmp.pose.position.z = position[2]
			tmp.pose.orientation.w = 1.0
			tmp.pose.orientation.x = 0.0
			tmp.pose.orientation.y = 0.0
			tmp.pose.orientation.z = 0.0
			#transformed_pose = self.listener.transformPose("/world",tmp)
			
			f1 = posemath.fromMsg(tmp.pose)
			#f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(w_T_r[1][0],w_T_r[1][1],w_T_r[1][2],w_T_r[1][3]),PyKDL.Vector(w_T_r[0][0],w_T_r[0][1],w_T_r[0][2]))
			#Should be:
			#f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(w_T_c[1][0],w_T_c[1][1],w_T_c[1][2],w_T_c[1][3]),PyKDL.Vector(w_T_c[0][0],w_T_c[0][1],w_T_c[0][2]))
			#f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(w_T_r[1][0],w_T_r[1][1],w_T_r[1][2],w_T_r[1][3]),PyKDL.Vector(0.98,0.0,1.10)) #TOREMOVE this is just an example
			#We want w_T_b = w_T_c * c_T_b
			f = wTc * f1
			
			tmp.pose = posemath.toMsg(f)
			
			marker.points.append(tmp.pose.position)
			marker.colors.append(self.r_arm_state.colors[i])
			# call moveit with 
			#joint_positions = moveit
			#self.r_arm_state.joint_positions.append(joint_positions)
		
		markers.markers.append(marker)
		self.markers_pub.publish(markers)

		print self.r_leg_state.joint_positions
		print self.r_arm_state.joint_positions
		

