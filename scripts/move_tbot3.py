#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
from pid_pro import PID
import copy
import os,sys
from gazebo_msgs.srv import SpawnModel

class moveTbot3:
	def __init__(self):
		self.model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size = 10)
		rospy.init_node('move_turtle',anonymous = True)
		self.actions = String()
		self.pose = Pose()
		self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
		self.action_subscriber = rospy.Subscriber('/actions',String,self.callback_actions)
		self.pid_subscriber = rospy.Subscriber("/Controller_Status",String,self.callback_pid)
		self.pose_subscriber = rospy.Subscriber('/odom',Odometry,self.pose_callback)
		self.status_publisher = rospy.Publisher("/status",String,queue_size = 10)
		self.free = String(data = "Idle")
		self.rate = rospy.Rate(30)
		print "Ready!"
		rospy.spin()

	def spawn_can(self,posx, posy):
	    global i_d
	    i_d+=1
	    f_read = open(os.path.expanduser("~") + "/.gazebo/models/fuelstation/model-2.sdf",'r')
	    sdff = f_read.read()
	    f_read.close()
	    rospy.wait_for_service('gazebo/spawn_sdf_model')
	    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
	    pose = Pose()
	    pose.position.x = posx
	    pose.position.y = posy
	    pose.position.z = 0
	    spawn_model_prox("fuelstat{}".format(i_d), sdff, "robots_name_space", pose, "world")

	def callback_pid(self,data):
		if data.data == "Done":
			if len(self.actions)>0:
				self.execute_next()

	def callback_actions(self,data):
		self.actions = data.data.split("_")
		self.rate.sleep()
		self.execute_next()
		# self.move()

	def execute_next(self):
		action = self.actions.pop(0)
		direction = None
		
		if action == "REFUEL":
			#add changing color of fuel can
			print "Recharged!"
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			current_yaw = euler[2]
			
			'''
			approx pos to nearest 0.5
			'''
			fuel_can_pose_x = round(current_pose.position.x*2.0)/2
			fuel_can_pose_y = round(current_pose.position.y*2.0)/2
			print("HELLLOOOO")
			self.spawn_can(fuel_can_pose_x,fuel_can_pose_y)

			PID(current_pose,"linear").publish_velocity()




		
		elif action == "MoveF":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			current_yaw = euler[2]
			if current_yaw > (-1.0* math.pi /8.0) and current_yaw < (math.pi / 8.0):
				print "Case 1 : East"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += 0.5
				#direction = 'x'
				#incr y co-ordinate
			elif current_yaw > (math.pi / 8.0 ) and current_yaw < (3.0 * math.pi / 8.0):
				print "Case 2: North East"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y += 0.5
				target_pose.position.x += 0.5
				#direction = 'y'
				#decr x co
			elif current_yaw > (3.0*math.pi /8.0) and current_yaw < (5.0*math.pi /8.0):
				print "Case 3:North"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y += 0.5
				#direction = '-y'
			elif current_yaw > (5 * math.pi / 8.0) and current_yaw < (7.0 * math.pi / 8.0):  # Facing SOUTH EAST
				print "Case 4: North West"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x -= 0.5
				target_pose.position.y += 0.5
			elif current_yaw > (-3.0 * math.pi / 8.0) and current_yaw < (-1.0 * math.pi / 8.0):  # Facing SOUTH EAST
				print "Case 5: South East"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += 0.5
				target_pose.position.y -= 0.5
				#direction = '-x'
			elif current_yaw > (-5.0 * math.pi / 8.0) and current_yaw < (-3.0 * math.pi / 8.0):  # Facing SOUTH
				print "Case 6: South"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y -= 0.5
			elif current_yaw > (-7.0 * math.pi / 8.0) and current_yaw < (-5.0 * math.pi / 8.0):  # Facing SOUTH WEST
				print "Case 7: South West"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x -= 0.5
				target_pose.position.y -= 0.5
			else:
				print "Case 8: West"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x -= 0.5
			PID(target_pose,"linear").publish_velocity()

		elif action == "MoveB":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			current_yaw = euler[2]
			if current_yaw > (-1.0* math.pi /8.0) and current_yaw < (math.pi / 8.0):
				print "Case 1 : East"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x -= 0.5
				#direction = 'x'
				#incr y co-ordinate
			elif current_yaw > (math.pi / 8.0 ) and current_yaw < (3.0 * math.pi / 8.0):
				print "Case 2: North East"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y -= 0.5
				target_pose.position.x -= 0.5
				#direction = 'y'
				#decr x co
			elif current_yaw > (3.0*math.pi /8.0) and current_yaw < (5.0*math.pi /8.0):
				print "Case 3:North"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y -= 0.5
				#direction = '-y'
			elif current_yaw > (5 * math.pi / 8.0) and current_yaw < (7.0 * math.pi / 8.0):  # Facing SOUTH EAST
				print "Case 4: North West"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += 0.5
				target_pose.position.y -= 0.5
			elif current_yaw > (-3.0 * math.pi / 8.0) and current_yaw < (-1.0 * math.pi / 8.0):  # Facing SOUTH EAST
				print "Case 5: South East"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x -= 0.5
				target_pose.position.y += 0.5
				#direction = '-x'
			elif current_yaw > (-5.0 * math.pi / 8.0) and current_yaw < (-3.0 * math.pi / 8.0):  # Facing SOUTH
				print "Case 6: South"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y += 0.5
			elif current_yaw > (-7.0 * math.pi / 8.0) and current_yaw < (-5.0 * math.pi / 8.0):  # Facing SOUTH WEST
				print "Case 7: South West"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += 0.5
				target_pose.position.y += 0.5
			else:
				print "Case 8: West"
				#raw_input()
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += 0.5
			PID(target_pose,"linear").publish_velocity()

		elif action == "TurnCW" or action == "TurnCCW":
			print "Case: Turning"
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			yaw = euler[2]
			if action == "TurnCW":
				target_yaw = yaw - ( math.pi / 4.0)
				if target_yaw < -math.pi:
					target_yaw += (math.pi * 2)
			else:
				target_yaw = yaw + ( math.pi / 4.0)
				if target_yaw >= (math.pi ):
					target_yaw -= (math.pi * 2 )
			target_pose = Pose()
			target_pose.position = current_pose.position
			target_quat = Quaternion(*tf.transformations.quaternion_from_euler(euler[0],euler[1],target_yaw))
			target_pose.orientation = target_quat
			print target_pose.orientation
			PID(target_pose,"rotational").publish_velocity()

		else:
			print "Invalid action " + action
			exit(-1)
		if len(self.actions) == 0:
			self.status_publisher.publish(self.free)


	def pose_callback(self,data):
		self.pose = data.pose.pose

	

if __name__ == "__main__":
	i_d = 100
	try:
		moveTbot3()
	except rospy.ROSInterruptException:
		pass
