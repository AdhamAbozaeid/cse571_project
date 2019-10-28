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
from pid import PID
import copy

class moveTbot3:
	def __init__(self):
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
		if action == "MoveF" or action == "MoveB":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			current_yaw = euler[2]

			position_change = {
				"MoveF" :  {
					"EAST" : {
						"X" : 0.5,
						"Y" : 0.0
					},
					"NORTH_EAST": {
						"X": 0.5,
						"Y": 0.5
					},
					"NORTH" : {
						"X" : 0.0,
						"Y" : 0.5
					},
					"NORTH_WEST": {
						"X": 0.5,
						"Y": -0.5
					},
					"WEST" : {
						"X" : -0.5,
						"Y" : 0.0
					},
					"SOUTH_WEST": {
						"X": -0.5,
						"Y": -0.5
					},
					"SOUTH" : {
						"X" : 0.0,
						"Y" : -0.5
					},
					"SOUTH_EAST": {
						"X": 0.5,
						"Y": -0.5
					},
				},
			}

			direction = "WEST"
			if action == "MoveF":
				factor = 1
			else:
				factor = -1
			if current_yaw > (-math.pi / 8.0) and current_yaw < (math.pi / 8.0):  # Facing EAST
				print "Case EAST"
				direction = "EAST"
			if current_yaw > (math.pi / 8.0) and current_yaw < (3.0 * math.pi / 8.0):  # Facing NORTH EAST
					print "Case NORTH_EAST"
					direction = "NORTH_EAST"
			elif current_yaw > (3.0 * math.pi / 8.0 ) and current_yaw < (5.0 * math.pi / 8.0):  # Facing NORTH
				print "Case NORTH"
				direction = "NORTH"
			elif current_yaw > (5 * math.pi / 8.0) and current_yaw < (7.0 * math.pi / 8.0):  # Facing NORTH WEST
				print "Case NORTH_WEST"
				direction = "NORTH_WEST"
			elif current_yaw > (-5.0 * math.pi / 8.0) and current_yaw < (-3.0 * math.pi / 8.0):  # Facing SOUTH
				print "Case SOUTH"
				direction = "SOUTH"
			elif current_yaw > (-7.0 * math.pi / 8.0) and current_yaw < (-5.0 * math.pi / 8.0):  # Facing SOUTH WEST
				print "Case SOUTH_WEST"
				direction = "SOUTH_WEST"
			else:  # Facing WEST
				print "Case 4"
				direction = "WEST"
			target_pose = copy.deepcopy(current_pose)
			target_pose.position.x += (position_change[action][direction]["X"] * factor)
			target_pose.position.y += (position_change[action][direction]["Y"] * factor)

			PID(target_pose,"linear").publish_velocity()
			
		elif action == "TurnCW" or action == "TurnCCW":
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
			print "Invalid action"
			exit(-1)
		if len(self.actions) == 0:
			self.status_publisher.publish(self.free)


	def pose_callback(self,data):
		self.pose = data.pose.pose

	

if __name__ == "__main__":
	try:
		moveTbot3()
	except rospy.ROSInterruptException:
		pass
