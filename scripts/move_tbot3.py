#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

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
		self.free = String(data = "next")
		self.rate = rospy.Rate(30)
		print("Ready!")
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
		#print(self.actions)
		action = self.actions.pop(0)
		print('execute_action',action)
		direction = None
		if action == "MoveF" or action == "MoveB":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			current_yaw = euler[2]

			position_change = {
				"MoveF" :  {
					"EAST" : 0.5,
					"NORTH" : 0.5,
					"WEST" : -0.5,
					"SOUTH" : -0.5
				},
				"MoveB" : {
					"EAST" : -0.5,
					"NORTH" : -0.5,
					"WEST" : 0.5,
					"SOUTH" : 0.5
				}
			}
			
			if current_yaw > (-math.pi / 8.0) and current_yaw < (math.pi / 8.0):  # Facing EAST
				print "Case 1"
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += position_change[action]["EAST"]
			elif current_yaw > (math.pi / 8.0) and current_yaw < (3.0*math.pi / 8.0):  # Facing NORTHEAST
				print "Case 5"
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += position_change[action]["EAST"]
				target_pose.position.y += position_change[action]["NORTH"]
				
			elif current_yaw > (3.0* math.pi / 8.0 ) and current_yaw < (5.0 * math.pi / 8.0):  # Facing NORTH
				print "Case 2"
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y += position_change[action]["NORTH"]
			elif current_yaw > (-3.0*math.pi / 8.0) and current_yaw < (-math.pi / 8.0):  # Facing SOUTHEAST
				print "Case 6"
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += position_change[action]["EAST"]
				target_pose.position.y += position_change[action]["SOUTH"]
				
			elif current_yaw > (-5.0 * math.pi / 8.0) and current_yaw < (-3.0*math.pi /8.0):  # Facing SOUTH
				print "Case 3"
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.y +=  position_change[action]["SOUTH"]
			elif current_yaw > (5.0*math.pi / 8.0) and current_yaw < (7.0*math.pi / 8.0):  # Facing NORTHWEST
				print "Case 7"
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += position_change[action]["WEST"]
				target_pose.position.y += position_change[action]["NORTH"]
				
			elif current_yaw < (-7.0 * math.pi / 8.0) and current_yaw > (7.0*math.pi /8.0):  # Facing WEST
				print "Case 4"
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x +=  position_change[action]["WEST"]

			elif current_yaw > (-7.0*math.pi / 8.0) and current_yaw < (-5.0*math.pi / 8.0):  # Facing SOUTHWEST
				print "Case 8"
				target_pose = copy.deepcopy(current_pose)
				target_pose.position.x += position_change[action]["WEST"]
				target_pose.position.y += position_change[action]["SOUTH"]
				
			PID(target_pose, "linear").publish_velocity()
			
		elif action == "TurnHalfCW" or action == "TurnHalfCCW":
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			yaw = euler[2]
			if action == "TurnHalfCW":
				target_yaw = yaw - ( math.pi / 4.0)
				if target_yaw < -math.pi:
					target_yaw += (math.pi * 2)
			else:
				target_yaw = yaw + ( math.pi / 4.0)
				if target_yaw >= (math.pi):
					target_yaw -= (math.pi * 2 )
			target_pose = Pose()
			target_pose.position = current_pose.position
			target_quat = Quaternion(*tf.transformations.quaternion_from_euler(euler[0],euler[1],target_yaw))
			target_pose.orientation = target_quat
			print target_pose.orientation
			PID(target_pose,"rotational").publish_velocity()

		elif action == 'TurnCW' or action == 'TurnCCW':
			print('Hiiii')
			current_pose = self.pose
			quat = (current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quat)
			yaw = euler[2]
			if action == "TurnCW":
				target_yaw = yaw - ( math.pi / 2.0)
				if target_yaw < -math.pi:
					target_yaw += (math.pi * 2)
			else:
				target_yaw = yaw + ( math.pi / 2.0)
				if target_yaw >= (math.pi):
					target_yaw -= (math.pi * 2 )
			target_pose = Pose()
			target_pose.position = current_pose.position
			target_quat = Quaternion(*tf.transformations.quaternion_from_euler(euler[0],euler[1],target_yaw))
			target_pose.orientation = target_quat
			print target_pose.orientation
			PID(target_pose,"rotational").publish_velocity()

		else:
			print "Invalid action"
			return
		if len(self.actions) == 0:
			self.status_publisher.publish(self.free)


	def pose_callback(self,data):
		self.pose = data.pose.pose

	

if __name__ == "__main__":
	try:
		moveTbot3()
	except rospy.ROSInterruptException:
		pass
