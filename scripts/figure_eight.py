#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import math
import sys
import time
import tf

radius = 5
offsetX = 48
offsetY = 7
class FigureEight:
	global radius
	global offsetX
	global offsetY
	VEL_COM_TOPIC = "/sensors/odometry/velocity/cmd"
	GOAL_TOPIC = "/path_segment"
	global rospy

	def __init__(self):
		self.vel_sub = rospy.Subscriber(self.VEL_COM_TOPIC, Twist, self.velCb)
		self.goal_pub = rospy.Publisher(self.GOAL_TOPIC, Path, queue_size=2)
		self.sendGoal = True
		self.firstLoop = True
		self.pathLoop = 0

	def sendGoalFunc(self):
		path = Path()
		path.header.seq = self.pathLoop
		path.header.frame_id = 'map'
		if(self.firstLoop):
			for i in range(95):
				newPose = PoseStamped()
				newPose.header.seq = i
				newPose.header.frame_id = 'map'
				newPose.pose.position.x = math.cos(i*3.14159/50.0) * radius + offsetX
				newPose.pose.position.y = math.sin(i*3.14159/50.0) * radius + offsetY
				newPose.pose.position.z = 0
				newQuaternion = tf.transformations.quaternion_from_euler(0, 0, i*3.14159/50.0 + 3.14159/2)
				newPose.pose.orientation.x = 0
				newPose.pose.orientation.y = 0
				newPose.pose.orientation.z = newQuaternion[2]
				newPose.pose.orientation.w = newQuaternion[3]
				path.poses.append(newPose)
		else:
			for i in range(95):
				newPose = PoseStamped()
				newPose.header.seq = i
				newPose.header.frame_id = 'map'
				newPose.pose.position.x = -1 * math.cos(i*3.14159/50.0) * radius + offsetX + 2 * radius
				newPose.pose.position.y = math.sin(i*3.14159/50.0) * radius + offsetY
				newPose.pose.position.z = 0
				newQuaternion = tf.transformations.quaternion_from_euler(0, 0, -1 * i*3.14159/50.0 + 3.14159/2)
				newPose.pose.orientation.x = 0
				newPose.pose.orientation.y = 0
				newPose.pose.orientation.z = newQuaternion[2]
				newPose.pose.orientation.w = newQuaternion[3]
				path.poses.append(newPose)
		self.goal_pub.publish(path)
		print "woop"
		self.sendGoal = False

	def velCb(self, msg):
		if(msg.linear.x == 0 and msg.angular.z == 0):
			self.sendGoal = True
			self.firstLoop =  not self.firstLoop
			self.pathLoop = self.pathLoop + 1
		if(self.sendGoal):
			self.sendGoalFunc()
		

if __name__ == '__main__':
	hs = FigureEight()
	rospy.init_node('figure_eight', anonymous=True)
	time.sleep(1.0)
	rate = rospy.Rate(200) # 10hz
	print "Here we go"
	
	hs.sendGoalFunc()
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
