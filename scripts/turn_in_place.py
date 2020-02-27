#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import math
import sys
import time
import tf
from srslib_framework.msg import MsgUpdateToteLights




def messageCreation(message, cmd, startColor, endColor, startSegment, endSegment, freq):
    message.lightCmd = cmd
    message.startColor.r = startColor[0]
    message.startColor.g = startColor[1]
    message.startColor.b = startColor[2]
    message.startColor.a = startColor[3]
    message.endColor.r = endColor[0]
    message.endColor.g = endColor[1]
    message.endColor.b = endColor[2]
    message.endColor.a = endColor[3]
    message.startSegment.x = startSegment[0]
    message.startSegment.y = startSegment[1]
    message.startSegment.z = startSegment[2]
    message.endSegment.x = endSegment[0]
    message.endSegment.y = endSegment[1]
    message.endSegment.z = endSegment[2]
    message.frequency = freq


class TurnInPlace:
	VEL_COM_TOPIC = "/sensors/odometry/velocity/cmd"
	GOAL_TOPIC = "/path_segment"
	LIGHT_TOPIC = "/drivers/brainstem/cmd/update_tote_lights"
	MAP_POSE_TOPIC = "/map_pose"
	global rospy

	def __init__(self):
		self.loopNum = 0
		self.sendGoal = True
		self.pathLoop = 0
		self.timeChange = 1.0
		self.redTop = True
		self.changeLights = 0
		self.mapPoseX = 0
		self.mapPosey = 0
		self.light_pub = rospy.Publisher(self.LIGHT_TOPIC, MsgUpdateToteLights, queue_size=5)
		self.vel_sub = rospy.Subscriber(self.VEL_COM_TOPIC, Twist, self.velocityCmdCallback)
		self.map_pose_sub = rospy.Subscriber(self.MAP_POSE_TOPIC, PoseStamped, self.mapPoseCallback)
		self.goal_pub = rospy.Publisher(self.GOAL_TOPIC, Path, queue_size=2)
		
		

	def sendGoalFunc(self):
		path = Path()
		path.header.seq = self.pathLoop
		path.header.frame_id = 'map'
		for i in range(1):
			newPose = PoseStamped()
			newPose.header.seq = i
			newPose.header.frame_id = 'map'
			newPose.pose.position.x = self.mapPoseX
			newPose.pose.position.y = self.mapPosey
			newPose.pose.position.z = 0
			newQuaternion = tf.transformations.quaternion_from_euler(0, 0, self.loopNum * math.pi/2 + (i+1)*math.pi)
			newPose.pose.orientation.x = 0
			newPose.pose.orientation.y = 0
			newPose.pose.orientation.z = newQuaternion[2]
			newPose.pose.orientation.w = newQuaternion[3]
			path.poses.append(newPose)
		self.goal_pub.publish(path)
		self.sendGoal = False

	def velocityCmdCallback(self, msg):
		if(msg.linear.x == 0 and msg.angular.z == 0):
			self.sendGoal = True
			self.loopNum = self.loopNum + 1
			if self.loopNum == 4:
				self.loopNum = 0
			self.pathLoop = self.pathLoop + 1
		if(self.sendGoal):
			self.sendGoalFunc()
	def mapPoseCallback(self, msg):
		self.mapPoseX = msg.pose.position.x
		self.mapPosey = msg.pose.position.y

		

if __name__ == '__main__':
	rospy.init_node('simple_loop', anonymous=True)
	time.sleep(2.0)
	hs = TurnInPlace()
	time.sleep(3.0)
	rate = rospy.Rate(20) # 10hz
	print "Here we go"
	
	hs.sendGoalFunc()
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
