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


class StraightLine:
	VEL_COM_TOPIC = "/sensors/odometry/velocity/cmd"
	GOAL_TOPIC = "/path_segment"
	LIGHT_TOPIC = "/drivers/brainstem/cmd/update_tote_lights"
	global rospy

	def __init__(self, startPoint, endPoint):
		self.startPoint = startPoint
		self.endPoint = endPoint
		self.vel_sub = rospy.Subscriber(self.VEL_COM_TOPIC, Twist, self.velocityCmdCallback)
		self.goal_pub = rospy.Publisher(self.GOAL_TOPIC, Path, queue_size=2)
		self.light_pub = rospy.Publisher(self.LIGHT_TOPIC, MsgUpdateToteLights, queue_size=5)
		self.timeChange = 1.0
		self.redTop = True
		self.changeLights = 0

	def velocityCmdCallback(self, msg):
		if(rospy.get_time() - self.changeLights > self.timeChange):
			if(self.redTop):
				lightMsg1 = MsgUpdateToteLights()
				lightMsg2 = MsgUpdateToteLights()
				messageCreation(lightMsg1,1,[255,0,0,0],[255,0,0,0],[0,0,0],[26,1,0],1)
				self.light_pub.publish(lightMsg1)
				messageCreation(lightMsg2,1,[0,255,0,0],[0,255,0,0],[0,0,1],[26,1,1],1)
				self.light_pub.publish(lightMsg2)
			else:
				lightMsg1 = MsgUpdateToteLights()
				lightMsg2 = MsgUpdateToteLights()
				messageCreation(lightMsg1,1,[0,255,0,0],[0,255,0,0],[0,0,0],[26,1,0],1)
				self.light_pub.publish(lightMsg1)
				messageCreation(lightMsg2,1,[255,0,0,0],[255,0,0,0],[0,0,1],[26,1,1],1)
				self.light_pub.publish(lightMsg2)
			self.redTop = not self.redTop
			self.changeLights = rospy.get_time()

	def sendGoalFunc(self):
		path = Path()
		path.header.frame_id = 'map'
		for i in range(99):
			newPose = PoseStamped()
			newPose.header.seq = i
			newPose.header.frame_id = 'map'
			newPose.pose.position.x = self.startPoint[0] + (self.endPoint[0] - self.endPoint[0])/100.0
			newPose.pose.position.y = self.startPoint[1] + (self.endPoint[1] - self.endPoint[1])/100.0
			newPose.pose.position.z = 0
			newQuaternion = tf.transformations.quaternion_from_euler(0, 0,  math.tan((self.endPoint[1] - self.startPoint[1])/( .0001 + self.endPoint[0] - self.startPoint[0])))
			newPose.pose.orientation.x = 0
			newPose.pose.orientation.y = 0
			newPose.pose.orientation.z = newQuaternion[2]
			newPose.pose.orientation.w = newQuaternion[3]
			path.poses.append(newPose)
		self.goal_pub.publish(path)

if __name__ == '__main__':
	rospy.init_node('simple_loop', anonymous=True)
	startPoint = []
	startPoint[0] = 10 # x
	startPoint[1] = 10 # y

	
	startPoint = []
	endPoint[0] = 10 # x
	endPoint[1] = 10 # y



	hs = StraightLine(startPoint, endPoint)
	time.sleep(1.0)
	rate = rospy.Rate(200) # 10hz
	print "Here we go"
	
	hs.sendGoalFunc()
	rospy.spin()
