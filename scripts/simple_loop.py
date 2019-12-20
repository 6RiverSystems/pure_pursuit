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




def messageCreation(message, cmd, sC, eC, sS, eS, freq):
    message.lightCmd = cmd
    message.startColor.r = sC[0]
    message.startColor.g = sC[1]
    message.startColor.b = sC[2]
    message.startColor.a = sC[3]
    message.endColor.r = eC[0]
    message.endColor.g = eC[1]
    message.endColor.b = eC[2]
    message.endColor.a = eC[3]
    message.startSegment.x = sS[0]
    message.startSegment.y = sS[1]
    message.startSegment.z = sS[2]
    message.endSegment.x = eS[0]
    message.endSegment.y = eS[1]
    message.endSegment.z = eS[2]
    message.frequency = freq


class FigureEight:
	VEL_COM_TOPIC = "/sensors/odometry/velocity/cmd"
	GOAL_TOPIC = "/path_segment"
	LIGHT_TOPIC = "/drivers/brainstem/cmd/update_tote_lights"
	global rospy

	def __init__(self,offX, offY, rad):
		self.offsetX = offX
		self.offsetY = offY
		self.radius = rad
		self.vel_sub = rospy.Subscriber(self.VEL_COM_TOPIC, Twist, self.velCb)
		self.goal_pub = rospy.Publisher(self.GOAL_TOPIC, Path, queue_size=2)
		self.light_pub = rospy.Publisher(self.LIGHT_TOPIC, MsgUpdateToteLights, queue_size=5)
		self.sendGoal = True
		self.firstLoop = True
		self.pathLoop = 0
		self.timeChange = 1.0
		self.redTop = True
		self.changeLights = 0

	def sendGoalFunc(self):
		path = Path()
		path.header.seq = self.pathLoop
		path.header.frame_id = 'map'
		for i in range(97):
			newPose = PoseStamped()
			newPose.header.seq = i
			newPose.header.frame_id = 'map'
			newPose.pose.position.x = math.cos(i*3.14159/50.0) * self.radius + self.offsetX
			newPose.pose.position.y = math.sin(i*3.14159/50.0) * self.radius + self.offsetY
			newPose.pose.position.z = 0
			newQuaternion = tf.transformations.quaternion_from_euler(0, 0, i*3.14159/50.0 + 3.14159/2)
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

		

if __name__ == '__main__':
	rospy.init_node('simple_loop', anonymous=True)
	offX = rospy.get_param("simple_loop/offsetX", 40)
	offY = rospy.get_param("simple_loop/offsetY", 7)
	rad = rospy.get_param("simple_loop/radius", 2)
	hs = FigureEight(offX, offY, rad)
	time.sleep(5.0)
	rate = rospy.Rate(200) # 10hz
	print "Here we go"
	
	hs.sendGoalFunc()
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
