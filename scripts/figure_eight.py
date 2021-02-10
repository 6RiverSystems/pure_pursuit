#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header, Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import math
import sys
import time
import tf
from srslib_framework.msg import MsgUpdateToteLights, PipeLoopApproachPath


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


class FigureEight:
	VEL_COM_TOPIC = "/sensors/odometry/velocity/cmd"
	GOAL_TOPIC = "/pure_pursuit/goal"
	LIGHT_TOPIC = "/drivers/brainstem/cmd/update_tote_lights"
	ARRIVED_TOPIC = "/pure_pursuit/arrived"
	global rospy

	def __init__(self, offsetX, offsetY, radius):
		self.offsetX = offsetX
		self.offsetY = offsetY
		self.radius = radius
		self.vel_sub = rospy.Subscriber(self.VEL_COM_TOPIC, Twist, self.velocityCmdCallback)
		self.arrival_sub = rospy.Subscriber(self.ARRIVED_TOPIC, Bool, self.arrivedCallback)
		self.goal_pub = rospy.Publisher(self.GOAL_TOPIC, PipeLoopApproachPath, queue_size=2)
		self.light_pub = rospy.Publisher(self.LIGHT_TOPIC, MsgUpdateToteLights, queue_size=5)
		self.sendGoal = True
		self.firstLoop = True
		self.pathLoop = 0
		self.timeChange = 1.0
		self.redTop = True
		self.changeLights = 0

	def sendGoalFunc(self):
		approach_msg = PipeLoopApproachPath()
		approach_msg.header.frame_id = 'map'
		path = Path()
		path.header.seq = self.pathLoop
		path.header.frame_id = 'map'
		if(self.firstLoop):
			for i in range(95):
				newPose = PoseStamped()
				newPose.header.seq = i
				newPose.header.frame_id = 'map'
				newPose.pose.position.x = math.cos(i*math.pi/50.0) * self.radius + self.offsetX
				newPose.pose.position.y = math.sin(i*math.pi/50.0) * self.radius + self.offsetY
				newPose.pose.position.z = 0
				newQuaternion = tf.transformations.quaternion_from_euler(0, 0, i*math.pi/50.0 + math.pi/2)
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
				newPose.pose.position.x = -1 * math.cos(i*math.pi/50.0) * self.radius + self.offsetX + 2 * self.radius
				newPose.pose.position.y = math.sin(i*math.pi/50.0) * self.radius + self.offsetY
				newPose.pose.position.z = 0
				newQuaternion = tf.transformations.quaternion_from_euler(0, 0, -1 * i*math.pi/50.0 + math.pi/2)
				newPose.pose.orientation.x = 0
				newPose.pose.orientation.y = 0
				newPose.pose.orientation.z = newQuaternion[2]
				newPose.pose.orientation.w = newQuaternion[3]
				path.poses.append(newPose)
		
		approach_msg.path = path
		approach_msg.max_linear_velocity = 1.0
		approach_msg.max_angular_velocity = 0.6
		self.goal_pub.publish(approach_msg)
		print "woop - sent new goal (path)"
		self.sendGoal = False

	def velocityCmdCallback(self, msg):
		if self.sendGoal:
			# self.sendGoal = False
			self.firstLoop =  not self.firstLoop
			self.pathLoop = self.pathLoop + 1
		if self.sendGoal:
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
	
	def arrivedCallback(self, msg):
		self.sendGoal = msg.data
		# pass

		

if __name__ == '__main__':
	rospy.init_node('figure_eight', anonymous=True)
	offX = rospy.get_param("figure_eight/offsetX", 40)
	offY = rospy.get_param("figure_eight/offsetY", 7)
	rad = rospy.get_param("figure_eight/radius", 4.5)
	hs = FigureEight(offX, offY, rad)
	time.sleep(5.0)
	rate = rospy.Rate(200) # 10hz
	print "Here we go"
	
	hs.sendGoalFunc()
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
