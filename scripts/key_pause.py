#!/usr/bin/env python
import keyboard

from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import math
import sys
import time
import tf
from srslib_framework.msg import MsgSetOperationalState
import rospy



def talker():
    pub = rospy.Publisher('/drivers/brainstem/cmd/set_motion_state', MsgSetOperationalState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = MsgSetOperationalState()
        if keyboard.is_pressed('p'):  # if key 'q' is pressed 
            msg.operationalState.pause = True
            msg.state = True
            pub.publish(msg)
            print "woop"
        elif keyboard.is_pressed('q'):
            msg.operationalState.pause = True
            msg.state = False
            pub.publish(msg)
            print "boo"
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass