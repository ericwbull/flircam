#!/usr/bin/env python

import time
import rospy
from beginner_tutorials.msg import NodeBytes
    
pub = rospy.Publisher('byte_commands_to_nodes', NodeBytes, queue_size=1)
rospy.init_node('Shutdown', anonymous=True)
time.sleep(1)
pubMsg = NodeBytes()
pubMsg.node = 5
pubMsg.data = [11]
pub.publish(pubMsg)

