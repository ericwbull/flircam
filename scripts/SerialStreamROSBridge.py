#!/usr/bin/env python

import rospy
import SerialStream

from std_msgs.msg import String

def DataReceivedFromROSFabric(data, bridge):
    print "sending '{}'".format(data.data)
    bridge.sendToSerialStream(data.data)

# Transfers messages, bidirectionally, between the SerialStream.Server (the gateway.js web app is on the other side) and ROS (serial port hardware is on the other side)
class SerialStreamROSBridge:
    def __init__(self):
        self.serialStream = SerialStream.Server()
        self.pub = rospy.Publisher('commands_to_nodes', String, queue_size=10)
        rospy.Subscriber('telemetry_from_nodes', String, DataReceivedFromROSFabric, self)
        rospy.init_node('SerialStreamROSBridge', anonymous=True)
        
    def run(self):
        while True:
            data = self.serialStream.receive()
            self.pub.publish(data)

    def sendToSerialStream(self,data):
        self.serialStream.send(data)

if __name__ == '__main__':
    bridge = SerialStreamROSBridge()
    bridge.run()
