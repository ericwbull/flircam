#!/usr/bin/env python

import rospy
import pigpio

from std_msgs.msg import String
from flircam.msg import PanTiltPos

def CommandReceived(data, controller):
    print "horizontal={0} vertical={1}".format(data.horizontalAngle,data.verticalAngle)
    controller.SetHorizontal(data.horizontalAngle)
    controller.SetVertical(data.verticalAngle)

# Transfers messages, bidirectionally, between the SerialStream.Server (the gateway.js web app is on the other side) and ROS (serial port hardware is on the other side)
class PanTiltController:
    def __init__(self):
        self.pi = pigpio.pi()
        rospy.Subscriber('pantilt_position_command', PanTiltPos, CommandReceived, self)
        rospy.init_node('PanTiltController', anonymous=True)
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            
    def SetHorizontal(self,angle):
        # horizontal: 530 is 90 degrees is 2430 is -90 degrees
        slope = (530.0-2430.0)/180.0
        mid = (2430.0+530.0)/2.0
        servoPos = slope*angle+mid
        print "horizontal servo={}".format(servoPos)
        # ToDO: clip servoPos to limits
        self.pi.set_servo_pulsewidth(18, servoPos)
        
    def SetVertical(self,angle):
        # vertial: 1500 is -90, 600 is 0 degrees
        servoPos = -10*angle+600
        print "vertical servo={}".format(servoPos)
        # ToDO: clip servoPos to limits
        self.pi.set_servo_pulsewidth(13, servoPos)


if __name__ == '__main__':
    controller = PanTiltController()
    controller.run()
