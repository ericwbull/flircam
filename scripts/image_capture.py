#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import String
from std_msgs.msg import UInt32
from flircam.msg import PanTiltPos


# Transfers messages, bidirectionally, between the SerialStream.Server (the gateway.js web app is on the other side) and ROS (serial port hardware is on the other side)
class ImageCapture:
    def __init__(self):
        self.pubPanTilt = rospy.Publisher('pantilt_position_command', PanTiltPos, queue_size=10)
        self.pubSaveImage = rospy.Publisher('save_image', UInt32, queue_size=10)
        rospy.init_node('ImageCapture', anonymous=True)
        
    def run(self):
        frameNum=1
        pantilt=PanTiltPos()
        pantilt.horizontalAngle=-90
        pantilt.verticalAngle=-90
        self.pubPanTilt.publish(pantilt)
        time.sleep(1.5)
        sweepUp=True
        
        for x in range(-90,91,20):
            if sweepUp:
                yrange = range(-90,1,15)
                sweepUp=False
            else:
                yrange = range(0,-91,-15)
                sweepUp=True
                    
            for y in yrange:
                pantilt.horizontalAngle=x
                pantilt.verticalAngle=y
                print "x={} y={} frame={}".format(x,y,frameNum)
                self.pubPanTilt.publish(pantilt)
                time.sleep(0.25)
                self.pubSaveImage.publish(frameNum)
                frameNum = frameNum+1
            

if __name__ == '__main__':
    controller = ImageCapture()
    controller.run()
