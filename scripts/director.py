#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import String
from std_msgs.msg import UInt32
from flircam.msg import PanTiltPos


class ImageCapture:
    def __init__(self):
        self.pubPanTilt = rospy.Publisher('pantilt_position_command', PanTiltPos, queue_size=10)
        self.pubSaveImage = rospy.Publisher('save_image', UInt32, queue_size=10)
        self.count=0
        rospy.init_node('ImageCapture', anonymous=True)
        
    def run(self):
#        time.sleep(30)
        self.acquireFlatfield()
        while True:
#           time.sleep(120)
            raw_input("Press enter")
            self.pointAndCapture(0,0,92)
#            self.sweep()
            self.count += 1
        
    def capture(self, frameNum):
        self.pubSaveImage.publish(frameNum)
        print "{}: frame={}".format(self.count, frameNum)
        
    def point(self, h, v):
        pantilt=PanTiltPos()
        pantilt.horizontalAngle=h
        pantilt.verticalAngle=v
        self.pubPanTilt.publish(pantilt)
        print "{}: x={} y={}".format(self.count, h,v)
        time.sleep(1.5)

    def pointAndCapture(self, h, v, frm):
        self.point(h, v)
        self.capture(frm)
        
    def acquireFlatfield(self):
        self.point(0,0)
        self.pointAndCapture(0,25,0)
        self.point(0,0)

    def sweep(self):
        frameNum = 1
        self.point(-90,-90)
        time.sleep(5)
        sweepUp=True
        
        for x in range(-90,91,20):
            if sweepUp:
                yrange = range(-90,1,15)
                sweepUp=False
            else:
                yrange = range(0,-91,-15)
                sweepUp=True
                    
            for y in yrange:
                self.pointAndCapture(x,y,frameNum)
                frameNum += 1
        

if __name__ == '__main__':
    try:
        controller = ImageCapture()
        controller.run()
    except rospy.ROSInterruptException:
        pass

