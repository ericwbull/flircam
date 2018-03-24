#!/usr/bin/env python

import rospy
import time

import threading
from flircam.msg import ImageId

class FrameGrabberTest:
    def __init__(self):
        self.pubSaveImage = rospy.Publisher('save_image', ImageId, queue_size=10)
        self.collectionNum = 0
        rospy.init_node('FrameGrabberTest', anonymous=True)

    def run(self):

        # flatfield
        time.sleep(1)
        self.publishSaveImage(0,0)
        time.sleep(5)

        for serialNum in range(0,40):
            self.publishSaveImage(1,serialNum)
            time.sleep(1)
                
        # send endCollection
        self.publishEndCollection()
        
    def publishSaveImage(self, frameNum, serialNum):
        imageId = ImageId()
        imageId.collectionNumber=self.collectionNum
        imageId.frameNumber=frameNum
        imageId.serialNumber=serialNum
        imageId.endCollection=False
        self.pubSaveImage.publish(imageId)
        print "{}: frame={}".format(self.collectionNum, frameNum)

    def publishEndCollection(self):
        imageId = ImageId()
        imageId.collectionNumber=self.collectionNum
        imageId.frameNumber=0
        imageId.serialNumber=0
        imageId.endCollection=True
        self.pubSaveImage.publish(imageId)
        print "{}: end collection".format(self.collectionNum)

        
if __name__ == '__main__':
    try:
        test = FrameGrabberTest()
        test.run()
    except rospy.ROSInterruptException:
        pass
