#!/usr/bin/env python

import rospy
import time

import threading
from flircam.msg import ImageId

# Image capture publishes servo motion and save_image request commands.
# At init it is told the row and column positions of all frames.  It automatically associates a unique
# frame number with each of the row column positions.  Frame numbers are sequential starting from 1 (use a row first, column second, sorting to number the frames).
# The client specifies a row column position of the flatfield frame.  Frame number 0 is assigned to the flatfield frame.
# The flatfield frame is automatically processed at the beginning of the collection, and every so often.
#
# The interface provides methods for the client to request regions or individual cells of the grid to be imaged. Note flatfield is automatic after it is configured at init.
# These requests go into a collection, and control returns immediately to the caller.
# If a cell is requested and it's already in the collection to be processed, it is not added again.  Implement the collection as a bit flag for each cell.  The flag
# is raised when a request comes in and it is lowered when the request is processed.
# 
# An asynchronous thread
# processes the requests in the collection.  There's no requirement to process first-in-first-out.
# The algorithm should try to minimize servo movement.  Try processing the nearest image next.
# At init the client also specifies a time duration that ends collection.  If there's nothing to process
# for the specified duration, then an endCollection message is published to the save_image stream.
#
# The client monitors ImageCapture status to know when it has ended the collection.  The client can set a
# new collection number which is latched when the next image is processed, otherwise the collection number
# is automatically incremented.  ImageCapture sets the next pending collection number at the same time
# that it latches the current collection number.  


class FrameGrabberTest:
    def __init__(self):
        self.pubSaveImage = rospy.Publisher('save_image', ImageId, queue_size=10)
        self.collectionNum = 0
        rospy.init_node('FrameGrabberTest', anonymous=True)

    # This is the asynchronous thread that keeps processing requests until endCollection, and then it exits.
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
