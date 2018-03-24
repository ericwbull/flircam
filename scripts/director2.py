#!/usr/bin/env python

import rospy
import time
import ImageCapture

from std_msgs.msg import String
from std_msgs.msg import UInt32
from flircam.msg import Detection

def ImageIdToString(imageId):
    return "{}/{}.{}".format(imageId.collectionNumber, imageId.frameNumber, imageId.serialNumber)


def DetectionReceived(data, directorNode):
    directorNode.DetectionReceived(data)

class Director:
    def __init__(self):
        self.count=0
        (imagePositions, rowAngles, colAngles) = self.getPositionList()
        flatAnglePos = tuple((25,0))
        servoDelay = 0.5
        cameraDelay = 0.5
        self.safeCountdown = 0
        self.imageCapture = ImageCapture.ImageCapture(flatAnglePos, imagePositions, rowAngles, colAngles, servoDelay, cameraDelay)
        
        rospy.Subscriber('detection', Detection, DetectionReceived, self)
        rospy.init_node('Director', anonymous=True)


    def getPositionList(self):
        posList = list()

        # Make a list of the image frames and angles
        colAngles = [h for h in range(-90,91,20)]
        rowAngles = [v for v in range(-90,-14,15)]
        
        for row in range(len(rowAngles)):
            for col in range(len(colAngles)):
                # Only one capture pointing straight down, which is row 0
                if (row==0):
                    if (col==0):
                        posList.append(tuple((row,col)))
                else:
                    posList.append(tuple((row,col)))

        return tuple((posList, rowAngles, colAngles))
                               
    def run(self):
#        time.sleep(60)
        while not rospy.is_shutdown():
#            time.sleep(120)
            # set to start processing near the flatfield position
            self.imageCapture.setNextFrame(46)
            self.imageCapture.requestAll()
            # start the image capture thread going and wait for it to finish
            # while waiting, the rospy subscriber may give imageCapture more requests
            self.imageCapture.doCollection(self.count % 5)
            self.count += 1

    
    def DetectionReceived(self, data):
        print "!detection imageId={} detectionCount={} detection={} safe={} error={}"\
        .format(ImageIdToString(data.imageId), data.detectionCount, data.detection, data.safe, data.error)
        if data.detection:
            # Set 5 second delay between frames
            delay = 5
            span = 2
            self.imageCapture.requestFrameAndNeighbors(data.imageId.frameNumber,span,delay)
            self.safeCountdown = 5
        if data.safe:
            if self.safeCountdown > 0:
                self.safeCountdown-=1
                if (self.safeCountdown == 0):
                    self.imageCapture.resetFrameDelay()
            
    # redirect the camera to the detection frame, and then each neighboring frame
    # collection ends when all of the following conditions are satisfied:
    # - we have imaged each of the frames at least once
    # - no detection in the last image of each frame
    # - we have completed all requested frames (even if all frames have no detection status, if there are still more frames in the queue,
    #   then we must continue collection until we empty the queue 

if __name__ == '__main__':
    try:
        director = Director()
        director.run()
    except rospy.ROSInterruptException:
        pass

