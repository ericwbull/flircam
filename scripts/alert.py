#!/usr/bin/env python

import time
import rospy
import subprocess
import threading
import struct
import flircam_util as fcutil
from flircam.msg import Detection
from flircam.msg import DownlinkData
import sys


def DetectionReceived(data, alertNode):
    alertNode.DetectionReceived(data)

    
# Transfers messages, bidirectionally, between the SerialStream.Server (the gateway.js web app is on the other side) and ROS (serial port hardware is on the other side)
class AlertNode:
    def __init__(self):
        self.frameCount = 0
        self.beginCollection = True
        self.detectionCount = [0 for x in range(70)]
        self.detectionTotalFrameCount = 0

        rospy.init_node('AlertNode', anonymous=True)
        rospy.Subscriber('detection', Detection, DetectionReceived, self)
        self.pubDownlink = rospy.Publisher('downlink', DownlinkData, queue_size=200)

    def DetectionReceived(self,data):
        print "imageId={} detectionCount={} detection={} safe={} error={}".format(fcutil.ImageIdToString(data.imageId), data.detectionCount, data.detection, data.safe, data.error)
        sys.stdout.flush()

        if (self.beginCollection):
            self.beginCollection = False;
            self.detectionCount = [0 for x in range(70)]
            self.detectionTotalFrameCount = 0
            # beginning of collecion init here

        alert = False
        if (data.imageId.endCollection):
            self.beginCollection = True
        
        else:
            if data.detection:
                self.detectionTotalFrameCount += 1 
                self.detectionCount[data.imageId.frameNumber] += 1
                
                if self.detectionCount[data.imageId.frameNumber] > 2:
                    alert = True

        self.frameCount += 1
        if (alert):
            self.sendAlertToDownlink(data)
            
    def run(self):
        rate = rospy.Rate(10)
        print "Ready"
        rospy.spin()

    def sendAlertToDownlink(self,detection):
        imageId = detection.imageId
        detectionPixelCount = detection.detectionCount
        thisFrameDetectionCount = self.detectionCount[imageId.frameNumber]
        fileData = fcutil.ReadCurrentImageFile(detection.imageId)
        minPixel = fileData.min()
        maxPixel = fileData.max()
        avgPixel = fileData.mean()
        downlink = DownlinkData()
        downlink.streamId = fcutil.StreamID.RETURN_ALERT.value
        data = bytearray(chr(0)*20)
        downlink.verifyReceipt = True

        struct.pack_into('>HHHHHHLHH', data, 0, imageId.collectionNumber, imageId.frameNumber, imageId.serialNumber, \
                         minPixel, maxPixel, avgPixel, \
                         detectionPixelCount, thisFrameDetectionCount, self.detectionTotalFrameCount)
        
        downlink.data = list(data)
        print "Alert: {} min:{} max:{} avg:{} pixelCount:{} thisFrameCount:{} totalCount:{}".format(fcutil.ImageIdToString(imageId),minPixel,maxPixel,avgPixel,detectionPixelCount, thisFrameDetectionCount, self.detectionTotalFrameCount)
        self.pubDownlink.publish(downlink)
                           
if __name__ == '__main__':
    alertNode = AlertNode()
    alertNode.run()
