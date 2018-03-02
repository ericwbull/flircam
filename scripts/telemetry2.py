#!/usr/bin/env python

import time
import rospy
import subprocess
import threading
import struct
import flircam_util as fcutil
from flircam.msg import Detection
from flircam.msg import ImageRequest
from flircam.msg import DownlinkData
from flircam.msg import Block
import sys


def ImageRequestReceived(request, telemetryNode):
    telemetryNode.doSendImage(request)
    
def DetectionReceived(data, telemetryNode):
    telemetryNode.DetectionReceived(data)

    
# Transfers messages, bidirectionally, between the SerialStream.Server (the gateway.js web app is on the other side) and ROS (serial port hardware is on the other side)
class TelemetryNode:
    def __init__(self):
        self.count = 0

        # these are the latched safe/detect bits for the current collection only.
        # all bits initially False.  They latch True until end of collection 
        self.detectionBits = []
        self.safeBits = []
        self.beginCollection = True
#        self.errorBits = [False for x in range(60)]

        rospy.init_node('TelemetryNode', anonymous=True)
        rospy.Subscriber('image_request', ImageRequest, ImageRequestReceived, self)
        rospy.Subscriber('detection', Detection, DetectionReceived, self)
        self.pubDownlink = rospy.Publisher('downlink', DownlinkData, queue_size=200)

    def DetectionReceived(self,data):
        print "imageId={} detectionCount={} detection={} safe={} error={}".format(fcutil.ImageIdToString(data.imageId), data.detectionCount, data.detection, data.safe, data.error)
        sys.stdout.flush()

        if (self.beginCollection):
            self.beginCollection = False;
            self.detectionBits = [False for x in range(70)]
            self.safeBits = [False for x in range(70)]

        if (data.imageId.endCollection):
        # detect bits will be reset when the next detection is received
            self.beginCollection = True
            # ToDO: Send to downlink? Does the gateway need to know that the collection ended?
        
        else:
            bitNum = data.imageId.frameNumber - 1
            if data.safe:
                self.safeBits[bitNum]=True
            else:
                self.safeBits[bitNum]=False

            if data.detection:
                self.detectionBits[bitNum]=True
            else:
                self.detectionBits[bitNum]=False
        
            self.count += 1

#            self.sendSafeDetectArrayToDownlink(data.imageId)
            
    def run(self):
        rate = rospy.Rate(10)
        print "Ready"
        rospy.spin()

    def doSendImage(self, r):
        print "SendImageRequest: id={} type={} blockSize={} rangeCount={}".format(r.id, r.type, r.blockSize, len(r.blockList))
        for b in r.blockList:
            self.SendImageBlockRange(r.id,r.type,r.blockSize,b.start,b.count)
        sys.stdout.flush()
            
    def SendImageBlockRange(self, imageId, imageType, blockSize, start, count):
        print "SendImageBlockRange start={} count={}".format(start, count)
        for i in range(start, start+count):
            self.SendImageBlock(imageId, imageType, blockSize, i)

    def SendImageBlock(self, imageId, imageType, size, num):
        print "SendImageBlock imageId={}  type={} size={} num={}".format(imageId, imageType, size, num)
        if (imageType == 1):
            self.SendCurrentImageBlock(imageId, size, num)
        elif (imageType == 2):
            self.SendBaselineImageBlock(imageId, size, num)
        elif (imageType == 3):
            self.SendDetectionImageBlock(imageId, size, num)

    def SendCurrentImageBlock(self, imageId, size, num):
        self.SendBlockFromFile(GetCurrentImageFileName(imageId), num, size, 60*80*2)

    def SendBaselineImageBlock(self, imageId, size, num):
        self.SendBlockFromFile(GetBaselineImageFileName(imageId), num, size, 60*80*2)
        
    def SendDetectionImageBlock(self, imageId, size, num):
        self.SendBlockFromFile(GetDetectionImageFileName(imageId), num, size, 60*80/8)

    def SendBlockFromFile(self, fileName, num, size, endPos):
        fileData = fcutil.ReadBlockFromFile(filename, num, size, endPos)
        downlink = DownlinkData()
        downlink.streamId = StreamID.RETURN_IMAGE.value
        downlink.verifyReceipt = False
        downlink.data = [num]
        downlink.data.extend(map(ord,fileData))
#        time.sleep(1)
        self.pubDownlink.publish(downlink)

#    def sendDetectionMessageToDownlink(self, msg):
#        downlink = DownlinkData()
#        downlink.streamId = StreamID.RETURN_DETECTION.value
#        downlink.data = bytearray(chr(0)*50)
#        downlink.verifyReceipt = False
        
#        struct.pack_into('>IIBB', downlink.data, 0, msg.imageId, msg.signalStrength, msg.detection, msg.safe)
#        i = 0
#        for h in msg.signalHistogram:
#            struct.pack_into('>I', downlink.data, 10 + 4 * i, h)
#            i += 1
#        self.pubDownlink.publish(downlink)

    def sendSafeDetectArrayToDownlink(self,imageId):
        downlink = DownlinkData()
        downlink.streamId = StreamID.RETURN_DETECTION_ARRAY.value
        data = bytearray(chr(0)*24)
        downlink.verifyReceipt = False

        struct.pack_into('>HHH', data, 0, imageId.collectionNumber, imageId.frameNumber, imageId.serialNumber)
        
        detectBytes = BoolListToByteList(self.detectionBits)
        i = 6
        for b in detectBytes:
            data[i] = chr(b)
            i += 1

        safeBytes = BoolListToByteList(self.safeBits)
        for b in safeBytes:
            data[i] = chr(b)
            i += 1
        downlink.data = list(data)
        self.pubDownlink.publish(downlink)
                           
if __name__ == '__main__':
    telemetry = TelemetryNode()
    telemetry.run()
#    while True:
#       raw_input("Press enter")
#       telemetry.sendToSerialPort(data)
