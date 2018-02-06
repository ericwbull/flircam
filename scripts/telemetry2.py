#!/usr/bin/env python

import time
import rospy
import subprocess
import threading
import struct
from enum import Enum
from flircam.msg import Detection
from flircam.msg import ImageRequest
from flircam.msg import DownlinkData
from flircam.msg import Block
import sys

class StreamID(Enum):
    RETURN_DETECTION = 4
    RETURN_DETECTION_ARRAY = 5
    REQUEST_WIFI = 7
    REQUEST_IMAGE = 8
    REQUEST_DETECTION_ARRAY = 10
    RETURN_IMAGE = 9
    RETURN_PING = 66
    REQUEST_PING = 65
    REQUEST_SHUTDOWN = 11

def ImageRequestReceived(request, telemetryNode):
    telemetryNode.doSendImage(request)

def DetectionReceived(data, telemetryNode):
    print "imageId={} signalStength={} detection={} safe={} signalHistogram={}".format(data.imageId, data.signalStrength, data.detection, data.safe, ','.join(str(x) for x in data.signalHistogram))

#    telemetryNode.sendDetectionMessageToSerialPort(data)

    bitNum = data.imageId - 1
    if data.safe:
        telemetryNode.safeBits[bitNum]=True
    else:
        telemetryNode.safeBits[bitNum]=False

    if data.detection:
        telemetryNode.detectionBits[bitNum]=True
    else:
        telemetryNode.detectionBits[bitNum]=False
        
    telemetryNode.count += 1

    telemetryNode.sendSafeDetectArrayToDownlink(data.imageId)


def BoolListToByteList(mylist):
    weight = 1
    value = 0
    byteList = []
    for b in mylist:
        if b:
            value += weight

        if weight == 128:
            # finished the uppermost bit
            weight = 1
            byteList.append(value)
            value = 0
        else:
            # go to next bit
            weight *= 2

    if weight > 1:
        # add last byte to list
        byteList.append(value)
        
    return byteList

def GetCurrentImageFileName(imageId):
    majorId = imageId >> 65536
    minorId = imageId & 0xffff
    return "/tmp/flircam/{0}/{1}".format(majorId,minorId)
def GetBaselineImageFileName(imageId):
    return "{0}.baseline".format(GetCurrentImageFileName(imageId))
def GetDetectionImageFileName(imageId):
    return "{0}.detection".format(GetCurrentImageFileName(imageId))
    

# Transfers messages, bidirectionally, between the SerialStream.Server (the gateway.js web app is on the other side) and ROS (serial port hardware is on the other side)
class TelemetryNode:
    def __init__(self):
        self.count = 0
        self.detectionBits = [False for x in range(70)]
        self.safeBits = [False for x in range(70)]

        rospy.init_node('TelemetryNode', anonymous=True)
        rospy.Subscriber('image_request', ImageRequest, ImageRequestReceived, self)
        rospy.Subscriber('detection', Detection, DetectionReceived, self)
        self.pubDownlink = rospy.Publisher('downlink', DownlinkData, queue_size=200)
        
    def run(self):
        rate = rospy.Rate(10)
        print "Ready"
        rospy.spin()

    def doSendImage(self, r):
        print "SendImageRequest: id={} type={} blockSize={} rangeCount={}".format(r.id, r.type, r.blockSize, len(r.blockList))
        for b in r.blockList:
            self.SendImageBlockRange(r.id,r.type,r.blockSize,b.start,b.count)

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
        imageFile = file(fileName,"rb")
        pos = num*size
        imageFile.seek(pos)
        if (pos + size > endPos):
            size = endPos - pos

        print "SendDetectionBlock pos={} size={}".format(pos,size)
        fileData = imageFile.read(size)
        downlink = DownlinkData()
        downlink.streamId = StreamID.RETURN_IMAGE.value
        downlink.verifyReceipt = False
        downlink.data = [num]
        downlink.data.extend(map(ord,fileData))
        time.sleep(1)
        self.pubDownlink.publish(downlink)

    def sendDetectionMessageToDownlink(self, msg):
        downlink = DownlinkData()
        downlink.streamId = StreamID.RETURN_DETECTION.value
        downlink.data = bytearray(chr(0)*50)
        downlink.verifyReceipt = False
        
        struct.pack_into('>IIBB', downlink.data, 0, msg.imageId, msg.signalStrength, msg.detection, msg.safe)
        i = 0
        for h in msg.signalHistogram:
            struct.pack_into('>I', downlink.data, 10 + 4 * i, h)
            i += 1
        self.pubDownlink.publish(downlink)

    def sendSafeDetectArrayToDownlink(self,imageId):
        downlink = DownlinkData()
        downlink.streamId = StreamID.RETURN_DETECTION_ARRAY.value
        data = bytearray(chr(0)*22)
        downlink.verifyReceipt = False

        struct.pack_into('>I', data, 0, imageId)
        
        detectBytes = BoolListToByteList(self.detectionBits)
        i = 4
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
