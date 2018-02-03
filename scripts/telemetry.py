#!/usr/bin/env python

import time
import serial
import rospy
import subprocess
import threading
import struct
from enum import Enum
from flircam.msg import Detection
    
SERIAL_PORT='/dev/ttyS0'
# Radio rate is about 55000.
# It will be trouble if we send data to moteino faster than it can transmit it.
BAUD_RATE=38400

class StreamID(Enum):
    RETURN_DETECTION = 4
    RETURN_DETECTION_ARRAY = 5
    REQUEST_WIFI = 7
    REQUEST_IMAGE = 8
    REQUEST_DETECTION_ARRAY = 10
    RETURN_IMAGE = 9
    RETURN_PING = 66
    REQUEST_PING = 65

def DetectionReceived(data, telemetryNode):
    print "imageId={} signalStength={} detection={} safe={} signalHistogram={}".format(data.imageId, data.signalStrength, data.detection, data.safe, ','.join(str(x) for x in data.signalHistogram))

    telemetryNode.sendDetectionMessageToSerialPort(data)

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

# Don't send the detection or safe array unless requested
#    if (0 == (telemetryNode.count % 10)):
#        telemetryNode.sendSafeDetectArrayToSerialPort()


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


# Transfers messages, bidirectionally, between the SerialStream.Server (the gateway.js web app is on the other side) and ROS (serial port hardware is on the other side)
class TelemetryNode:
    def __init__(self):
        serialIsOpen = False
        self.count=0
        while not serialIsOpen:
            try:
                self.ser = serial.Serial(SERIAL_PORT, baudrate=BAUD_RATE)
                serialIsOpen = True
            except serial.SerialException:
                print "Serial port did not open"
                pass
        self.detectionBits = [False for x in range(70)]
        self.safeBits = [False for x in range(70)]
                                                                        
        rospy.Subscriber('detection', Detection, DetectionReceived, self)
        rospy.init_node('TelemetryNode', anonymous=True)
        self.serialSendLock = threading.Lock()
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                data = self.readMessageFromSerialPort()
                streamid = StreamID(data[0])
                self.doMessageByStream(streamid, data[1:])
            except struct.error:
                pass
            rate.sleep()

    def doPing(self,data):
        print "Ping={}".format(data)
        self.sendDataToSerialPort(StreamID.RETURN_PING, data)
        
    def doWifiOnOff(self,data):
        print "WifiReqest={}".format(data)
        on = data[0]
        if on:
            cmd = "block"
        else:
            cmd = "unblock"
        subprocess.call(["rfkill", cmd, "0"])

    def doSendImage(self,data):
        imageId = data[0]
        imageType = data[1]
        blockSize = data[2]
        rangeListSize = data[3]
        print "SendImageRequest: id={} type={} blockSize={} rangeCount={}".format(imageId, imageType, blockSize, rangeListSize)
        rangeData = data[4:]
        for i in range(0, rangeListSize):
            startBlock = rangeData[i*2]
            blockCount = rangeData[i*2 + 1]
            self.SendImageBlockRange(imageId, imageType, blockSize, startBlock, blockCount)

    def SendImageBlockRange(self, imageId, imageType, blockSize, start, count):
        print "SendImageBlockRange start={} count={}".format(start, count)
        for i in range(start, start+count):
            self.SendImageBlock(imageId, imageType, blockSize, i)

    def SendImageBlock(self, imageId, imageType, size, num):
        if (imageType == 1):
            self.SendCurrentImageBlock(imageId, size, num)
        elif (imageType == 2):
            self.SendBaselineImageBlock(imageId, size, num)
        elif (imageType == 3):
            self.SendDetectionImageBlock(imageId, size, num)

    def GetCurrentImageFileName(imageId):
        majorId = imageId >> 65536
        minorId = imageId & 0xffff
        return "/tmp/flircam/{0}/{1}".format(majorId,minorId)
    def GetBaselineImageFileName(imageId):
        return "{0}.baseline".format(GetCurrentImageFileName(imageId))
    def GetDetectionImageFileName(imageId):
        return "{0}.detection".format(GetCurrentImageFileName(imageId))
    
    def SendCurrentImageBlock(self, imageId, size, num):
        imageFile = file(GetCurrentImageFileName(imageId),"rb")
        endByte = 60*80*2
        imageFile.seek(num * size)
        if (num * size + size > endByte):
            size = endByte - num * size
        data = imageFile.read(size)
        sendData = bytearray(chr(num))
        sendData.append(data)
        self.sendToSerialPort(StreamID.RETURN_IMAGE,sendData)

    def SendBaselineImageBlock(self, imageId, size, num):
        imageFile = file(GetBaselineImageFileName(imageId),"rb")
        endByte = 60*80*2
        imageFile.seek(num * size)
        if (num * size + size > endByte):
            size = endByte - num * size
        data = imageFile.read(size)
        sendData = bytearray(chr(num))
        sendData.append(data)
        self.sendToSerialPort(StreamID.RETURN_IMAGE,sendData)

    def SendDetectionImageBlock(self, imageId, size, num):
        imageFile = file(GetBaselineImageFileName(imageId),"rb")
        endByte = 60*80/8
        imageFile.seek(num * size)
        if (num * size + size > endByte):
            size = endByte - num * size
        data = imageFile.read(size)
        sendData = bytearray(chr(num))
        sendData.append(data)
        self.sendToSerialPort(StreamID.RETURN_IMAGE,sendData)

    def doMessageByStream(self, streamid, data):
        if (streamid == StreamID.REQUEST_WIFI):
            self.doWifiOnOff(data)
        if (streamid == StreamID.REQUEST_IMAGE):
            self.doSendImage(data)
        if (streamid == StreamID.REQUEST_DETECTION_ARRAY):
            self.sendSafeDetectArrayToSerialPort()
        if (streamid == StreamID.REQUEST_PING):
            self.doPing(data)

    def readLineFromSerialPort(self):
        msg = ""
        i = 0
        while not msg:
            print "polling serial {}".format(i)
            i = i + 1
            msg = self.ser.readline()
        return msg.rstrip()

    def readDataFromSerialPort(self):
        size = self.ser.read()
        print "size from serial data = {}".format(size)
        msg = bytearray(size)
        
        for i in range(0, size):
            msg[i] = self.ser.read();
        return msg

    def readMessageFromSerialPort(self):
        msg = self.readLineFromSerialPort()
        print "message='{}'".format(msg)

        
        #convert hex to data
        data = bytearray.fromhex(msg)
        
#        # Parse node from message
#        matchObj = re.match(r'\s*\[(\d+)\](.*)', msg)
#        node = 0
#        trailer = ''
#        if matchObj:
#            node = int(matchObj.group(1))
#            trailer = matchObj.group(2).strip()
            
#        print "node={} trailer='{}'".format(node,trailer)
#        data = self.readDataFromSerialPort();
        #        print "data='{}' len={}".format(data,len(data))

        return data

    def sendDataToSerialPort(self,streamId,data):
        self.serialSendLock.acquire(True)
        msg = "{:02x}".format(streamId.value)
        for b in data:
            msg += "{:02x}".format(b)
        self.ser.write(msg)        
        self.ser.write('\n')        
        self.serialSendLock.release()

    def sendDetectionMessageToSerialPort(self, msg):
        data = bytearray(chr(0)*50)
        struct.pack_into('>IIBB', data, 0, msg.imageId, msg.signalStrength, msg.detection, msg.safe)
        i = 0
        for h in msg.signalHistogram:
            struct.pack_into('>I', data, 10 + 4 * i, h)
            i += 1
        self.sendDataToSerialPort(StreamID.RETURN_DETECTION, data)

    def sendSafeDetectArrayToSerialPort(self):
        detectBytes = BoolListToByteList(self.detectionBits)
        data = bytearray(chr(0)*18)
        i = 0
        for b in detectBytes:
            data[i] = chr(b)
            i += 1

        safeBytes = BoolListToByteList(self.safeBits)
        for b in safeBytes:
            data[i] = chr(b)
            i += 1
        self.sendDataToSerialPort(StreamID.RETURN_DETECTION_ARRAY, data)
                           
if __name__ == '__main__':
    telemetry = TelemetryNode()
    telemetry.run()
#    while True:
#       raw_input("Press enter")
#       telemetry.sendToSerialPort(data)
