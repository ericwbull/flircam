#!/usr/bin/env python

import time
import serial
import rospy
import subprocess
import threading
import struct
from enum import Enum
from flircam.msg import DownlinkData
from flircam.msg import Block
from flircam.msg import ImageRequest
import sys

SERIAL_PORT='/dev/ttyS0'
# Radio rate is about 55000.
# It will be trouble if we send data to moteino faster than it can transmit it.
BAUD_RATE=38400

def DetectionReceived(downlink, bridgeNode):
    # Send downlink data to serial port
    # Todo: implement verifyReceipt
    self.sendDataToSerialPort(downlink.streamId,downlink.data)

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

class SerialPortROSBridge:
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
                                                                        
        rospy.init_node('SerialPortROSBridge', anonymous=True)
        rospy.Subscriber('downlink', DownlinkData, DownlinkDataReceived, self)
        self.pubImageRequest = rospy.Publisher('image_request', ImageRequest, queue_size=200)
        self.serialSendLock = threading.Lock()
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                data = self.readMessageFromSerialPort()
                if (len(data)>0):
                    streamid = StreamID(data[0])
                    self.doMessageByStream(streamid, data[1:])
            except struct.error:
                pass
            sys.stdout.flush()
            rate.sleep()

    def doPing(self,data):
        print "Ping={}".format(data)
        self.sendDataToSerialPort(StreamID.RETURN_PING, data)
        
    def doWifiOnOff(self,data):
        off = data[0]
        cmd1="up"
        cmd2="unblock"
        if off:
            cmd1 = "down"
            cmd2 = "block"

        print "Wifi cmd1={} cmd2={}".format(cmd1,cmd2)
        subprocess.call(["/sbin/ifconfig", "wlan0", cmd1])
        time.sleep(5)
        subprocess.call(["/usr/sbin/rfkill", cmd2, "0"])

    def doShutdown(self):
        print "Shutdown"
        subprocess.call(["/sbin/shutdown","-h","now"])
        
    def doSendImage(self,data):
        imageRequest = ImageRequest()
        imageRequest.id = data[0]
        imageRequest.type = data[1]
        imageRequest.blockSize = data[2]
        imageRequest.blockList = []
        rangeListSize = data[3]
        print "SendImageRequest: id={} type={} blockSize={} rangeCount={}".format(imageId, imageType, blockSize, rangeListSize)
        rangeData = data[4:]
        for i in range(0, rangeListSize):
            block = Block()
            block.start = rangeData[i*2]
            block.count = rangeData[i*2+1]
            pubImageRequest.publish(imageRequest)


    def doMessageByStream(self, streamid, data):
        if (streamid == StreamID.REQUEST_WIFI):
            self.doWifiOnOff(data)
        if (streamid == StreamID.REQUEST_SHUTDOWN):
            # If the message has nonzero data length then it is bad input - don't shutdown.
            if (len(data) == 0):
                self.doShutdown()
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

    def readMessageFromSerialPort(self):
        msg = self.readLineFromSerialPort()
        print "message='{}'".format(msg)
        data = bytearray.fromhex(msg)
        return data

    def sendDataToSerialPort(self,streamId,data):
        self.serialSendLock.acquire(True)
        msg = "{:02x}".format(streamId.value)
        for b in data:
            msg += "{:02x}".format(b)
        self.ser.write(msg)        
        self.ser.write('\n')        
        self.serialSendLock.release()
                           
if __name__ == '__main__':
    bridge = SerialPortROSBridge()
    bridge.run()
#    while True:
#       raw_input("Press enter")
#       bridge.sendToSerialPort(data)
