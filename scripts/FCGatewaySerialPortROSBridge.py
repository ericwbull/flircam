#!/usr/bin/env python

import sys
import rospy
import serial
import struct
import re
import binascii
from enum import Enum

from std_msgs.msg import String
from flircam.msg import DownlinkData
from flircam.msg import NodeBytes
import flircam_util as fcutil


def NodeBytesReceivedFromROSFabric(data, bridge):
    dataAsBytes=bytearray(data.data)
    msg = "{}:{}".format(data.node, binascii.hexlify(dataAsBytes))
    print msg
    bridge.sendToSerialPort(msg)
    
def DataReceivedFromROSFabric(data, bridge):
   # Data received from the gateway application only
   # Message is formated as nodeid:data
   # We need to format the data as hex before sending to serial port
   # data is a bytearray
   # translate data to hex
    rospy.loginfo('{} command={}'.format(rospy.get_caller_id(),data.data))

    # separate the node number from the data
        # Parse node from message
    matchObj = re.match(r'(.*):(.*)', data)
    node = 0
    data = ""
    if matchObj:
        node = int(matchObj.group(1))
        data = bytearray(matchObj.group(2))

    msg = "{}:{}".format(node,binascii.hexlify(data))
    bridge.sendToSerialPort(msg)

SERIAL_PORT='/dev/ttyS0'
BAUD_RATE=115200

def DownlinkDataReceived(downlink, bridgeNode):
    # Only the ROS thread calls this
    # Send downlink data to serial port
    # Todo: implement verifyReceipt
    bridge.DownlinkDataReceived(downlink)

class GatewaySerialPortROSBridge:
    def __init__(self):
        serialIsOpen = False
        while not serialIsOpen:
            try:
                self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
                serialIsOpen = True
            except serial.SerialException:
                print "Serial port did not open"
                pass
        
        
        self.pub = rospy.Publisher('telemetry_from_nodes', String, queue_size=10)
        self.pubNodeBytes = rospy.Publisher('byte_return_from_nodes', NodeBytes, queue_size=10)
        rospy.Subscriber('commands_to_nodes', String, DataReceivedFromROSFabric, self)
        rospy.Subscriber('byte_commands_to_nodes', NodeBytes, NodeBytesReceivedFromROSFabric, self)
        rospy.Subscriber('downlink', DownlinkData, DownlinkDataReceived, self)
        rospy.init_node('GatewaySerialPortROSBridge', anonymous=True)
        self.threatCount = 0

    def readLineFromSerialPort(self):
        msg = ""
        i = 0
        while not msg:
            print "polling serial {}".format(i)
            i = i + 1
            msg = self.ser.readline()
        return msg

    def readDataFromSerialPort(self):
        size = struct.unpack('B', self.ser.read())[0];
        print "size from serial data = {}".format(size)
        msg = bytearray(size)

        for i in range(0, size):
            msg[i] = self.ser.read();
        return msg

    def readMessageFromSerialPort(self):
        msg = self.readLineFromSerialPort();
        print "message={}".format(msg)

        # Parse node from message
        matchObj = re.match(r'\s*\[(\d+)\](.*)', msg)
        node = 0
        data = bytearray()
        other = ''
        if matchObj:
            node = int(matchObj.group(1))
            other = matchObj.group(2).strip()
        else:
            return node, data, other

        # Parse data from other
        matchObj = re.match(r'(.*)\[data\((\d+)\):([0-9a-f]*)\](.*)', other)
        
        if matchObj:
            other = matchObj.group(1) + matchObj.group(4)
            datalen = matchObj.group(2)
            data = matchObj.group(3)
        else:
            return node, data, other

        other = other.rstrip()
        print "node={} other={} data={}".format(node,other,data)

        # translate the data to a bytearray
        data = bytearray.fromhex(data)
        
        return node, data, other
        
    def sendToSerialPort(self,msg):
        self.ser.write(msg)
        self.ser.write('\n')

    def doInfoReturn(self,node,data,other):
        pubMsg = "[{}] {} {}\n".format(node,data,other)
        print "publish '{}'".format(pubMsg)
        self.pub.publish(pubMsg)

    def doAlertReturn(self,data,node,other):
        (collectionNum, frameNum, serialNum, minPixel, maxPixel, avgPixel, pixelCount, thisFrameCount, totalFrameCount) = struct.unpack_from('>HHHHHHLHH',data, 0)

        msg = "[{}] threat:1 imageId:{} collectionNum:{} serialNum:{} minPixel:{} maxPixel:{} avgPixel:{} pixelCount:{} thisFrameCount:{} totalFrameCount:{}\n"\
              .format(node, frameNum, collectionNum, serialNum, minPixel, maxPixel, avgPixel, pixelCount, thisFrameCount, totalFrameCount)
        print msg
        pubMsg=String(msg)
        self.pub.publish(pubMsg)

    def doStatusReturn(self,data,node,other):
        (collectionNum, frameNum, serialNum, minPixel, maxPixel, avgPixel, statusCount, alertCount, totalImageCount) = struct.unpack_from('>HHHHHHHHH',data, 0)

        msg = "[{}] imageId:{} collectionNum:{} serialNum:{} minPixel:{} maxPixel:{} avgPixel:{} statusCount:{} alertCount:{} imageCount:{}\n"\
              .format(node, frameNum, collectionNum, serialNum, minPixel, maxPixel, avgPixel, statusCount, alertCount, totalImageCount)
        print msg
        pubMsg=String(msg)
        self.pub.publish(pubMsg)

    def doDetectionArrayReturn(self,data,node,other):
#        print "data=".format(data)
        (collectionNum, frameNum, serialNum) = struct.unpack_from('>HHH',data, 0)
#        print "imageId={}".format(imageId)
        detection = struct.unpack_from('>9B',data, 6)
        safe = struct.unpack_from('>9B',data, 15)
            
        bitMask = 1
        bitNum = 0
        detectBitList = list()
        safeBitList = list()
        for i in range(70):
            byteNum = i/8
            bitNum = i%8
            if (detection[byteNum] & (1 << bitNum)):
                detectBitList.append(i+1)
            if (safe[byteNum] & (1 << bitNum)):
                safeBitList.append(i+1)
        threatCount = len(detectBitList)
        threatCountDelta = threatCount - self.threatCount
        if (len(detectBitList)==0):
            detectBitStr = "0"
        else:
            detectBitStr = ",".join(map(str,detectBitList))
        if (len(safeBitList)==0):
            safeBitStr = "0"
        else:
            safeBitStr = ",".join(map(str,safeBitList))
            
        msg = "[{}] imageId:{} collectionNum:{} serialNum:{} detectArray:{} safeArray:{} threatCount:{} threat:{}\n"\
              .format(node,frameNum,collectionNum,serialNum,detectBitStr,safeBitStr,threatCount,threatCountDelta)
        self.threatCount = threatCount
        print msg
        pubMsg=String(msg)
        self.pub.publish(pubMsg)
        
#        pubMsg="[{}] safeArray:".format(node)
#        for b in safe:
#            pubMsg += ":{:02x}".format(b)
#            pubMsg += " \n"
#        self.pub.publish(pubMsg)

    def doImageReturn(self,node,data):
        # Forward to ROS
        pubMsg = NodeBytes()
        pubMsg.node = node
        pubMsg.data = data
        self.pubNodeBytes.publish(pubMsg)

    def DownlinkDataReceived(self,downlink):
        streamId = fcutil.StreamID(downlink.streamId)
        self.processMessageByStream(5, streamId, downlink.data, "")
        
    def processMessageByStream(self,node,streamid,data,other):
        if (streamid==fcutil.StreamID.RETURN_INFO):
            self.doInfoReturn(node,data,other)
        elif (streamid==fcutil.StreamID.RETURN_ALERT):  # flircam detection
            self.doAlertReturn(data,node,other)
        elif (streamid==fcutil.StreamID.RETURN_STATUS):  # flircam status
            self.doStatusReturn(data,node,other)
        elif (streamid==fcutil.StreamID.RETURN_DETECTION_ARRAY):  # flircam detection&safebits
            self.doDetectionArrayReturn(data,node,other)
        elif (streamid==fcutil.StreamID.RETURN_IMAGE):
            self.doImageReturn(node,data)
        else:
            print "unhandled streamid '{}'".format(streamid)
        
    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            try: 
                (node, data, other) = self.readMessageFromSerialPort()

                if (len(data)>0):
                
                    streamid=fcutil.StreamID.UNKNOWN
                    try:
                        streamid=fcutil.StreamID(data[0])
                    except ValueError:
                        pass

                    print "streamid={}".format(streamid)
                    data = data[1:]

                    self.processMessageByStream(node, streamid, data, other)
                else:
                     print "no data"   
            except struct.error:
                print "struct error"
                pass
            rate.sleep()
            sys.stdout.flush()

if __name__ == '__main__':
    try:
        bridge = GatewaySerialPortROSBridge()
        bridge.run()
        
    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        pass
