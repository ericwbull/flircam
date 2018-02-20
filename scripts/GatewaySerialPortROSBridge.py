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
from beginner_tutorials.msg import Grideye
from beginner_tutorials.msg import GrideyeRaw
from beginner_tutorials.msg import NodeBytes

def ImageIdToString(imageId):
    return "{}/{}.{}".format(imageId.collectionNumber, imageId.frameNumber, imageId.serialNumber)

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

class StreamID(Enum):
    RETURN_INFO = 1
    RETURN_GRIDEYE = 2
    RETURN_GRIDEYE_RAW = 3
    RETURN_DETECTION = 4
    RETURN_DETECTION_ARRAY = 5
    REQUEST_WIFI = 7
    REQUEST_IMAGE = 8
    REQUEST_DETECTION_ARRAY = 10
    RETURN_IMAGE = 9
    RETURN_PING = 66
    REQUEST_PING = 65
    UNKNOWN = 0

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
        self.pubgrideye = rospy.Publisher('grideye_pixels', Grideye, queue_size=10)
        self.pubgrideyeraw = rospy.Publisher('grideye_pixels_raw', GrideyeRaw, queue_size=10)
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

    def doGridEyeReturn(self,data):
        datalist = list(data)
        databytecount = len(datalist)
        pixelCount = (databytecount - 1) / 4
        start = struct.unpack_from('B', data)[0]
        
        print "start={} databytecount={} pixelCount={} data={} datalen={}".format(start,databytecount,pixelCount,datalist,len(data)) 
        unpackstr="{}f".format(pixelCount)
        pixels = struct.unpack(unpackstr, data[1:])
        grideyeMsg = Grideye()
        grideyeMsg.start = start
        grideyeMsg.pixels = pixels
        
        print "grideye msg={}".format(grideyeMsg) 
        self.pubgrideye.publish(grideyeMsg)

    def doGrideyeRawReturn(self,data):
        datalist = list(data)
        databytecount = len(datalist)
        if (databytecount % 2 != 0):
            print "grideyeraw bad message"
            return
        if (databytecount < 1):
            print "grideyeraw bad message"
            return
        pixelCount = (databytecount - 1) / 2
        start = struct.unpack_from('B', data)[0]
        
        print "start={} databytecount={} pixelCount={} data={} datalen={}".format(start,databytecount,pixelCount,datalist,len(data)) 
        unpackstr="{}h".format(pixelCount)
        pixels = struct.unpack(unpackstr, data[1:])
        grideyeMsg = GrideyeRaw()
        grideyeMsg.start = start
        grideyeMsg.pixels = pixels
        
        print "grideyeraw msg={}".format(grideyeMsg) 
        self.pubgrideyeraw.publish(grideyeMsg)

#    def doDetectionReturn(self,data,node,other):
#        imageId,signalStrength,detection,safe = struct.unpack_from('>IIBB',data, 0)
#        signalHistogram = struct.unpack_from('>10I', data, 10);
#            
#        receivedMsg = "[{0}] imageId={1} signalStrength=0x{2:04x} ({2}) detection={3} safe={4} signalHistogram={5} {6}".format(node,imageId,signalStrength,detection,safe,','.join(str(x) for x in signalHistogram), other)
#        print "{}".format(receivedMsg)
#            
#        if detection:
#            pubMsg = "[{}] imageId:{} threat:{} threatSignal:{}:{} {}\n".format(node,imageId,imageId,signalStrength,','.join(str(x) for x in signalHistogram),other)
#            self.pub.publish(pubMsg)
#        if safe:
#            pubMsg = "[{}] imageId:{} safe:{} {}\n".format(node,imageId,imageId,other)
#            self.pub.publish(pubMsg)    

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
        streamId = StreamID(downlink.streamId)
        self.processMessageByStream(5, streamId, downlink.data, "")
        
    def processMessageByStream(self,node,streamid,data,other):
        if (streamid==StreamID.RETURN_INFO):
            self.doInfoReturn(node,data,other)
        elif (streamid==StreamID.RETURN_GRIDEYE):
            self.doGridEyeReturn(data)
        elif (streamid==StreamID.RETURN_GRIDEYE_RAW):
            self.doGridEyeRawReturn(data)
        elif (streamid==StreamID.RETURN_DETECTION):  # flircam detection
            self.doDetectionReturn(data,node,other)
        elif (streamid==StreamID.RETURN_DETECTION_ARRAY):  # flircam detection&safebits
            self.doDetectionArrayReturn(data,node,other)
        elif (streamid==StreamID.RETURN_IMAGE):
            self.doImageReturn(node,data)
        else:
            print "unhandled streamid '{}'".format(streamid)
        
    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            try: 
                (node, data, other) = self.readMessageFromSerialPort()

                if (len(data)>0):
                
                    streamid=StreamID.UNKNOWN
                    try:
                        streamid=StreamID(data[0])
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
