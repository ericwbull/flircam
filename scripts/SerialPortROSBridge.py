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

def DownlinkDataReceived(downlink, bridgeNode):
    # Only the ROS thread calls this
    # Send downlink data to serial port
    # Todo: implement verifyReceipt
    streamId = StreamID(downlink.streamId)
    bridge.BlockingSendDataToSerialPort(streamId, map(ord, downlink.data))

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
    RETURN_SERIAL_SYNC = 12
    REQUEST_SERIAL_SYNC = 13
    
class SerialPortROSBridge:
    def __init__(self):
        serialIsOpen = False
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
        self.okToSendSerial = False
        self.lastEgressSerialNumber = 0
        
    def run(self):
        # Main thread.
        rate = rospy.Rate(20)
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

# Potential deadlock if the main thread is ever blocked waiting for the send lock.
#    def doPing(self,data):
#        # Only the main thread calls this.
#        print "Ping={}".format(data)
#        self.LockedSendDataToSerialPort(StreamID.RETURN_PING, data)
        
    def doWifiOnOff(self,data):
        # Only the main thread calls this.
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
        # Only the main thread calls this.
        print "Shutdown"
        subprocess.call(["/sbin/shutdown","-h","now"])
        
    def doSendImage(self,data):
        # Only the main thread calls this.
        # publises ROS message.  Does not send serial.
        imageRequest = ImageRequest()
        imageRequest.id = data[0]
        imageRequest.type = data[1]
        imageRequest.blockSize = data[2]
        imageRequest.blockList = []
        rangeListSize = data[3]
        rangeData = data[4:]
        for i in range(0, rangeListSize):
            block = Block()
            block.start = rangeData[i*2]
            block.count = rangeData[i*2+1]
            imageRequest.blockList.append(block)

        print "SendImageRequest: id={} type={} blockSize={} rangeCount={}"\
            .format(imageRequest.id, imageRequest.type, imageRequest.blockSize, len(imageRequest.blockList))
        self.pubImageRequest.publish(imageRequest)

    def ReceiveSerialSync(self, data):
        # This is called on the main thread only.
        # Moteino is acknowledging the serial number of the last message that it received from the serial port.
        # This happens after every transmission to Moteino.
        # It must match the last serial number sent.
        # Other than sync requests no messages are sent to Moteino until okToSendSerial is set True
        # Another thread may be blocked waiting for okToSendSerial
        # The design prevents transmission of messages until the previous message or a special sync request is properly acknowledged.
        # We should not normally need to send special sync request messages because Moteino is required to ack every message.

        # if SendSerialSync jumped in and changed the lastEgressSerialNumber after we received this message,
        # then this will not set okToSendSerial = True.
        
        self.serialSendLock.acquire(True)
        if (self.lastEgressSerialNumber == data[0]):
            self.okToSendSerial = True
        self.serialSendLock.release()
        
    def doMessageByStream(self, streamid, data):
        # Only the main thread calls this.
        if (streamid == StreamID.REQUEST_WIFI):
            self.doWifiOnOff(data)
        elif (streamid == StreamID.REQUEST_SHUTDOWN):
            # If the message has nonzero data length then it is bad input - don't shutdown.
            if (len(data) == 0):
                self.doShutdown()
        elif (streamid == StreamID.REQUEST_IMAGE):
            # Forwarded ROS
            self.doSendImage(data)
        elif (streamid == StreamID.RETURN_SERIAL_SYNC):
            self.ReceiveSerialSync(data)

# Forward ping ROS
#        elif (streamid == StreamID.REQUEST_PING):
#            self.doPing(data)

    def readLineFromSerialPort(self):
        # Only the main thread calls this.
        msg = ""
        i = 0
        while not msg:
            print "polling serial {}".format(i)
            i = i + 1
            msg = self.ser.readline()
        return msg.rstrip()

    def readMessageFromSerialPort(self):
        # This is the ingress data stream.
        # Only the main thread calls this.
        # No serial number tracking here.
        messageIsGood = False
        while not messageIsGood:
            msg = self.readLineFromSerialPort()
            print "message='{}'".format(msg)
            if len(msg) == 0:
                # empty message.  Ignore.
                continue
            if (len(msg) % 2) == 1:
                # odd number of chars is bad
                self.badIngressMessageCount += 1
                continue
            
            data = bytearray.fromhex(msg)
            messageIsGood = True

        return data

    def SendSerialSync(self):
        self.SendDataToSerialPort(StreamID.REQUEST_SERIAL_SYNC,[])

    def WaitForOkToSend(self):
        # Spin until OkToSend or timeout
        # Only the ROS thread calls this
        # Depends upon main thread to asynchronously set okToSend.
        timeout = time.time() + 0.05
        while not self.okToSendSerial and (time.time() < timeout):
            timenow = time

        if (self.okToSendSerial):
            return
        
        # The following will keep sending sync until we ok to send means we don't need to send it
        # Moteino just needs to receive the message and respond in less than the sleep time
        while (True == self.LockingTrySendSerialSync()):
            # During this time the main thread will set okToSendSerial=true
            time.sleep(0.2)

    def LockingTrySendSerialSync(self):
        serialSyncSent = False
        # Sends serial sync only if okToSend is found to be false.
        # If serial sync is found to be true, then no need to send sync, and the caller which was
        # waiting for ok to send is now free to proceed.
        self.serialSendLock.acquire(True)
        if (self.okToSendSerial == False):
            self.SendSerialSync()
        else:
            # This is good we don't need to send another sync message
            serialSyncSent = True
        self.serialSendLock.release()
        return serialSyncSent

    def LockingTrySendDataToSerialPort(self, streamId, data):
        dataSent = False
        self.serialSendLock.acquire(True)
        if (self.okToSendSerial == True):
            self.SendDataToSerialPort(streamId, data)
            dataSent = true
        self.serialSendLock.release()
        return dataSent

    def BlockingSendDataToSerialPort(self, streamId, data):
        # ROS thread calls this
        # Try sending once, and then invoke the waiting code if the send was blocked
        # This should not execute the loop more than once.
        while False == LockingTrySendDataToSerialPort(self, streamId, data):
            self.WaitForOKToSend()

    def SendDataToSerialPort(self, streamId, data):
        # Only ROS calls this.  Main thread never transmits on serial egress directly -- it goes through ROS thread to transmit
        # This sends the message unconditionally and then releases the lock
        # Stream of incrementing serial numbers on egress is guaranteed because serial number increment and send are atomic
        # Implies that a stream of identically incrementing acknowledgements on ingress is expected.
        # OkToSend means ok to send *one* message only.
        msg = "{:02x}".format(streamId.value)
        for b in data:
            msg += "{:02x}".format(b)

        # increment before putting into the message
        self.lastEgressSerialNumber += 1
        self.okToSendSerial = False
        # Moteino must return ack for this serial number before another message can be sent from ROS thread
        msg += "{:02x}".format(self.lastEgressSerialNumber)
        self.ser.write(msg)        
        self.ser.write('\n')
        # Any time after this point, it is possible for Moteino to have returned the serial number ack.
        print "sendDataToSerialPort: {}".format(msg)

if __name__ == '__main__':
    bridge = SerialPortROSBridge()
    bridge.run()
#    while True:
#       raw_input("Press enter")
#       bridge.sendToSerialPort(data)
