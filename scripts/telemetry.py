#!/usr/bin/env python

import time
import serial
import rospy

from flircam.msg import Detection

def DetectionReceived(data, telemetryNode):
    print "imageId={} signalStength={} detection={} safe={} signalHistogram={}".format(data.imageId, data.signalStrength, data.detection, data.safe, ','.join(str(x) for x in data.signalHistogram))
    telemetryNode.sendToSerialPort(data)

SERIAL_PORT='/dev/ttyS0'
BAUD_RATE=115200

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
                                                                        
        rospy.Subscriber('detection', Detection, DetectionReceived, self)
        rospy.init_node('TelemetryNode', anonymous=True)
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def sendToSerialPort(self,data):
        msg = "04{:08x}{:08x}{:02x}{:02x}".format(data.imageId,data.signalStrength,data.detection,data.safe)
        for h in data.signalHistogram:
            msg += "{:08x}".format(h)

#        msg = "0141424344454647\r"
#        print "msg={}".format(msg)
#        for c in msg:
#            self.ser.write(c)
#        self.ser.flush()
#        self.ser.write('\r')        
#        self.ser.write('\n')        
#        self.ser.write('01')
#        for i in range(0,self.count):
#            print "{}:{}".format(self.count,chr(85+i))
#            self.ser.write("{:02x}".format(85+i))
#        self.ser.write('\r')        
        self.ser.write(msg)        
        self.ser.write('\n')        
#        self.ser.flush()
        self.count += 1
                           
if __name__ == '__main__':
    telemetry = TelemetryNode()
    telemetry.run()
#    while True:
#       raw_input("Press enter")
#       telemetry.sendToSerialPort(data)
