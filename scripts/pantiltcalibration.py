#!/usr/bin/env python

import rospy
import time
import os
import sys

from flircam.msg import PanTiltPos

def wait_key():
    ''' Wait for a key press on the console and return it. '''
    result = None
    if os.name == 'nt':
        import msvcrt
        result = msvcrt.getch()
    else:
        import termios
        fd = sys.stdin.fileno()

        oldterm = termios.tcgetattr(fd)
        newattr = termios.tcgetattr(fd)
        newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, newattr)

        try:
            result = sys.stdin.read(1)
        except IOError:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)

    return result


class PanTiltCalibration:
    def __init__(self):
        self.pubPanTilt = rospy.Publisher('pantilt_position_command', PanTiltPos, queue_size=10)
        rospy.init_node('PanTiltCalibration', anonymous=True)
        self.h = 0
        self.v = 0

    def point(self, h, v):
        pantilt=PanTiltPos()
        pantilt.horizontalAngle=h
        pantilt.verticalAngle=v
        self.pubPanTilt.publish(pantilt)
        print "h={} v={}".format(h, v)
        
                               
    def run(self):
#        time.sleep(60)
        self.h = 0
        self.v = 0
        while not rospy.is_shutdown():
            self.point(self.h, self.v)
            x = wait_key()
            if x == 'q':
                break

            if (x == 'r'):
                self.h -= 5
            if (x == 'y'):
                self.h += 5
                
            if (x == 'f'):
                self.h -= 1
            if (x == 'h'):
                self.h += 1
            

            if (x == 'x'):
                self.v -= 5
            if (x == 'w'):
                self.v += 5
                
            if (x == 'c'):
                self.v -= 1
            if (x == 'e'):
                self.v += 1
            


if __name__ == '__main__':
    try:
        me = PanTiltCalibration()
        me.run()
    except rospy.ROSInterruptException:
        pass

