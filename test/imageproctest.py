#!/usr/bin/env python

import rospy
import time

import threading
from flircam.msg import ImageId
import sys
import os

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

class ImageProcTest:
    def __init__(self):
        self.pubImageDone = rospy.Publisher('image_done', ImageId, queue_size=10)
        self.collectionNum = 0
        rospy.init_node('ImageProcTest', anonymous=True)

    def run(self):
        time.sleep(2)

        for serialNum in range(0,40):
            self.publishImageDone(1,serialNum)
            if wait_key() == 'q':
                break
                
        # send endCollection
        self.publishEndCollection()
        
    def publishImageDone(self, frameNum, serialNum):
        imageId = ImageId()
        imageId.collectionNumber=self.collectionNum
        imageId.frameNumber=frameNum
        imageId.serialNumber=serialNum
        imageId.endCollection=False
        self.pubImageDone.publish(imageId)
        print "{}: frame={}".format(self.collectionNum, frameNum)

    def publishEndCollection(self):
        imageId = ImageId()
        imageId.collectionNumber=self.collectionNum
        imageId.frameNumber=0
        imageId.serialNumber=0
        imageId.endCollection=True
        self.pubImageDone.publish(imageId)
        print "{}: end collection".format(self.collectionNum)

        
if __name__ == '__main__':
    try:
        test = ImageProcTest()
        test.run()
    except rospy.ROSInterruptException:
        pass
