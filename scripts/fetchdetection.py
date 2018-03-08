#!/usr/bin/env python

import time
import rospy
import binascii
import array
from beginner_tutorials.msg import NodeBytes

import sys

def NodeBytesReceived(data,fetch):
    dataAsBytes=bytearray(data.data)
    msg = "Received {}:{}".format(data.node, binascii.hexlify(dataAsBytes))
    blockNum = dataAsBytes[0]
    fetch.addBlock(blockNum, dataAsBytes[1:])
    fetch.blockReceived[blockNum] = True
    print msg
    # Write data to the file or save in memory until all the parts arrive
    
class ImageFetcher:
    def __init__(self, imageId, folderNum, serialNum):
        self.frameNum = imageId
        self.folderNum = folderNum
        self.serialNum = serialNum
        self.fileData = bytearray([0]*600)
        self.imageData = array.array('H')
        self.imageData.extend([0]*4800)
        self.pub = rospy.Publisher('byte_commands_to_nodes', NodeBytes, queue_size=10)
        rospy.Subscriber('byte_return_from_nodes', NodeBytes, NodeBytesReceived, self)
        self.blockReceived = []
        self.blockCountPending = 0
        rospy.init_node('FetchImage', anonymous=True)

    def missingBlocks(self):
        blockCount = 0
        missing = list()
        
        for i in range(len(self.blockReceived)):
            if not self.blockReceived[i]:
                blockCount += 1
                missing.append(i)
        return missing
        
    def addBlock(self,blockNum, data):
        print "len={}".format(len(data))
        print "blocksize={}".format(self.blockSize)
        print "blockNum={}".format(blockNum)
        for i in range(len(data)):
            byteNum = blockNum * self.blockSize + i
#            print "fileData[{}]={}".format(byteNum,data[i])
            print "data[{}]={}".format(byteNum,data[i])
            self.fileData[byteNum] = data[i]

    def minMaxPixel(self):
        minVal = 65536
        maxVal = 0
        for p in self.imageData:
            if (p > 0 and p < minVal):
                minVal = p
            if (p > maxVal):
                maxVal = p

        return (minVal,maxVal)

    def writeImageDataToPBM(self):
        (minVal, maxVal) = self.minMaxPixel()
        print "minVal={} maxVal={}".format(minVal,maxVal)
        # write as pgm
        nMaxVal = maxVal - minVal
        filename='/tmp/image{}.pbm'.format(self.frameNum)
        f = file(filename, 'w')
        f.write('P1\n')
        f.write('# image.pbm\n')
        f.write('80 60\n')
        
        for p in self.imageData:
            f.write(str(p))
            f.write('\n')
        f.close()
        print "Wrote {}".format(filename)
            
    def copyFileDataToImageData(self):
        for i in range(len(self.imageData)):
            bitNum = i % 8
            byteNum = i / 8
            if (self.fileData[byteNum] & (1 << bitNum)):
                self.imageData[i] = 1
            
    def saveDetectionToPBM(self):
        self.blockSize = 58
        self.blockCountPending = 11
        self.blockReceived = [False for x in range(self.blockCountPending)]

        rate = rospy.Rate(10) # 10hz
#        while self.blockCountPending:
            # Keep requesting the missing parts until they all arrive
        pubMsg = NodeBytes()
        pubMsg.node = 5
        # streamid 8, imageid, imagetype (1=current), blocksize 59, range count, range list (start, count), ...
        #                 collection, frame,           serial  type
        pubMsg.data = [8, self.folderNum,0,        self.frameNum,0,  self.serialNum,0,    3,    self.blockSize, 1, 0, self.blockCountPending]
        print "publish"
        self.pub.publish(pubMsg)
        missing = self.missingBlocks()

        prevMissingCount = len(missing)

        watchdogStart = time.time()
        timeout = False
        while not rospy.is_shutdown() and len(missing)>0 and not timeout:
            rate.sleep()
            missing = self.missingBlocks()
            print "missing blocks ({})={}".format(len(missing),",".join(map(str,missing)))
            if (len(missing) != prevMissingCount):
                prevMissingCount = len(missing)
                watchdogStart = time.time()
            watchdogEnd = time.time()

            if (watchdogEnd - watchdogStart > 40):
                timeout = True
            
#        if (len(missing) == 0):

        # transfer the filedata to the image data
        self.copyFileDataToImageData()
        self.writeImageDataToPBM()    
            
if __name__ == '__main__':
    try:
        time.sleep(1)
        fetch = ImageFetcher(int(sys.argv[1]),int(sys.argv[2]),int(sys.argv[3]))
        fetch.saveDetectionToPBM()
        
    except rospy.ROSInterruptException:
        pass

