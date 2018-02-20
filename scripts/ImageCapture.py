import rospy
import time

import threading
from flircam.msg import PanTiltPos
from flircam.msg import ImageId

# Image capture publishes servo motion and save_image request commands.
# At init it is told the row and column positions of all frames.  It automatically associates a unique
# frame number with each of the row column positions.  Frame numbers are sequential starting from 1 (use a row first, column second, sorting to number the frames).
# The client specifies a row column position of the flatfield frame.  Frame number 0 is assigned to the flatfield frame.
# The flatfield frame is automatically processed at the beginning of the collection, and every so often.
#
# The interface provides methods for the client to request regions or individual cells of the grid to be imaged. Note flatfield is automatic after it is configured at init.
# These requests go into a collection, and control returns immediately to the caller.
# If a cell is requested and it's already in the collection to be processed, it is not added again.  Implement the collection as a bit flag for each cell.  The flag
# is raised when a request comes in and it is lowered when the request is processed.
# 
# An asynchronous thread
# processes the requests in the collection.  There's no requirement to process first-in-first-out.
# The algorithm should try to minimize servo movement.  Try processing the nearest image next.
# At init the client also specifies a time duration that ends collection.  If there's nothing to process
# for the specified duration, then an endCollection message is published to the save_image stream.
#
# The client monitors ImageCapture status to know when it has ended the collection.  The client can set a
# new collection number which is latched when the next image is processed, otherwise the collection number
# is automatically incremented.  ImageCapture sets the next pending collection number at the same time
# that it latches the current collection number.  
class Frame:
    def __init__(self, indexPos, anglePos):
        (row,col) = indexPos
        (v,h) = anglePos
        self.number = -1
        self.row = row
        self.col = col
        self.request = False
        self.servoH = h
        self.servoV = v
        self.serialNumber = 0
        
    def __lt__(self, other):
        if (self.row == other.row):
            return self.col < other.col
        return self.row < other.row

    # this should not be needed
#    def __eq__(self, other):
#        return not (self < other || other < self)

class CollectionThread(threading.Thread):
    def __init__(self, imageCapture):
        threading.Thread.__init__(self)
        self.imageCapture = imageCapture
        
    # This is the asynchronous thread that keeps processing requests until endCollection, and then it exits.
    def run(self):
        self.imageCapture.run()

        

class ImageCapture:
    def __init__(self, flatfieldAnglePos, imagePos, servoRowAngles, servoColAngles, servoDelay, cameraDelay):
        self.initImageFrames(imagePos, servoRowAngles, servoColAngles)
        
        self.printFrameMap()        
        self.flatfield = Frame((-1,-1), flatfieldAnglePos)
        self.flatfield.number = 0
        self.pubPanTilt = rospy.Publisher('pantilt_position_command', PanTiltPos, queue_size=10)
        self.pubSaveImage = rospy.Publisher('save_image', ImageId, queue_size=10)
        self.collectionNum = 0
        self.lastCapturedFrame = False
        self.delayBetweenFramesDefault = 0.1
        self.delayBetweenFrames = self.delayBetweenFramesDefault
        self.cameraDelay = cameraDelay
        self.servoDelay = servoDelay
        # lock is for synchronizing the main thread with the ImageCapture thread
        self.lock = threading.Lock()

    def resetFrameDelay(self):
        self.delayBetweenFrames = self.delayBetweenFramesDefault

    def setNextFrame(self,frameNum):
        self.lastCapturedFrame = self.frames[frameNum-1]
        
    def printFrameMap(self):
        for f in self.frames:
            print "num:{} row:{} col:{} h:{} v:{}".format(f.number,f.row,f.col,f.servoH,f.servoV)

        for r in range(self.maxRow+1):
            for c in range(self.maxCol+1):
                if not self.frameGrid[r][c]:
                    num='-'
                else:
                    num = self.frameGrid[r][c].number
                print("{:>3}".format(num)),
            print("")
    def doCollection(self, num):
        # Create the collectoin thread and run it
        # No point in this being a thread if we are joining before returning
        self.collectionNum = num
        self.run()
        #        collectionThread = CollectionThread(self)
        #        collectionThread.start()
        #        collectionThread.join()

    def frameIfRequest(self, row, col):
        frm = self.frameGrid[row][col]
        if frm:
            if frm.request:
                return frm
        return False

    def frameAbove(self,f):
        # define above as row number + 1
        if (f.row < self.maxRow):
            return self.frameGrid[f.row+1][f.col]
        return False
    
    def frameBelow(self,f):
        # define below as row number - 1
        if (f.row > 0):
            return self.frameGrid[f.row-1][f.col]
        return False

    def frameRight(self,f):
        # define right as col number + 1
        if (f.col < self.maxCol):
            return self.frameGrid[f.row][f.col+1]
        return False

    def frameLeft(self,f):
        # define left as col number - 1
        if (f.col > 0):
            return self.frameGrid[f.row][f.col-1]
        return False

    def frameNeighbors(self,f,span=1):
        ns = set()
        ns.update([f])
        while(span > 0):
            newItems = set()
            for i in ns:
                n = self.frameRight(i)
                if n:
                    newItems.add(n)
                n = self.frameLeft(i)
                if n:
                    newItems.add(n)
                n = self.frameAbove(i)
                if n:
                    newItems.add(n)
                n = self.frameBelow(i)
                if n:
                    newItems.add(n)
            ns.update(newItems)
            span -= 1
        ns.difference_update([f])
        return ns
    
    def findFrameWithRequest(self, frameSet):
        for f in frameSet:
            if f.request:
                return f
        return False

    def seekNeighbors(self, frameSet, discardSet):
        # add the neighbors right left above below and discard the previous items
        discardSet.update(frameSet)
        newItems = set()
        for f in frameSet:
            newItems.update(self.frameNeighbors(f))
        frameSet.update(newItems)
        frameSet.difference_update(discardSet)
        
    def nextFrame(self):
        if not self.lastCapturedFrame:
            # return the first frame with a request, or False if none
            return self.findFrameWithRequest(self.frames)
        else:
            # See if the last captured frame has a request
            f = self.findFrameWithRequest([self.lastCapturedFrame])
            if f:
                return f

            # Look for request in neighbors
            ns = set()
            ns.update(self.frameNeighbors(self.lastCapturedFrame))

            # this set tracks the items already checked
            # seekNeighbors updates discard set
            alreadyChecked = set()
            while(len(ns)):
                print("lastCaptured:{} ns:{}".format(self.lastCapturedFrame.number, ','.join(str(s.number) for s in ns)))
                nlsorted = list(ns)
                nlsorted.sort()
                f = self.findFrameWithRequest(nlsorted)
                if f:
                    return f
                else:
                    self.seekNeighbors(ns,alreadyChecked)
            # No frames have a request
            return False
            
    # This is the asynchronous thread that keeps processing requests until endCollection, and then it exits.
    def run(self):
        self.resetSerialNumbers()
        self.delayBetweenFrames = self.delayBetweenFramesDefault
        # request flatfield
        self.acquireFlatfield()
        
        # Walk through the frame list looking for requests
        timeout = False
        while not rospy.is_shutdown() and not timeout:
            self.lock.acquire()
            f = self.nextFrame()
            delay = self.delayBetweenFrames
            if f:
                self.pointAndCapture(f)
                self.lastCapturedFrame = f
                f.request = False
                watchDog = time.time()
            self.lock.release()

            if time.time() - watchDog > 10:
                timeout = True
                print "Done waiting for frame requests."
            else:
                if f:
                    time.sleep(delay)
                else:
                    print "No more frames to capture."
                    time.sleep(1)
                
        
        # send endCollection
        self.publishEndCollection()
        
    def initImageFrames(self, imagePos, servoRowAngles, servoColAngles):
        # Get max row and col number
        self.maxRow = 0
        self.maxCol = 0
        for p in imagePos:
            (row,col) = p
            if row > self.maxRow:
                self.maxRow = row
            if col > self.maxCol:
                self.maxCol = col

        # Create a [row][col] grid
        self.frameGrid = []
        for r in range(0,self.maxRow+1):
            carr = []
            for c in range(0,self.maxCol+1):
                # This initializes the grid with False
                carr.append(False)
            self.frameGrid.append(carr)

        # Sort the frames and number them
        self.frames = list()
        for p in imagePos:
            (row,col) = p
            anglePos = tuple((servoRowAngles[row],servoColAngles[col]))
            frame = Frame(p,anglePos)
            self.frames.append(frame)
            self.frameGrid[row][col] = frame
        self.frames.sort()

        frameNum = 1
        for f in self.frames:
            f.number = frameNum
            frameNum += 1

    def resetSerialNumbers(self):
        for f in self.frames:
            f.serialNumber = 0
            
    def requestAll(self, delay=0):
        self.lock.acquire()
        if delay:
            self.delayBetweenFrames = delay
        # Put all image frames in the requested state
        for f in self.frames:
            f.request = True
        self.lock.release()

    def requestFrame(self, frameNum, delay=0):
        # Put the one frame in the requested state
        frame = self.frames[frameNum-1]
        self.lock.acquire()
        if delay:
            self.delayBetweenFrames = delay
        frame.request = True
        self.lock.release()

    def requestFrameAndNeighbors(self, frameNum, span=1, delay=0):
        # Put the frame and its neighbors in the requested state
        self.lock.acquire()
        if delay:
            self.delayBetweenFrames = delay
        frame = self.frames[frameNum-1]
        frame.request = True
        for f in self.frameNeighbors(frame, span):
            f.request = True
        self.lock.release()

    def publishSaveImage(self, frameNum, serialNum):
        imageId = ImageId()
        imageId.collectionNumber=self.collectionNum
        imageId.frameNumber=frameNum
        imageId.serialNumber=serialNum
        imageId.endCollection=False
        self.pubSaveImage.publish(imageId)
        print "{}: frame={}".format(self.collectionNum, frameNum)

    def publishEndCollection(self):
        imageId = ImageId()
        imageId.collectionNumber=self.collectionNum
        imageId.frameNumber=0
        imageId.serialNumber=0
        imageId.endCollection=True
        self.pubSaveImage.publish(imageId)
        print "{}: end collection".format(self.collectionNum)

    def capture(self, frameNum, serialNum):
        self.publishSaveImage(frameNum, serialNum)
        # Give camera some time to grab the image
        time.sleep(self.cameraDelay)
        
    def point(self, h, v):
        pantilt=PanTiltPos()
        pantilt.horizontalAngle=h
        pantilt.verticalAngle=v
        self.pubPanTilt.publish(pantilt)
        print "{}: x={} y={}".format(self.collectionNum, h, v)
        # Give servo some time to move
        time.sleep(self.servoDelay)

    def pointAndCapture(self, frm):
        self.point(frm.servoH, frm.servoV)
        self.capture(frm.number,frm.serialNumber)
        frm.serialNumber += 1
        
    def acquireFlatfield(self):
        # always return to 0,0 before pointing to flatfield.
        # This is a hack.  The pantilt module should take care of
        #  safely moving to the requested position.
        self.point(0,0)
        time.sleep(5)
        self.pointAndCapture(self.flatfield)
        # always return to 0,0 after pointing to flatfield.
        self.point(0,0)
