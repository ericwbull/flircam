from enum import Enum
import numpy as np
import struct

class StreamID(Enum):
    RETURN_INFO = 1
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
    RETURN_STATUS = 14
    RETURN_ALERT = 15

    
def ImageIdToString(imageId):
    return "{}/{}.{}".format(imageId.collectionNumber, imageId.frameNumber, imageId.serialNumber)

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

def GetCurrentImageFileName(imageId):
    folderNum = imageId.collectionNumber
    frameNum = imageId.frameNumber
    serialNum = imageId.serialNumber
    return "/tmp/flircam/{}/{}.{}".format(folderNum, frameNum, serialNum)

def GetBaselineImageFileName(imageId):
    return "{0}.baseline".format(GetCurrentImageFileName(imageId))

def GetDetectionImageFileName(imageId):
    return "{0}.detection".format(GetCurrentImageFileName(imageId))

def ReadCurrentImageFile(imageId):
    return ReadUInt16Image(GetCurrentImageFileName(imageId))
    
def ReadBlockFromFile(fileName, num, size, endPos):
    imageFile = file(fileName,"rb")
    pos = num*size
    imageFile.seek(pos)
    if (pos + size > endPos):
        size = endPos - pos

    print "Block filename={} pos={} size={}".format(fileName,pos,size)
    fileData = imageFile.read(size)
    return fileData

def ReadUInt16Image(fileName):
    f = open(fileName, 'rb')
    data = f.read(9600)
    dataH = struct.unpack_from("4800H", data, 0)
    return np.array(dataH, dtype=np.uint16)
    
