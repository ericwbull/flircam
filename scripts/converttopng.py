#!/usr/bin/env python
import sys
import numpy as np
import png
import flircam_util as fcu
from flircam.msg import ImageId

imageId = ImageId()
imageId.collectionNumber = int(sys.argv[1])
for frm in range(1,52):

    imageId.serialNumber = 0
    imageId.frameNumber = frm


    data = fcu.ReadCurrentImageFile(imageId)
    minVal = data.min()
    maxVal = data.max()
    scaleVal = maxVal - minVal
    data -= minVal
    data *= 255/scaleVal
    data = data.reshape((60,80))

    f = open("{}.png".format(fcu.GetCurrentImageFileName(imageId)),'wb')
    w = png.Writer(80,60,greyscale = True)
    w.write(f,data)
    f.close()
