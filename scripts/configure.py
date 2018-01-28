#!/usr/bin/env python

import rospy
import time

from flircam.msg import Configure


class ConfigureIt:
    def __init__(self):
        self.pubConfigure = rospy.Publisher('configure', Configure, queue_size=10)
        rospy.init_node('Configure', anonymous=True)
        
    def run(self):
        configure=Configure()
        configure.pixelChangeThreshold = 100
        configure.detectionThreshold = 1000
        configure.safeThreshold = 500
        configure.signalHistogramWeight = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
        self.pubConfigure.publish(configure)
        

if __name__ == '__main__':
    try:
        config = ConfigureIt()
        config.run()
    except rospy.ROSInterruptException:
        pass
