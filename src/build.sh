#!/bin/sh
g++ -std=c++11 -I/home/pi/git/LePi/lib/leptonAPI/include FrameGrabber.cpp  -L/home/pi/git/LePi/install/lib -lleptonAPI -lleptonSDK -lpng -lbcm2835 -lpthread
