#!/usr/bin/python

import pigpio
import time
import curses
import atexit

pi = pigpio.pi()

if not pi.connected:
    exit()

up = 1500
right = 1500
UPKEY = 'w'
DNKEY = 'x'
LEFTKEY = 'a'
RIGHTKEY = 'd'
EXITKEY = 'g'
FINEKEY = 'f'
COARSEKEY = 'c'

incr = 100
while True:

    print "right={}".format(right)
    print "up={}".format(up)
    pi.set_servo_pulsewidth(18, right)
    pi.set_servo_pulsewidth(13, up)

    time.sleep(0.01)
    c = raw_input()
    print c
    if c == FINEKEY:
        incr=10
    if c == COARSEKEY:
        incr=100
    if c == EXITKEY:
        break
    if c == UPKEY:
        up += incr
    if c == DNKEY:
        up -= incr
    if c == RIGHTKEY:
        right += incr
    if c == LEFTKEY:
        right -= incr
        
