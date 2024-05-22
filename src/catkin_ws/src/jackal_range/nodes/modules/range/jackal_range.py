#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import actionlib
import serial
import time
import sys
import message_filters 
import math
import roslib
import numpy as np
from collections import deque

import geometry_msgs.msg
import nav_msgs.msg

BUFSIZE = 6

def serialWrite(srl, data, sleepTime):
    
    # write
    srl.flushInput()
    srl.flushOutput()

    chunks = [data[i:i+BUFSIZE] for i in range(0, len(data)+1, BUFSIZE)]
    for ck in chunks:
        srl.write(ck)
        print('Send: ' + ck)

        t0 = time.time()
        dt = time.time() - t0
        while dt < sleepTime:
            dt = time.time() - t0
            if (srl.in_waiting > 0):
                data = srl.readline().decode().strip()
                print('Receive: ' + data.decode('utf-8'))

    srl.close()
    srl.open()




