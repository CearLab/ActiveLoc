#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import serial
import time
import sys
import numpy as np
import re

# custom message import
from jackal_range.msg import RD_recap as RD
from jackal_range.msg import AnchorsInfo 
from general.jackal_general import JackalGeneral
from range.jackal_range import JackalRange


class UWB_server(JackalRange):
    
    def __init__(self,topic_name,anchors_pos):
        
        # Inherit
        super().__init__()
        
        # Convert the list of strings to a list of floats
        # Split the string by spaces and convert each to an integer
        numbers_str_list = anchors_pos.split()
        numbers_int_list = [int(num) for num in numbers_str_list]
        anchors_pos = np.array(numbers_int_list)
        
        # define a publisher for AnchorsInfo
        for i in range(len(self.anchors_params)):
            row = self.anchors_params[i]
            row[-3:] = anchors_pos[3*(i):3*(i+1)]
            self.anchors_params[i] = row
        
        self.anchors_topic = topic_name
        self.anchors_info_publisher = rospy.Publisher(self.anchors_topic, AnchorsInfo, queue_size=10)
        self.AnchorsInfoMsg = AnchorsInfo()
    
    def timer_callback_tf(self, event):
        pass
    
    def timer_callback(self, event):
        self.anchors_server()
        
    def anchors_server(self):
        
        self.AnchorsInfoMsg.header.stamp = rospy.Time.now()
        self.AnchorsInfoMsg.N_ID = []
        self.AnchorsInfoMsg.A_ID = []
        self.AnchorsInfoMsg.NUM_A = len(self.anchors_params)
        self.AnchorsInfoMsg.A_POS = np.concatenate([row[-3:] for row in self.anchors_params]).astype(float)
        self.anchors_info_publisher.publish(self.AnchorsInfoMsg)