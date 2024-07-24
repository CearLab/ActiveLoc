#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import serial
import time
import sys
import numpy as np
import re
import message_filters

# custom message import
from jackal_range.msg import RD_recap as RD
from jackal_range.msg import AnchorsInfo 
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
        
        range_topic =  '/' + self.namespace + '/Anchors/range'
        self.anchors_sub = message_filters.Subscriber(range_topic, RD)
        ts = message_filters.ApproximateTimeSynchronizer([self.anchors_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.setup_anchor_params)
        
        self.anchors_topic = topic_name
        self.anchors_info_publisher = rospy.Publisher(self.anchors_topic, AnchorsInfo, queue_size=10)
        self.AnchorsInfoMsg = AnchorsInfo()
    
    def timer_callback_tf(self, event):
        pass
    
    def timer_callback(self, event):
        self.anchors_server()
        
    def setup_anchor_params(self, msg):
        for i in range(self.NUM_A):
            row = self.anchors_params[i]
            row[-3:] = msg.A_POS[3*i:3*(i+1)]
            self.anchors_params[i] = row
        
    def anchors_server(self):
        
        self.AnchorsInfoMsg.header.stamp = rospy.Time.now()
        self.AnchorsInfoMsg.N_ID = []
        self.AnchorsInfoMsg.A_ID = []
        self.AnchorsInfoMsg.NUM_A = len(self.anchors_params)
        try:
            self.AnchorsInfoMsg.A_POS = np.concatenate([row[-3:] for row in self.anchors_params]).astype(float)
        except:
            rospy.logwarn_once('No anchor positions')
            return
        self.anchors_info_publisher.publish(self.AnchorsInfoMsg)