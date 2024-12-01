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
    
class JackalRange(JackalGeneral):
    
    # SERIAL BUFFER LENGTHS
    BUFSIZE_SET = 6
    BUFSIZE_GET = 4
    
    # anchors ID dictionary
    ID = ['AN0',
          'AN1',
          'AN2',
          'AN3'
          ]
    
    # range
    RANGE = 6
    
    # general anchors_params with anchor positions
    anchors_params = [
        ['UKN', ID[0], 0.0, 0.0, 0.0],
        ['UKN', ID[1], 0.0, 0.0, 0.0],
        ['UKN', ID[2], 0.0, 0.0, 0.0],
        ['UKN', ID[3], 0.0, 0.0, 0.0]
    ]

    # general anchors_params with anchor positions
    tag_params = [
        ['UKN', ID[0], 0.0],
        ['UKN', ID[1], 0.0],
        ['UKN', ID[2], 0.0],
        ['UKN', ID[3], 0.0]
    ]
    
    N_A = len(anchors_params)
    
    anchors_topic = ''

    def __init__(self):
        
        # Inherit
        super().__init__()