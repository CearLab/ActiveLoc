#!/usr/bin/env python
# license removed for brevity

import rospy
import sys
import serial
import numpy as np
import range.jackal_range as jr

if __name__ == '__main__':        
    
    try:
        
        # ros::init() the node
        rospy.init_node('MAS_line_visualization', anonymous=True)
        rospy.Rate(10)
        
        # define class instance
        jr_instance = jr.JackalRange()
        
        # node started
        rospy.loginfo('Visualize line in rviz')     
        
        # get params        
        color = rospy.get_param('~color', '')
        subtopic = rospy.get_param('~subtopic', '')
        pubtopic = rospy.get_param('~pubtopic', '')
        
        # call talker
        jr_instance.publish_line_agents(color,subtopic,pubtopic)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("MAS visualization failed")
