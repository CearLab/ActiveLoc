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
        rospy.init_node('Line_visualization', anonymous=True)
        rospy.Rate(10)
        
        # define class instance
        jr_instance = jr.JackalRange()
        
        # node started
        rospy.loginfo('Visualize line in rviz')     
        
        # get params
        odom = rospy.get_param('~odom', '')        
        params_name = rospy.get_param('~params_name', '')        
        topic = rospy.get_param('~topic', '')        
        color = rospy.get_param('~color', '')
        
        rospy.loginfo(odom)        
        rospy.loginfo(params_name)        
        rospy.loginfo(topic)
        rospy.loginfo(color)
        
        # call talker
        jr_instance.publish_line_marker(odom,params_name,topic,color)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("UWB_setup failed")
