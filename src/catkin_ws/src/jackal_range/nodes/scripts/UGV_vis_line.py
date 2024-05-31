#!/usr/bin/env python
# license removed for brevity

import rospy
import sys
import serial
import range.jackal_range as jr

if __name__ == '__main__':
    
    try:
        
        # ros::init() the node
        rospy.init_node('Line_visualization', anonymous=True)
        
        # node started
        rospy.loginfo('Visualize line in rviz')     
        
        # get params
        odom = rospy.get_param('~odom', '')        
        params_name = rospy.get_param('~params_name', '')        
        topic = rospy.get_param('~topic', '')
        
        rospy.loginfo(odom)        
        rospy.loginfo(params_name)        
        rospy.loginfo(topic)
        
        # call talker
        jr.publish_line_marker(odom,params_name,topic)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("UWB_setup failed")
