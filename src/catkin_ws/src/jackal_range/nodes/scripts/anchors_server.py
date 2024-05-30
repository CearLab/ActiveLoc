#!/usr/bin/env python
# license removed for brevity

import rospy
import sys
import serial
import range.jackal_range as jr

if __name__ == '__main__':
    
    try:
        
        # ros::init() the node
        rospy.init_node('Anchors_server', anonymous=True)
        
        # node started
        rospy.loginfo('Anchors setup')     
        
        # mode (0-move_base, 1-prop control)                
        anchors_pos = rospy.get_param('~anchors_pos', '')        
        topic = rospy.get_param('~topic', '')
        
        rospy.loginfo(anchors_pos)        
        rospy.loginfo(topic)
        
        # call talker
        jr.anchors_server(anchors_pos,topic)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("UWB_setup failed")
