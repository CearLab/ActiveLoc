#!/usr/bin/env python
# license removed for brevity

import rospy
import sys
import serial
import range.jackal_range as jr

if __name__ == '__main__':        
    
    try:
        
        # ros::init() the node
        rospy.init_node('UWB_read', anonymous=True)
        
        # define class instance
        jr_instance = jr.JackalRange()
        
        # node started
        rospy.loginfo('UWB read')                
        
        # call talker
        jr_instance.talker_read()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("UWB_read failed")
