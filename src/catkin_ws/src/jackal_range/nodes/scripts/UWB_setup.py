#!/usr/bin/env python
# license removed for brevity

import rospy
import sys
import serial
import range.jackal_range as jr

if __name__ == '__main__':
    
    try:
        
        # ros::init() the node
        rospy.init_node('UWB_setup', anonymous=True)
        
        # node started
        rospy.loginfo('UWB setup')                
        
        # call talker
        jr.talker_setup()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("UWB_setup failed")
