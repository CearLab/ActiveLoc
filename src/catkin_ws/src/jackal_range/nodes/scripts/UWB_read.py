#!/usr/bin/env python
# license removed for brevity

import rospy
import range.UWB_real as jr

if __name__ == '__main__':        
    
    try:
        
        # ros::init() the node
        rospy.init_node('UWB_read', anonymous=True)
        
        # instance jackal_move
        jr_instance = jr.UWB_real()
        rate = rospy.Rate(jr_instance.RATE)
        
        # call talker
        jr_instance.talker_read()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("UWB_read failed")
