#!/usr/bin/env python
# license removed for brevity

import rospy
from range.UWB_sim import UWB_sim

# wrapper
if __name__ == '__main__':
    
    try:
        
        # ros::init() the node
        rospy.init_node('UWB_SIM', anonymous=True)
        
        # get anchors params
        anchors_topic_name = rospy.get_param('~anchors_topic_name', '')  
        odometry_topic_name = rospy.get_param('~odometry_topic_name', '')        
        range_cov = rospy.get_param('~range_meas_cov', 0.0)
        
        # instance jackal_move
        uwb_instance = UWB_sim(anchors_topic_name, odometry_topic_name, range_cov)
        rate = rospy.Rate(uwb_instance.RATE)
        
        # call talker
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("UWB simulation failed")
