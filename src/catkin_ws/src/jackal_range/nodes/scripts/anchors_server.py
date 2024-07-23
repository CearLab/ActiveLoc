#!/usr/bin/env python
# license removed for brevity

import rospy
import range.UWB_server as jr

if __name__ == '__main__':    
    
    try:
        
        # ros::init() the node
        rospy.init_node('Anchors_server', anonymous=True)
        
        # mode (0-move_base, 1-prop control)                
        anchors_pos = rospy.get_param('~anchors_pos', '')        
        topic_name = rospy.get_param('~topic_name', '')
        
        # instance jackal_move
        jr_instance = jr.UWB_server(topic_name, anchors_pos)
        rate = rospy.Rate(jr_instance.RATE)
        
        # call talker
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Anchors server failed")
