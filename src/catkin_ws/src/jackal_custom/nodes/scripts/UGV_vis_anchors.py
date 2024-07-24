#!/usr/bin/env python
# license removed for brevity

import rospy
from vis.jackal_vis import JackalVis

if __name__ == '__main__':        
    
    try:
        
        # ros::init() the node
        rospy.init_node('Line_visualization', anonymous=True)
        
        # get params
        odom_topic = ''
        anchors_topic = rospy.get_param('~anchors_topic', '')
        pub_topic = rospy.get_param('~pub_topic', '')        
        color = rospy.get_param('~color', '')
        
        # instance jackal_move
        jvis_instance = JackalVis(anchors_topic,odom_topic,pub_topic,color)
        rospy.Rate(jvis_instance.RATE)
        
        # call talker
        jvis_instance.publish_anchors_marker()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Visualization failed")
