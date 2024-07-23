#!/usr/bin/env python
# license removed for brevity

import rospy
from vis.jackal_vis import JackalVis

if __name__ == '__main__':        
    
    try:
        
        # ros::init() the node
        rospy.init_node('MAS_line_visualization', anonymous=True)
        
        # instance jackal_move
        jvis_instance = JackalVis()
        rospy.Rate(jvis_instance.RATE)
        
        # get params        
        color = rospy.get_param('~color', '')
        subtopic = rospy.get_param('~subtopic', '')
        pubtopic = rospy.get_param('~pubtopic', '')
        
        # call talker
        jvis_instance.publish_line_agents(color,subtopic,pubtopic)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("MAS visualization failed")
