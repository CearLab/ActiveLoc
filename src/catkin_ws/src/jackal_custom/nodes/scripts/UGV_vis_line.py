#!/usr/bin/env python
# license removed for brevity

import rospy
from vis.jackal_vis import JackalVis

if __name__ == '__main__':        
    
    try:
        
        # ros::init() the node
        rospy.init_node('Line_visualization', anonymous=True)
        
        # instance jackal_move
        jvis_instance = JackalVis()
        rospy.Rate(jvis_instance.RATE)
        
        # get params
        odom = rospy.get_param('~odom', '')        
        params_name = rospy.get_param('~params_name', '')        
        topic = rospy.get_param('~topic', '')        
        color = rospy.get_param('~color', '')
        
        # call talker
        jvis_instance.publish_line_marker(odom,params_name,topic,color)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Visualization failed")
