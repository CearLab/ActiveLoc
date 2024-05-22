#!/usr/bin/env python
# license removed for brevity

# imports
import rospy
import general.jackal_custom as jc
import numpy as np

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:        
        
        # Initializes a rospy node 
        rospy.init_node('follower', anonymous=True)         
        rate = rospy.Rate(10)        
        
        # mode (0-move_base, 1-prop control)                
        mode = rospy.get_param('~mode', '')        
        
        # Get command line arguments
        # static goal
        if mode == 0:
            leader_topic = ''
            server_name = rospy.get_param('~server_name', '')
            odom_name = 'map'    
            goal_pos = rospy.get_param('~goal_pos', '')
            goal_ang = rospy.get_param('~goal_ang', '')
        # moving target
        else:
            leader_topic = rospy.get_param('~leader_topic', '')
            server_name = rospy.get_param('~server_name', '')
            odom_name = rospy.get_param('~odom_name', '')
            goal_pos = ''
            goal_ang =  ''                    
        
        # call function
        result = jc.follow(leader_topic, server_name, odom_name, goal_pos, goal_ang, mode)     
        
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")