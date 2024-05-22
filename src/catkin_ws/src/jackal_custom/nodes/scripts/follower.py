#!/usr/bin/env python
# license removed for brevity

# imports
import rospy
import general.jackal_custom as jc

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:        
        
        # Initializes a rospy node 
        rospy.init_node('follower', anonymous=True)         
        rate = rospy.Rate(10)        
        
        # Get command line arguments
        leader_topic = rospy.get_param('~leader_topic', '')
        server_name = rospy.get_param('~server_name', '')
        odom_name = rospy.get_param('~odom_name', '')        
                        
        # mode (0-move_base, 1-prop control)                
        mode = rospy.get_param('~mode', '')        
        
        # call function
        result = jc.follow(leader_topic, server_name, odom_name, mode)        
        
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")