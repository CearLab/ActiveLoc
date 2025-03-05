#!/usr/bin/env python
# license removed for brevity

# imports
import rospy
import move.jackal_move as jm

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:        
        
        # Initializes a rospy node 
        rospy.init_node('follower', anonymous=True)         
        
        # instance jackal_move
        jm_instance = jm.JackalMove()
        rate = rospy.Rate(jm_instance.RATE)
        
        # mode (0-move_base, 1-prop control)                
        jm_instance.mode = rospy.get_param('~mode', '')        
        
        # Get command line arguments
        # static goal
        if jm_instance.mode == 0:            
            jm_instance.server_name = rospy.get_param('~server_name', '')            
            jm_instance.odom_name = rospy.get_param('~odom_name', '')              
            jm_instance.p_goal = rospy.get_param('~goal_pos', '')
            jm_instance.a_goal = rospy.get_param('~goal_ang', '')
        # moving target
        else:
            jm_instance.leader_topic = rospy.get_param('~leader_topic', '')
            jm_instance.server_name = rospy.get_param('~server_name', '')
            jm_instance.odom_name = rospy.get_param('~odom_name', '')
            jm_instance.p_goal = ''
            jm_instance.a_goal =  ''                    
        
        # call function
        result = jm_instance.follow()
        
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")