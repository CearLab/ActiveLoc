#!/usr/bin/env python
# license removed for brevity

# imports
import rospy
import move.jackal_move as jm
STEPS = [[1.0 , 4.0], [0.0 , 4.0], [0.0 , 0.3], [0.0 , -4.0]]
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
        jm_instance.first_parse = False
        jm_instance.server_name = rospy.get_param('~server_name', '')            
        jm_instance.odom_name = rospy.get_param('~odom_name', '')              
        jm_instance.p_goal = rospy.get_param('~goal_pos', '')
        jm_instance.a_goal = rospy.get_param('~goal_ang', '')                  
        for step in STEPS:
            rospy.logwarn("Step: %s", step)
            jm_instance.update_position_goal(step)
            result = jm_instance.follow()
            # wait 2 seconds
            # rate.sleep(2)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")