#!/usr/bin/env python
# license removed for brevity

# imports
import rospy
import general.jackal_custom as jc

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe      
        rospy.init_node('move_base_goal_pub', anonymous=True)  
        rate = rospy.Rate(10)         
        
        # Get command line arguments
        server_name = rospy.get_param('~server_name', '')
        odom_name = rospy.get_param('~odom_name', '')
        pos_goal = rospy.get_param('~pos_goal', '')
        ang_goal = rospy.get_param('~ang_goal', '')        
                
        result = jc.movebase_client(server_name,odom_name,pos_goal,ang_goal)
        
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")