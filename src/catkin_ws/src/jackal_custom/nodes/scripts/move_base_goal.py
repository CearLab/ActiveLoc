#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseActionGoal

def movebase_client(topic_name):

    # create publisher
    rospy.init_node('move_base_goal_pub', anonymous=True)
    pub = rospy.Publisher(topic_name, MoveBaseActionGoal, queue_size=10)
    rate = rospy.Rate(10)  # 1 Hz

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = "UGV01/base_link"
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.goal.target_pose.pose.position.x = 0.5
    # No rotation of the mobile base frame w.r.t. map frame
    goal.goal.target_pose.pose.orientation.w = 1.0
            
    while not rospy.is_shutdown():
        pub.publish(goal)
        rate.sleep()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        
        import sys
        if len(sys.argv) < 2:
            print("Usage: move_base_goal.py <topic_name>")
            sys.exit(1)

        topic_name = sys.argv[1]
        
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe        
        result = movebase_client(topic_name)
        
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")