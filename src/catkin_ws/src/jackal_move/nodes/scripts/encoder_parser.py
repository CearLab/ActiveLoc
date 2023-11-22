#!/usr/bin/env python

import rospy
import move.jackal_move as jm
import nav_msgs.msg

if __name__ == '__main__':
    
    try:        
        
        # Initializes a rospy node 
        rospy.init_node('encoder_parser', anonymous=True)         
        rate = rospy.Rate(10)
        
        feedback_topic = rospy.get_param('~feedback_topic', '')
        publish_topic = rospy.get_param('~publish_topic', '')

        # Publisher for the Odometry message
        odom_pub = rospy.Publisher(publish_topic, nav_msgs.msg.Odometry, queue_size=10)

        # Subscriber to the feedback topic
        sub = rospy.Subscriber(feedback_topic, jm.Feedback, lambda data: jm.encoder_parser(data, odom_pub))

        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Encoder parser failed.")
