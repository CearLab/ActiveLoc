#!/usr/bin/env python

import rospy
import move.jackal_move as jm
import sensor_msgs.msg

if __name__ == '__main__':
    
    try:    
        
        # instance jackal_move
        jm_instance = jm.JackalMove()
        
        # Initializes a rospy node 
        rospy.init_node('imu_remapper', anonymous=True)  
        rate = rospy.Rate(jm_instance.RATE)
        
        feedback_topic = rospy.get_param('~subscribe_topic', '')
        publish_topic = rospy.get_param('~publish_topic', '')
        ns = rospy.get_param('~ns', '')

        # Publisher for the Odometry message
        odom_pub = rospy.Publisher(publish_topic, sensor_msgs.msg.Imu, queue_size=10)

        # Subscriber to the feedback topic
        sub = rospy.Subscriber(feedback_topic, sensor_msgs.msg.Imu, lambda data: jm.imu_remapper(data, odom_pub, ns))

        # loop
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Encoder parser failed.")
