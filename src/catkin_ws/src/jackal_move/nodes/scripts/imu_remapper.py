#!/usr/bin/env python

import rospy
from move.jackal_move import JackalMove
import sensor_msgs.msg

if __name__ == '__main__':
    
    try:
        
        # Initializes a rospy node 
        rospy.init_node('imu_remapper', anonymous=True)
        
        # instance jackal_move
        jm_instance = JackalMove()
        rate = rospy.Rate(jm_instance.RATE)
        
        feedback_topic = rospy.get_param('~subscribe_topic', '')
        publish_topic = rospy.get_param('~publish_topic', '')
        ns = rospy.get_param('~ns', '')
        sigma = rospy.get_param('~ImuSigma', '')

        # Publisher for the Odometry message
        odom_pub = rospy.Publisher(publish_topic, sensor_msgs.msg.Imu, queue_size=10)

        # Subscriber to the feedback topic
        sub = rospy.Subscriber(feedback_topic, sensor_msgs.msg.Imu, lambda data: jm_instance.imu_remapper(data, odom_pub, ns, sigma))

        # loop
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Encoder parser failed.")
