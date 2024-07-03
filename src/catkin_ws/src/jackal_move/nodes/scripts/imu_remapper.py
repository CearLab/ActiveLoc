#!/usr/bin/env python

import rospy
import move.jackal_move as jm
import nav_msgs.msg
import sensor_msgs.msg
from geometry_msgs.msg import TransformStamped
import tf

if __name__ == '__main__':
    
    try:        
        
        # Initializes a rospy node 
        rospy.init_node('imu_remapper', anonymous=True)  
        rate = rospy.Rate(10)
          
        # Create a TransformBroadcaster object
        broadcaster = tf.TransformBroadcaster()     
        
        feedback_topic = rospy.get_param('~subscribe_topic', '')
        publish_topic = rospy.get_param('~publish_topic', '')
        ns = rospy.get_param('~ns', '')

        # Publisher for the Odometry message
        odom_pub = rospy.Publisher(publish_topic, sensor_msgs.msg.Imu, queue_size=10)

        # Subscriber to the feedback topic
        sub = rospy.Subscriber(feedback_topic, sensor_msgs.msg.Imu, lambda data: jm.imu_remapper(data, odom_pub, ns))

        # loop
        while not rospy.is_shutdown():
        
            rate.sleep()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Encoder parser failed.")
