#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import actionlib
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

import tf2_ros
import tf.transformations as tft

from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseResult, MoveBaseGoal
import geometry_msgs.msg
import nav_msgs.msg

# tf broadcaster
def handle_pose(msg, name):    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = name
    t.transform.translation.x = -msg.pose.pose.position.x
    t.transform.translation.y = -msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z    
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w    

    br.sendTransform(t)
    
    return 0