#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import actionlib
import numpy as np
import message_filters 
import math

import tf2_ros
import tf.transformations as tft

from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseResult, MoveBaseGoal
import geometry_msgs.msg
import nav_msgs.msg

# circulant motion
def follow(leader_topic, server_name, odom_name, mode):           
    
    # publish
    pub = rospy.Publisher(server_name,geometry_msgs.msg.Twist, queue_size=10)
    rospy.loginfo("Synchronization")        
    
    # Subscribe to the Odometry topic       
    if mode==0:
        # move_base
        ref_sub = rospy.Subscriber(leader_topic, nav_msgs.msg.Odometry, follow_odom_callback, (server_name, odom_name, pub, mode))
    else:    
        # moving target    
        ref_sub = message_filters.Subscriber(leader_topic, nav_msgs.msg.Odometry)
        pos_sub = message_filters.Subscriber(odom_name, nav_msgs.msg.Odometry)    

        # Synchronize the messages
        ts = message_filters.ApproximateTimeSynchronizer([ref_sub, pos_sub], queue_size=10, slop=0.1)        
        ts.registerCallback(follow_odom_callback, pub)        
    
    rospy.spin()        
    
    return 0

def follow_odom_callback(ref, pos, pub, mode):
    
    # Callback function to print Odometry messages
    # rospy.loginfo("Debug: Received Odometry")
    
    # send goal     
    if mode==0:
        # move_base
        p_goal = np.array((4,4))
        a_goal = np.array((0,0,0))
        result = movebase_client(ref,pos,p_goal,a_goal)
    else:
        # moving target
        result = ugv_control(ref,pos,pub)
    
    return 0

# PID control to reach a generic XY target
def ugv_control(ref,pos,pub):
    
    # create message
    cmd = geometry_msgs.msg.Twist()
    
    # current pos and orientation
    x = pos.pose.pose.position.x
    y = pos.pose.pose.position.y
    orientation = pos.pose.pose.orientation
    qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
    _, _, theta = tft.euler_from_quaternion([qx, qy, qz, qw])
    
    # desired position
    x_goal = ref.pose.pose.position.x
    y_goal = ref.pose.pose.position.y
    
    # position error
    ex =  x_goal - x
    ey =  y_goal - y    
    
    # distance and angle to goal
    d = math.sqrt(ex**2 + ey**2)        
    theta_goal = math.atan2(ey, ex)
    
    # Calculate angular error
    e_theta = theta_goal - theta
    
    # gains
    Kv = 0.1
    Ko = 1
    
    # compute control    
    v_x = Kv * d
    omega_z = Ko * e_theta
    
    # populate message
    cmd.linear.x = v_x
    cmd.angular.z = omega_z
    
    # debug
    # rospy.loginfo("Debug: publish on server") 
    pub.publish(cmd)       
    
    return 0

# send goal through move_base
def movebase_client(server_name,odom_name,pos_goal,ang_goal):
    
    # Initialize the action client
    client = actionlib.SimpleActionClient(server_name, MoveBaseAction)
    
    # debug
    rospy.loginfo("Debug: created action client")
    rospy.loginfo("Debug: waiting for server")
    
    # Wait until the action server has started up and started listening for goals
    client.wait_for_server()
    
    # debug
    rospy.loginfo("Debug: server reached")
    
    # Create a goal to send to the action server
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = odom_name
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # get pos
    try:
        p_goal = pos_goal.split(' ')
        p_goal = [float(x) for x in p_goal]
    except:
        rospy.loginfo('Pos: not received a string. Assuming float array')    
        p_goal = pos_goal
    rospy.loginfo(str(p_goal))
    
    # get angle
    try:
        a_goal = ang_goal.split(' ')
        a_goal = [float(x) for x in a_goal]
    except:
        rospy.loginfo('Ang: not received a string. Assuming float array')  
        a_goal = ang_goal  
    quaternion = tft.quaternion_from_euler(a_goal[0], a_goal[1], a_goal[2])
    rospy.loginfo(str(quaternion))
    
    # in which reference frame?
    goal.target_pose.pose.position.x = p_goal[0]
    goal.target_pose.pose.position.y = p_goal[1]
    
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    
    # Send the goal to the action server
    client.send_goal(goal)
    
    # debug
    rospy.loginfo("Debug: goal sent")

    # Wait for the server to finish performing the action
    client.wait_for_result()                    

    # Print the result of executing the action
    return client.get_result()