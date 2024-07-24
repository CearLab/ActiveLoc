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

from jackal_msgs.msg import Feedback
from range.jackal_range import JackalRange

class JackalMove(JackalRange):
    
    # setup attributes
    leader_topic = ''
    server_name = ''
    odom_name = ''
    p_goal = ''
    a_goal = ''
    mode = 0
    
    def __init__(self):
        
        # inheriting
        super().__init__()

    def follow(self):
        # publish
        pub = rospy.Publisher(self.server_name, geometry_msgs.msg.Twist, queue_size=10)
        rospy.loginfo("Synchronization")

        # from string to array
        p_goal = np.asarray([float(num) for num in self.p_goal.split()])
        a_goal = np.asarray([float(num) for num in self.a_goal.split()])
        rospy.loginfo(p_goal.size)
        rospy.loginfo(a_goal.size)

        # Subscribe to the Odometry topic
        if self.mode == 0:
            # move_base
            ref_sub = self.movebase_client(self.server_name, 'map', p_goal, a_goal)
        else:
            # moving target
            ref_sub = message_filters.Subscriber(self.leader_topic, nav_msgs.msg.Odometry)
            pos_sub = message_filters.Subscriber(self.odom_name, nav_msgs.msg.Odometry)

            # Synchronize the messages
            ts = message_filters.ApproximateTimeSynchronizer([ref_sub, pos_sub], queue_size=10, slop=0.1)
            ts.registerCallback(self.ugv_control, pub)
            rospy.spin()

        return 0

    def ugv_control(self, ref, pos, pub):
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
        ex = x_goal - x
        ey = y_goal - y

        # distance and angle to goal
        d = math.sqrt(ex ** 2 + ey ** 2)
        theta_goal = math.atan2(ey, ex)

        # Calculate angular error
        e_theta = theta_goal - theta

        # gains
        Kv = 0.1
        Ko = 0.5

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

    def movebase_client(self, server_name, odom_name, p_goal, a_goal):
        # Initialize the action client
        client = actionlib.SimpleActionClient(server_name, MoveBaseAction)

        # debug
        rospy.loginfo("Debug: created action client")

        # Wait until the action server has started up and started listening for goals
        client.wait_for_server()

        # debug
        rospy.loginfo("Debug: server reached")

        # Create a goal to send to the action server
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = odom_name
        goal.target_pose.header.stamp = rospy.Time.now()

        # get quaternion
        rospy.logwarn(str(a_goal))
        quaternion = tft.quaternion_from_euler(a_goal[0], a_goal[1], a_goal[2])

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

    def encoder_parser(self, data, pub):
        # Extract odometry information from feedback
        odom = nav_msgs.msg.Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # get data from encoders
        travel_left = data.drivers[0].measured_travel
        travel_right = data.drivers[1].measured_travel

        # chassis data [m] from xacro
        width = 0.320
        wheelbase = 0.262
        track = 0.37559
        wheel_vertical_offset = 0.0345

        # initial position
        x0 = 0.0
        y0 = 0.0
        z0 = wheel_vertical_offset

        # initial orientation
        R0 = 0.0
        P0 = 0.0
        Y0 = 0.0

        # compute current position
        x, y, Y = self.update_position(x0, y0, Y0, travel_left, travel_right, width)

        # keep the z
        z = z0

        # keep Roll and Pitch
        R = R0
        P = P0

        # Position
        odom.pose.pose.position = geometry_msgs.msg.Point(x, y, z)

        # get quaternion
        quaternion = tft.quaternion_from_euler(R, P, Y)
        odom.pose.pose.orientation = geometry_msgs.msg.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

        # Publish the odometry message
        pub.publish(odom)

        # rospy.logwarn('Encoder parsed: time '+ str(rospy.Time.now()))

        return 0

    def update_position(self, x0, y0, theta0, dL, dR, L):
        # Compute distance traveled by the center of the robot
        dC = (dL + dR) / 2.0

        # Compute change in orientation
        delta_theta = (dR - dL) / L

        # If delta_theta is not zero, compute the new position
        if delta_theta != 0:
            R = dC / delta_theta
            x = x0 + R * (math.sin(theta0 + delta_theta) - math.sin(theta0))
            y = y0 - R * (math.cos(theta0 + delta_theta) - math.cos(theta0))
            theta = theta0 + delta_theta
        else:  # If delta_theta is zero, the robot is moving straight
            x = x0 + dC * math.cos(theta0)
            y = y0 + dC * math.sin(theta0)
            theta = theta0

        return x, y, theta

    def imu_remapper(self, data, pub, ns):
        # Extract odometry information from feedback
        data.header.frame_id = str(ns) + data.header.frame_id

        # Publish the odometry message
        pub.publish(data)

        return 0