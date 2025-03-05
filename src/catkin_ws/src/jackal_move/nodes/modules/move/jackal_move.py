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
import subprocess

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
    finished = False
    first_parse = True
    last_time = None
    mode = 0 # 1 = follow, 0 = movebase    
    
    def __init__(self):
        
        # inheriting
        super().__init__()
        self.ei = 0.0
        self.etheta_i = 0.0
        self.iter = 0
    def follow(self):
        
        # publish
        pub = rospy.Publisher(self.server_name, geometry_msgs.msg.Twist, queue_size=10)
        rospy.loginfo("Synchronization")

        # from string to array
        if self.mode == 0 and self.first_parse:
            self.p_goal = np.asarray([float(num) for num in self.p_goal.split()])
            self.a_goal = np.asarray([float(num) for num in self.a_goal.split()])
            # Convert Euler angles to quaternion
            self.quaternion = tft.quaternion_from_euler(self.a_goal[0], self.a_goal[1], self.a_goal[2])
            self.odom_msg = nav_msgs.msg.Odometry()
            rospy.loginfo(self.p_goal)
            rospy.loginfo(self.a_goal)
            self.first_parse = False                
        
        # current pos
        pos_sub = message_filters.Subscriber(self.odom_name, nav_msgs.msg.Odometry)
        # moving target
        if self.mode == 1:
            ref_sub = message_filters.Subscriber(self.leader_topic, nav_msgs.msg.Odometry)     
            rospy.loginfo('leader topic: ' + self.leader_topic)

        # Synchronize the messages
        if self.mode == 0:
            ts = message_filters.ApproximateTimeSynchronizer([pos_sub, pos_sub], queue_size=10, slop=0.1)
        else:
            ts = message_filters.ApproximateTimeSynchronizer([pos_sub, ref_sub], queue_size=10, slop=0.1)
            
        ts.registerCallback(self.ugv_control, pub)
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Check some condition to stop spinning
            if self.finished:
                rospy.loginfo("Condition met, stopping the spin.")
                return 1
            rate.sleep()
        
        
        

        return 0

    def update_position_goal(self, position):
        self.p_goal = [0.0, 0.0]
        self.p_goal[0] = position[0]
        self.p_goal[1] = position[1]
        self.finished = False
        rospy.logwarn("Goal updated: " + str(self.p_goal))
        
    def ugv_control(self, pos, ref, pub):   
        
        #enforced the rate using return when the diff between the current system time the the last callback is too small
        if self.last_time is not None:
            if rospy.Time.now().to_sec() - self.last_time < (1 / self.RANGE):
                return 0
        
        # create message
        cmd = geometry_msgs.msg.Twist()
        
        # integral error
        self.iter += 1
        
        # integral reset
        if self.iter % 1000 == 0:
            self.ei = 0.0 
            self.etheta_i = 0.0

        # current pos and orientation
        x = pos.pose.pose.position.x
        y = pos.pose.pose.position.y
        orientation = pos.pose.pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, theta = tft.euler_from_quaternion([qx, qy, qz, qw])

        # desired position
        if self.mode == 1:
            x_goal = ref.pose.pose.position.x
            y_goal = ref.pose.pose.position.y
        else:
            x_goal = self.p_goal[0]
            y_goal = self.p_goal[1]
            
        # gains   
        Kpos = 0.1
        Kpos_i = 0.01
        Ktheta = 1            
        Ktheta_i = 0.1
        thresh_pos = 0.1         

        # position error
        ex = x_goal - x
        ey = y_goal - y
        
        # distance and angle to goal
        ed = np.linalg.norm(np.array((ex, ey)))
        self.ei =+ ed
        
        theta_goal = math.atan2(ey, ex)  # Desired heading
        etheta = math.atan2(math.sin(theta_goal - theta), math.cos(theta_goal - theta))
        self.etheta_i += etheta  
        
        self.last_time = rospy.Time.now().to_sec()
        
        # compute control
        if abs(ed) >= thresh_pos:
            
            rospy.logwarn_once('Distance control')
                                    
            v_x = np.linalg.norm(Kpos * ed + Kpos_i * self.ei)                        
            omega_z = Ktheta*etheta + Ktheta_i*self.etheta_i 
            
            # populate message
            cmd.linear.x = v_x
            cmd.angular.z = omega_z
                        
            pub.publish(cmd)

            return 0
        else:                                                
            
            rospy.logwarn_once('Target reached. Stopping the node.')
            self.finished = True
            # subprocess.check_call(['rosnode', 'kill', rospy.get_name()])
            return 1

        

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

    def imu_remapper(self, data, pub, ns, sigma=(0.1, 0.01)):
        # Extract odometry information from feedback
        data.header.frame_id = str(ns) + data.header.frame_id
        
        # add noise to the data
        # if sigma is a string, split it
        if isinstance(sigma, str):
            sigma = np.asarray([float(num) for num in sigma.split()])
        data.angular_velocity.z = data.angular_velocity.z + np.random.normal(0, sigma[0])
        data.linear_acceleration.x = data.linear_acceleration.x + np.random.normal(0, sigma[1])
        data.linear_acceleration.y = data.linear_acceleration.y + np.random.normal(0, sigma[1])
        data.linear_acceleration.z = data.linear_acceleration.z + np.random.normal(0, sigma[1])

        # Publish the odometry message
        pub.publish(data)

        return 0