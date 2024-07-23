#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import message_filters 
import numpy as np
import colorsys

# message import
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

# custom message import
from jackal_range.msg import RD_recap as RD
from general.jackal_general import JackalGeneral
from range.jackal_range import JackalRange

class JackalVis(JackalRange):
    
    def __init__(self):
        super().__init__()

    def publish_line_marker(self, odom, params_name, topic, color):
        sub = rospy.Subscriber(odom, Odometry, lambda msg: self.vis_line_callback(msg, params_name, topic, color))
        
    def vis_line_callback(self, msg, params_name, topic, color):
        
        pub = rospy.Publisher(topic+"/line", MarkerArray, queue_size=10)
        marker_array_line = MarkerArray()        
        
        try:
            tmp_anchors_params = rospy.get_param(params_name)
        except:
            rospy.logfatal('Params not found! (JackVis)')
            return
        
        N_A = len(tmp_anchors_params)
        color = np.asarray([float(x) for x in color.split()])  
        
        if color[0] == -1:
            color_gen = 1
        else:
            color_gen = 0
            color = tuple(color)   
        
        
        for i in range(N_A):
            
            if color_gen == 1:
                color = self.generate_color(i, N_A)     
                
            # set marker
            marker_line = Marker()    
            marker_line.header.frame_id = "world"
            marker_line.header.stamp = rospy.Time.now()
            marker_line.ns = "AN" + str(i) + "/line_markers"
            marker_line.id = i
            marker_line.type = Marker.LINE_STRIP
            marker_line.action = Marker.ADD            
            marker_line.scale.x = 0.01  # Line width
            marker_line.color.r = color[0]
            marker_line.color.g = color[1]
            marker_line.color.b = color[2]
            marker_line.color.a = color[3]
            marker_line.pose.orientation.x = 0
            marker_line.pose.orientation.y = 0
            marker_line.pose.orientation.z = 0
            marker_line.pose.orientation.w = 1
            
            pointANC = Point()
            pointANC.x = float(tmp_anchors_params[i][2])
            pointANC.y = float(tmp_anchors_params[i][3])
            pointANC.z = float(tmp_anchors_params[i][4])
            
            pointUGV = Point()
            pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            pos = self.DCM@(pos + self.trans)
            pointUGV.x = pos[0]
            pointUGV.y = pos[1]
            pointUGV.z = pos[2]
            
            x1, y1, z1 = pointANC.x, pointANC.y, pointANC.z
            x2, y2, z2 = pointUGV.x, pointUGV.y, pointUGV.z
            d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            
            if d > self.RANGE:
                pointANC.x = 0.0
                pointANC.y = 0.0
                pointANC.z = 0.0
                pointUGV.x = 0.0
                pointUGV.y = 0.0
                pointUGV.z = 0.0
                
            marker_line.points.append(pointANC)
            marker_line.points.append(pointUGV)                               
            marker_array_line.markers.append(marker_line)  
        pub.publish(marker_array_line)
        
    def publish_line_agents(self, color, subtopic, pubtopic):
        
        topic_name = '/' + self.namespace + subtopic
        odom_sub = message_filters.Subscriber(topic_name, Odometry)        
        
        rospy.logwarn('vis_line_MAS subscribers: '  + str(topic_name))
        
        pub = rospy.Publisher(pubtopic, MarkerArray, queue_size=10)
        self.get_ros_namespaces()
        
        if self.namespaces:  
            
            inter_agents_sub = []
            
            for i in range(len(self.namespaces)):                
                sub_name = '/' + self.namespaces[i] + subtopic
                rospy.logwarn('vis_line_MAS subscribers: '  + str(sub_name))
                inter_agents_sub.append(message_filters.Subscriber(sub_name, Odometry))
                
            list_subscribers = [odom_sub] + inter_agents_sub            
            ts = message_filters.ApproximateTimeSynchronizer(list_subscribers, queue_size=10, slop=0.1)
            ts.registerCallback(self.vis_agents_callback, color, pub)
        
    def vis_agents_callback(self, *args):
        
        if self.namespaces:
            expected_num_inputs = len(self.namespaces) + 3
        else:
            expected_num_inputs = 3
        if len(args) != expected_num_inputs:
            raise ValueError(f"Expected {expected_num_inputs} inputs, got {len(args)}")
        else:
            rospy.logwarn_once('Ranges received!')    
            
        color = args[-2]
        color = np.asarray([float(x) for x in color.split()])        
        
        if color[0] == -1:
            color_gen = 1
        else:
            color_gen = 0
            color = tuple(color)
            
        marker_array_line = MarkerArray()
        pos_local = args[0]
        pub = args[-1]
        
        if self.namespaces:
            for i in range(len(self.namespaces)):   
                pos_MAS = args[i+1]
                if color_gen == 1:
                    color = self.generate_color(i, len(self.namespaces))                    
                marker_line = Marker()    
                marker_line.header.frame_id = "world"
                marker_line.header.stamp = rospy.Time.now()
                marker_line.ns = self.namespaces[i] + "/line_markers"
                marker_line.id = i
                marker_line.type = Marker.LINE_STRIP
                marker_line.action = Marker.ADD            
                marker_line.scale.x = 0.01  # Line width
                marker_line.color.r = color[0]
                marker_line.color.g = color[1]
                marker_line.color.b = color[2]
                marker_line.color.a = color[3]
                marker_line.pose.orientation.x = 0
                marker_line.pose.orientation.y = 0
                marker_line.pose.orientation.z = 0
                marker_line.pose.orientation.w = 1
                
                pointANC = Point()
                pointANC.x = pos_local.pose.pose.position.x
                pointANC.y = pos_local.pose.pose.position.y
                pointANC.z = pos_local.pose.pose.position.z
                
                pointUGV = Point()                
                pointUGV.x = pos_MAS.pose.pose.position.x
                pointUGV.y = pos_MAS.pose.pose.position.y
                pointUGV.z = pos_MAS.pose.pose.position.z
                
                x1, y1, z1 = pointANC.x, pointANC.y, pointANC.z
                x2, y2, z2 = pointUGV.x, pointUGV.y, pointUGV.z
                d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
                
                if d > self.RANGE:
                    marker_line.points.append(pointANC)
            pub.publish(marker_array_line)    
    
    def generate_color(self,index, total_markers):
            hue = index / float(total_markers)  # Vary the hue between 0 and 1
            saturation = 1.0  # Full saturation
            value = 1.0  # Full brightness
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            return rgb + (1.0,)  # Return as (r, g, b, a) tuple with full opacity