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
import message_filters

# custom message import
from jackal_range.msg import RD_recap as RD
from jackal_range.msg import AnchorsInfo
from general.jackal_general import JackalGeneral
from range.jackal_range import JackalRange

class JackalVis(JackalRange):
    
    marker_array_line = MarkerArray()
    marker_array_anchor = MarkerArray()
    
    def __init__(self, anchors_topic_name, odometry_topic_name, pub_topic, color):
        
        super().__init__()
        
        self.sub_anchors_name = anchors_topic_name
        self.sub_odometry_name = odometry_topic_name
        self.pub_topic = pub_topic
        self.color = np.asarray([float(x) for x in color.split()]) 
        
        # class constructor        
        self.namespace_handler(None)
        
        # other stuff
        rospy.sleep(2)
        self.get_ros_namespaces()

    def publish_line_marker(self):
        
        # Publisher and subscriber local
        self.uwb_pub_topic_markers = rospy.Publisher(self.pub_topic, MarkerArray, queue_size=10)
        
        # get the ground truth topic and the publish topic
        odometry_topic_local = self.sub_odometry_name
        self.ground_truth_sub_local = message_filters.Subscriber(odometry_topic_local, Odometry)
        anchors_topic_local =  self.sub_anchors_name
        self.anchors_sub_local = message_filters.Subscriber(anchors_topic_local, AnchorsInfo)
        
        list_subscribers = [self.ground_truth_sub_local] + [self.anchors_sub_local]
        
        rospy.logwarn([str(tmp.topic) for tmp in list_subscribers])
        ts = message_filters.ApproximateTimeSynchronizer(list_subscribers, queue_size=10, slop=0.1)
        ts.registerCallback(self.vis_line_callback)
        
    def vis_line_callback(self, *args):
        
        odom = args[0]
        anchors = args[1]
        
        if self.color[0] == -1:
            color_gen = 1
        else:
            color_gen = 0
            color = tuple(self.color)   
            
        self.marker_array_line = MarkerArray()
        
        for i in range(self.N_A):
            
            if color_gen == 1:
                color = self.generate_color(i, self.N_A)     
                
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
            pointANC.x = float(anchors.A_POS[3*i+0])
            pointANC.y = float(anchors.A_POS[3*i+1])
            pointANC.z = float(anchors.A_POS[3*i+2])
            
            pointUGV = Point()
            pos = np.array([odom.pose.pose.position.x, 
                            odom.pose.pose.position.y, 
                            odom.pose.pose.position.z])
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
            self.marker_array_line.markers.append(marker_line)  
        
        self.uwb_pub_topic_markers.publish(self.marker_array_line)
        
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
    
    def publish_anchors_marker(self):
        
        # Publisher and subscriber local
        self.uwb_pub_topic_markers = rospy.Publisher(self.pub_topic, MarkerArray, queue_size=10)
        
        # get the ground truth topic and the publish topic
        anchors_topic_local =  self.sub_anchors_name
        self.anchors_sub_local = message_filters.Subscriber(anchors_topic_local, AnchorsInfo)
        
        list_subscribers = [self.anchors_sub_local]
        
        rospy.logwarn([str(tmp.topic) for tmp in list_subscribers])
        ts = message_filters.ApproximateTimeSynchronizer(list_subscribers, queue_size=10, slop=0.1)
        ts.registerCallback(self.vis_anchor_callback)
    
    def vis_anchor_callback(self, *args):
        
        anchors = args[0]
        # rospy.logwarn(anchors)
        
        if self.color[0] == -1:
            color_gen = 1
        else:
            color_gen = 0
            color = tuple(self.color)   
            
        self.marker_array_anchor = MarkerArray()
        
        for i in range(self.N_A):
            
            if color_gen == 1:
                color = self.generate_color(i, self.N_A)     
                
            # set marker
            marker_anchor = Marker()    
            marker_anchor.header.frame_id = "world"
            marker_anchor.header.stamp = rospy.Time.now()
            marker_anchor.ns = "AN" + str(i) + "/anchor_markers"
            marker_anchor.id = i
            marker_anchor.type = Marker.CUBE
            marker_anchor.action = Marker.ADD            
            marker_anchor.scale.x = 0.25
            marker_anchor.scale.y = 0.25
            marker_anchor.scale.z = 0.25
            marker_anchor.color.r = color[0]
            marker_anchor.color.g = color[1]
            marker_anchor.color.b = color[2]
            marker_anchor.color.a = color[3]
            marker_anchor.pose.position.x = anchors.A_POS[3*i+0]
            marker_anchor.pose.position.y = anchors.A_POS[3*i+1]
            marker_anchor.pose.position.z = anchors.A_POS[3*i+2]
            marker_anchor.pose.orientation.x = 0
            marker_anchor.pose.orientation.y = 0
            marker_anchor.pose.orientation.z = 0
            marker_anchor.pose.orientation.w = 1
            self.marker_array_anchor.markers.append(marker_anchor)  
        
        self.uwb_pub_topic_markers.publish(self.marker_array_anchor)
    
    def generate_color(self,index, total_markers):
            hue = index / float(total_markers)  # Vary the hue between 0 and 1
            saturation = 1.0  # Full saturation
            value = 1.0  # Full brightness
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            return rgb + (1.0,)  # Return as (r, g, b, a) tuple with full opacity